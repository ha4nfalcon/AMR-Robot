"""Decentralized AMR agent: senses locally, broadcasts intent, resolves conflicts peer-to-peer.

Conflict resolution (cooperative, no server):
- Each tick robot broadcasts STATE: {pos, next_cells (intent horizon=3), priority, task_id}
- On receive, robot builds a local reservation view of peers' intents.
- Policy ('smart'):
    * If my next cell is in a peer's intent and peer has HIGHER priority -> yield
      OR re-route (A* with contested cells penalized) if wait > threshold.
    * Head-on deadlock in 1-wide corridor (my next == peer pos and vice versa):
      lower priority robot steps aside / waits, higher proceeds.
    * Higher priority never stops for lower priority.
- Priority = (-task_urgency, battery_penalty, robot_id). Lower tuple = higher priority.
  Task urgency = earlier task index wins; tie -> higher battery wins; tie -> lower id.
- Policy ('stopwait' baseline): stop if ANY robot is within 2 cells of my next
  cell. No intent use, no rerouting. This is the traditional method we beat.

Battery drains per move; robot still operates (dashboard shows status).
"""
from dataclasses import dataclass, field
from warehouse_sim.planner import astar
from warehouse_sim.comms import Message

INTENT_HORIZON = 3
YIELD_REROUTE_THRESHOLD = 4  # ticks waited before trying alternate route
LOW_BATTERY = 25.0    # below this -> run the energy planner (trip / top-up?)
CRITICAL_BATTERY = 10.0  # below this -> charge even if the dock is busy
RESUME_CHARGE = 100.0  # default charge target (full green bar)
CHARGE_RATE = 5.0     # battery points per tick on the dock (~3s for full charge)
DRAIN_PER_MOVE = 0.8  # average battery usage per grid cell (estimates)
EMPTY_DRAIN = 0.6     # per-cell drain driving empty
LOADED_DRAIN = 1.0    # per-cell drain carrying a package
ENERGY_MARGIN = 1.5   # safety factor on movement estimates (detours/contention)
CHARGE_BUFFER = 5.0   # extra points on top of a computed charge target

@dataclass
class Robot:
    rid: int
    start: tuple
    wmap: object
    bus: object = None
    inbox: object = None
    policy: str = "smart"  # 'smart' or 'stopwait'
    pos: tuple = None
    path: list = field(default_factory=list)
    goal: tuple = None
    pickup: tuple = None
    drop: tuple = None
    phase: str = "pickup"  # 'pickup' -> go to pickup, then 'drop' -> deliver
    task_id: int = None
    task_urgency: int = 999
    battery: float = 100.0
    dead: bool = False          # True after running dry: frozen wreck
    charging: bool = False      # True while heading to / sitting on dock
    dock_target: object = None  # reserved dock cell while charging
    saved_task: object = None   # stashed (task_id, pickup, drop, phase, urgency)
    queued_charge: bool = False  # True while deferring trip (dock busy, keep working)
    topup_after_pickup: bool = False  # fetch first, then dock for a top-up
    charge_target: float = 100.0  # battery level that ends the dock stay
    wait_ticks: int = 0
    waited_total: int = 0
    replans: int = 0
    done_tasks: int = 0
    peers: dict = field(default_factory=dict)  # rid -> last STATE payload
    blocked_known: set = field(default_factory=set)
    no_route: bool = False  # True when goal provably unreachable (stops replan spam)
    pull_cell: object = None   # deadlock pull-over target (None = not pulling over)
    pull_goal: object = None   # real goal stashed during pull-over
    pull_urg: int = 999        # real urgency stashed during pull-over
    pull_timer: int = 0        # hard cap on pull-over duration
    pull_calm: int = 0         # consecutive calm ticks while pulled over
    log: list = field(default_factory=list)

    def __post_init__(self):
        self.pos = self.start
        if self.bus is not None and self.inbox is None:
            self.inbox = self.bus.register(self.rid)

    # ---- task / planning ----
    def assign(self, task_id, pickup, drop, urgency):
        self.task_id = task_id
        self.pickup, self.drop = pickup, drop
        self.phase = "pickup"
        self.goal = pickup
        self.task_urgency = urgency
        self.topup_after_pickup = False
        self.charge_target = RESUME_CHARGE
        # fresh work cancels any pull-over/clearing detour outright —
        # otherwise its stale restore would wipe this assignment later
        self.pull_cell, self.pull_goal = None, None
        self.pull_urg, self.pull_timer, self.pull_calm = 999, 0, 0
        self.plan(extra_cost=None)

    def plan(self, extra_cost=None):
        p = astar(self.pos, self.goal, self.wmap,
                  blocked=self.blocked_known, extra_cost=extra_cost)
        self.path = p[1:] if len(p) > 1 else ([] if self.pos == self.goal else [])
        self.replans += 1
        unreachable = bool(self.goal is not None and self.pos != self.goal
                           and not self.path)
        if unreachable and not self.no_route:
            self.log.append(f"R{self.rid} NO ROUTE to {self.goal} (waiting)")
        self.no_route = unreachable

    def priority(self):
        # lower tuple => higher priority (min wins)
        batt_pen = 0 if self.battery > 30 else 1
        return (self.task_urgency, batt_pen, self.rid)

    def intent(self, n=INTENT_HORIZON):
        return self.path[:n]

    # ---- comms ----
    def broadcast_state(self):
        # Always broadcast (even when idle/done) so peers never act on stale
        # intent — a finished robot advertises empty intent, freeing the cell.
        if self.bus is None:
            return
        m = Message(sender=self.rid, type="STATE", payload={
            "pos": self.pos, "intent": self.intent() if self.goal else [],
            "priority": self.priority(), "task": self.task_id,
            "battery": round(self.battery, 1),
        })
        self.bus.broadcast(m, pos=self.pos)

    def drain_inbox(self):
        if self.inbox is None:
            return
        while not self.inbox.empty():
            m = self.inbox.get()
            if m.type == "STATE":
                self.peers[m.sender] = m.payload
            elif m.type == "BLOCKED":
                cells = {tuple(c) for c in m.payload.get("cells", [])}
                new = cells - self.blocked_known
                if new:
                    self.blocked_known |= new
                    # replan if the blockage hits my route — or if I was
                    # previously stranded (new info deserves a fresh attempt)
                    if self.goal and (self.no_route or any(c in self.path for c in new)):
                        self.plan()
                        self.log.append(f"R{self.rid} re-routed around {sorted(new)}")

    def announce_blocked(self, cells):
        if self.bus is None:
            return
        self.bus.broadcast(Message(sender=self.rid, type="BLOCKED",
                                   payload={"cells": [list(c) for c in cells]}),
                           pos=self.pos)

    # ---- movement decision ----
    def next_step(self):
        """Return next cell to move to, or None to wait. Handles both policies."""
        if self.dead:
            return None  # dry wreck: never moves again
        if self.goal is None or self.pos == self.goal:
            return None
        if not self.path:
            if self.no_route:
                # last resort: step off magenta I'm parked on, or tunnel one
                # cell out of a pocket whose only exits are blocked cells.
                # Returns None (keep waiting, NO ROUTE status) if hopeless.
                if self.goal is not None:
                    esc = self._emergency_step()
                    if esc is not None:
                        self.no_route = False
                        return esc
                return None
            self.plan()
            if not self.path:
                return None
        nxt = self.path[0]
        if self.policy == "stopwait":
            # baseline (traditional): stop if next cell occupied or any peer
            # intends it. No priority, no rerouting — simple but slow.
            # Deadlock breaker: after 6 yields, force-go (arbitration still
            # serializes by priority, so safety holds but throughput suffers).
            blocked = False
            for rid, p in self.peers.items():
                ppos = tuple(p["pos"])
                pintent = [tuple(c) for c in p.get("intent", [])]
                if ppos == nxt or nxt in pintent:
                    blocked = True
                    break
            if not blocked:
                self.wait_ticks = 0
                return nxt
            self.wait_ticks += 1
            if self.wait_ticks >= 6:
                self.wait_ticks = 0
                return nxt
            return None
        else:
            return self._smart_step(nxt)

    def _smart_step(self, nxt):
        mine = self.priority()
        contested = False
        for rid, p in self.peers.items():
            ppos = tuple(p["pos"])
            pintent = [tuple(c) for c in p.get("intent", [])]
            ppri = tuple(p.get("priority", (999, 0, rid)))
            # case 1: peer currently ON my next cell
            if ppos == nxt:
                if ppri < mine:
                    contested = True  # higher-priority peer there; I yield
                else:
                    # I have priority; peer should move — but avoid ramming:
                    # move only if peer's intent shows it leaving
                    if ppos in pintent or nxt not in pintent:
                        continue
                    contested = True
            # case 2: peer intends my next cell too
            elif nxt in pintent:
                if ppri < mine:
                    contested = True
                # else: I win, keep going
            # case 2b: time-window reservation — my ETA-k cell collides with
            # a higher-priority peer's ETA-j cell (|k-j| <= 1 tick), so we
            # would arrive together. Yield early instead of driving in.
            if not contested and ppri < mine:
                for k, cell in enumerate(self.path[1:4], start=2):
                    for j, pc in enumerate(pintent[:3], start=1):
                        if cell == pc and abs(k - j) <= 1:
                            contested = True
                            break
                    if contested:
                        break
            # case 3: head-on swap (deadlock): my next == peer pos AND peer next == my pos
            if pintent and ppos == nxt and pintent[0] == self.pos:
                if ppri < mine:
                    contested = True
                else:
                    continue  # I proceed, peer yields
        if not contested:
            self.wait_ticks = 0
            return nxt
        # contested: yield, but reroute if waiting too long
        self.wait_ticks += 1
        if self.wait_ticks >= YIELD_REROUTE_THRESHOLD:
            extra = {nxt: 10}
            for p in self.peers.values():
                for c in [tuple(x) for x in p.get("intent", [])]:
                    extra[c] = extra.get(c, 0) + 5
            self.plan(extra_cost=extra)
            self.wait_ticks = 0
            if self.path and self.path[0] != nxt:
                self.log.append(f"R{self.rid} deconflicted via reroute")
                return self.path[0]
            # reroute found no alternative (e.g. goal cell itself contested):
            # force-go if no peer is PHYSICALLY on the cell — intent-only
            # conflicts are safe to override since the arbiter serializes
            # same-cell claims by priority (prevents infinite reroute loops).
            if not any(tuple(p.get("pos")) == nxt for p in self.peers.values()):
                return nxt
        return None

    def _emergency_step(self):
        """Escape hatch, logged — should be rare. Tries each walkable
        neighbor (skipping peer-occupied cells, clean ones first) and keeps
        the first from which a trial A* actually reaches the goal — a step
        off the magenta, or a one-cell tunnel through it. All trial state
        (pos/path/flag/counter/log) is restored. Returns None when truly
        sealed (keep NO ROUTE status instead of wandering)."""
        peers_at = {tuple(p.get("pos")) for p in self.peers.values()
                    if p.get("pos") is not None}

        def grid_free(c):
            return (0 <= c[0] < self.wmap.width and 0 <= c[1] < self.wmap.height
                    and self.wmap.grid[c[1]][c[0]] == 0)

        def key(c):
            dist = abs(c[0] - self.goal[0]) + abs(c[1] - self.goal[1]) \
                if self.goal is not None else 0
            return (c in self.blocked_known, dist)

        nbrs = [(self.pos[0] + dx, self.pos[1] + dy)
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))]
        cands = sorted((c for c in nbrs if grid_free(c) and c not in peers_at),
                       key=key)
        for c in cands:
            saved = (self.pos, self.path, self.no_route, self.replans,
                     len(self.log))
            self.pos = c
            self.plan()
            ok = bool(self.path) or c == self.goal
            self.pos, self.path, self.no_route, self.replans = saved[:4]
            del self.log[saved[4]:]
            if ok:
                self.log.append(f"R{self.rid} EMERGENCY step off blockage -> {c}")
                return c
        return None

    def advance(self, nxt):
        if nxt is None:
            self.waited_total += 1
            return
        self.pos = nxt
        self.path = self.path[1:]
        loaded = self.phase == "drop" and self.task_id is not None
        self.battery = max(0.0, self.battery
                           - (LOADED_DRAIN if loaded else EMPTY_DRAIN))
        self.wait_ticks = 0

    # ---- charging ----
    def clear_task(self):
        """Drop current assignment (task returns to the pool via caller)."""
        self.task_id, self.pickup, self.drop = None, None, None
        self.phase, self.task_urgency = "pickup", 999
        self.goal, self.path = None, []
        self.topup_after_pickup = False

    def leg_cost(self, a, b, margin=ENERGY_MARGIN):
        """Battery points to travel a -> b: A* length x drain-per-move
        x safety margin. inf when unreachable."""
        if a is None or b is None:
            return float("inf")
        if a == b:
            return 0.0
        p = astar(a, b, self.wmap, blocked=self.blocked_known)
        if not p:
            return float("inf")
        return (len(p) - 1) * DRAIN_PER_MOVE * margin

    def charge_needed(self, parts):
        """Battery target covering a chain of legs + buffer, capped at 100."""
        total = 0.0
        for a, b in parts:
            c = self.leg_cost(a, b)
            if c == float("inf"):
                return 100.0
            total += c
        return min(100.0, total + CHARGE_BUFFER)

    def start_charging_trip(self, dock, target=RESUME_CHARGE):
        """Suspend current task and head to the dock (highest priority),
        leaving once battery reaches target."""
        if self.task_id is not None:
            self.saved_task = (self.task_id, self.pickup, self.drop,
                               self.phase, self.task_urgency)
        else:
            self.saved_task = None
        self.charging = True
        self.dock_target = dock
        self.charge_target = min(100.0, target)
        self.task_urgency = -1  # charging trip outranks deliveries
        self.goal = dock
        self.path = []
        self.plan()
        self.log.append(f"R{self.rid} low battery ({self.battery:.0f}%) "
                        f"-> dock (target {self.charge_target:.0f}%)")

    def finish_charging(self):
        """Restore suspended task after recharge."""
        self.charging = False
        self.dock_target = None
        if self.saved_task is not None:
            tid, p, d, ph, urg = self.saved_task
            self.task_id, self.pickup, self.drop = tid, p, d
            self.phase, self.task_urgency = ph, urg
            self.goal = p if ph == "pickup" else d
            self.path = []
            self.plan()
            self.log.append(f"R{self.rid} charged ({self.battery:.0f}%) -> resume T{tid}")
        else:
            self.task_urgency = 999
            self.goal = None
        self.saved_task = None

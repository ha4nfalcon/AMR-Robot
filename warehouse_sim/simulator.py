"""Tick-based fleet simulator: moves robots, checks collisions, injects blocked aisles."""
from warehouse_sim.comms import LocalBus
from warehouse_sim.robot import Robot
from warehouse_sim.allocator import auction
from warehouse_sim.planner import astar

class Simulator:
    def __init__(self, wmap, starts, tasks, policy="smart", block_event=None, max_ticks=600):
        self.wmap = wmap
        self.bus = LocalBus()
        self.robots = [Robot(rid=i, start=s, wmap=wmap, bus=self.bus, policy=policy)
                       for i, s in enumerate(starts)]
        self.tasks = tasks  # list of {id, pickup, drop, urgency, assigned, done}
        self.block_event = block_event  # {tick: [(x,y), ...]}
        self.max_ticks = max_ticks
        self.tick = 0
        self.collisions = 0
        self.reroutes = 0       # contention/blockage replans (dashboard KPI)
        self.deadlocks = 0      # distinct wait-for cycle episodes (dashboard KPI)
        self.recoveries = 0     # pull-over recoveries executed
        self._last_deadlock = None
        self._deadlock_streak = 0  # consecutive ticks with the SAME stuck set
        self._last_involved = frozenset()
        self._prev_positions = {}  # positions at end of last tick (stall check)
        self.events = []        # (tick, text, kind) feed for the dashboard
        self.history = []  # per-tick positions for replay
        self._initial_assign()

    def _initial_assign(self):
        amap = auction(self.tasks, self.robots)
        for t in self.tasks:
            if t["id"] in amap:
                r = self.robots[amap[t["id"]]]
                t["assigned"] = r.rid
                r.assign(t["id"], t["pickup"], t["drop"], t["urgency"])

    def _assign_remaining(self):
        amap = auction(self.tasks, self.robots)
        n = 0
        for t in self.tasks:
            if t.get("assigned") is None and t["id"] in amap:
                r = self.robots[amap[t["id"]]]
                t["assigned"] = r.rid
                r.assign(t["id"], t["pickup"], t["drop"], t["urgency"])
                n += 1
        return n

    def step(self):
        replans_before = sum(r.replans for r in self.robots)
        # 1. dynamic blockage / obstacle-spawn event
        # Hazards never spawn under a robot — occupied cells are excluded.
        if self.block_event and self.tick in self.block_event:
            occupied = {r.pos for r in self.robots}
            cells = set(self.block_event[self.tick]) - occupied
            if cells:
                # first robot to "see" it broadcasts (peer-to-peer BLOCKED msg)
                seer = self.robots[0]
                seer.blocked_known |= cells
                seer.announce_blocked(cells)
                seer.log.append(f"R{seer.rid} detected new obstacle {sorted(cells)}")
                self.events.append((self.tick, f"Obstacle spawn {sorted(cells)}", "warn"))
            # EVERY robot re-evaluates — including the seer itself (it never
            # receives its own broadcast, so without this it would plow its
            # stale path straight through the new blockage). Robots standing
            # ON a fresh block are forced onto a new path off of it.
            for r in self.robots:
                if r.goal is not None and (r.pos in cells or any(c in r.path for c in cells)):
                    r.plan()
        # 2. comms phase: broadcast intents, then receive
        for r in self.robots:
            r.broadcast_state()
        for r in self.robots:
            r.drain_inbox()
        # 2b. battery phase.
        # - robots on the dock charge to full, then resume saved work.
        # - needy robots (lowest battery first) trip to the dock with their
        #   task stashed (carrying robots keep the package) — unless it is
        #   busy, in which case non-critical robots defer and KEEP WORKING.
        from warehouse_sim.robot import (LOW_BATTERY, CRITICAL_BATTERY,
                                         RESUME_CHARGE, CHARGE_RATE)
        dock = self.wmap.dock
        resumes = 0
        for r in self.robots:
            if r.charging and r.pos == dock:
                r.battery = min(100.0, r.battery + CHARGE_RATE)
                if r.battery >= r.charge_target:
                    r.finish_charging()
                    resumes += 1
        needy = sorted((r for r in self.robots
                        if not r.charging and r.battery < LOW_BATTERY),
                       key=lambda r: r.battery)
        # dock exclusion zone: while anyone is charging, idle robots keep
        # off the dock AND its neighboring cells (a squatter one step away
        # pins chargers just as hard — recoveries alone never converge).
        dock_zone = {dock, (dock[0] - 1, dock[1]), (dock[0] + 1, dock[1]),
                     (dock[0], dock[1] - 1), (dock[0], dock[1] + 1)}
        dock_zone = {c for c in dock_zone
                     if 0 <= c[0] < self.wmap.width and 0 <= c[1] < self.wmap.height
                     and self.wmap.grid[c[1]][c[0]] == 0}
        if any(q.charging for q in self.robots):
            for r in self.robots:
                if (r.task_id is None and not r.charging and r.pull_cell is None
                        and r.pos in dock_zone):
                    forbid = (set(dock_zone)
                              | {p for t in self.tasks for p in (t["pickup"], t["drop"])}
                              | {c for q in self.robots if q is not r for c in q.path})
                    walls = frozenset(q.pos for q in self.robots if q is not r)
                    cell = self._pullover_cell(r, forbid, walls)
                    if cell is None:
                        continue
                    r.pull_goal, r.pull_urg = None, 999
                    r.pull_cell, r.pull_timer, r.pull_calm = cell, 60, 0
                    r.task_urgency = -2  # step aside ahead of everyone
                    r.goal, r.path = cell, []
                    r.plan()
                    r.log.append(f"R{r.rid} clearing dock zone -> {cell}")
        for r in needy:
            if r.charging:
                continue  # tripped earlier this same phase
            # Energy planner: price the trip in battery points first.
            if r.task_id is not None and r.phase == "pickup":
                to_pick = r.leg_cost(r.pos, r.pickup)
                to_drop = r.leg_cost(r.pickup, r.drop)
                to_dock = r.leg_cost(r.pickup, dock)
                if r.battery >= to_pick + to_drop:
                    continue  # D1: finishes pickup + delivery, no trip
                if (not r.topup_after_pickup
                        and r.battery >= to_pick + to_dock):
                    # D2: fetch first, then dock for a top-up to the drop
                    r.topup_after_pickup = True
                    r.log.append(
                        f"R{r.rid} will top up after T{r.task_id} pickup")
                    continue
                # D3: cannot even reach the pickup -> fall through to dock
            elif r.task_id is not None:
                # carrying: finish if the drop leg fits, else dock with it
                if r.battery >= r.leg_cost(r.pos, r.drop):
                    continue
            dock_busy = any(q.charging for q in self.robots if q is not r)
            if dock_busy and r.battery >= CRITICAL_BATTERY:
                if not r.queued_charge:
                    r.queued_charge = True
                    r.log.append(f"R{r.rid} dock busy ({r.battery:.0f}%) -> working on")
                continue
            r.queued_charge = False
            if r.task_id is not None and r.phase == "pickup":
                # safety: never dock holding an unstarted task
                tid = r.task_id
                for t in self.tasks:
                    if t["id"] == tid and not t.get("done"):
                        t["assigned"] = None
                r.clear_task()
                r.log.append(f"R{r.rid} low battery -> released T{tid} to pool")
                target = 100.0
            else:
                # charge to whatever the remaining job needs from the dock
                goal = r.drop if r.phase == "drop" else None
                legs = ([(dock, goal)] if goal is not None
                        else [(dock, r.pickup), (r.pickup, r.drop)]
                        if r.pickup is not None else [])
                target = r.charge_needed(legs) if legs else 100.0
            r.start_charging_trip(dock, target)
        # 3. decide moves (decentralized — each robot decides locally)
        # Priority-ordered execution emulates the converged P2P reservation:
        # all robots share the same intent view + same priority rule, so all
        # compute the same winner deterministically without a server.
        order = sorted(self.robots, key=lambda r: r.priority())
        moves = {}
        claimed: dict = {}  # target cell -> winner rid (stays + moves of higher-priority)
        # Pass 1: raw proposals in priority order; higher may claim an
        # occupied cell (lower must vacate — handled below).
        raw = {}
        for r in order:
            raw[r.rid] = r.next_step()
        for r in order:
            m = raw[r.rid]
            if m is None:
                # staying — but if a higher-priority robot already claimed my
                # cell (moving into me), I must vacate via sidestep.
                if r.pos in claimed:
                    esc = self._escape_cell(r, claimed)
                    if esc is not None:
                        # vacate: move aside (replan next tick)
                        claimed[esc] = r.rid
                        moves[r.rid] = esc
                        r.path = []  # force replan from new cell
                    else:
                        # cannot vacate: revert the intruder to keep zero collisions
                        intruder = claimed[r.pos]
                        moves[intruder] = None
                        for rr in self.robots:
                            if rr.rid == intruder:
                                rr.wait_ticks += 1
                                claimed.pop(r.pos, None)
                                claimed.setdefault(rr.pos, rr.rid)
                                break
                        claimed.setdefault(r.pos, r.rid)
                        moves[r.rid] = None
                else:
                    claimed.setdefault(r.pos, r.rid)
                    moves[r.rid] = None
                continue
            if m in claimed:
                r.wait_ticks += 1
                if r.pos in claimed:
                    # higher-priority robot is moving into me: vacate or revert it
                    esc = self._escape_cell(r, claimed)
                    if esc is not None:
                        claimed[esc] = r.rid
                        moves[r.rid] = esc
                        r.path = []
                    else:
                        intruder = claimed[r.pos]
                        moves[intruder] = None
                        for rr in self.robots:
                            if rr.rid == intruder:
                                rr.wait_ticks += 1
                                claimed.pop(r.pos, None)
                                claimed.setdefault(rr.pos, rr.rid)
                                break
                        claimed.setdefault(r.pos, r.rid)
                        moves[r.rid] = None
                else:
                    claimed.setdefault(r.pos, r.rid)
                    moves[r.rid] = None
                continue
            # edge-swap: if higher moves into my pos while I move into theirs,
            # lower (me, later in order) yields — higher already decided.
            swap = False
            for higher in order:
                if higher.rid == r.rid:
                    break
                if moves.get(higher.rid) == r.pos and higher.pos == m:
                    swap = True
                    break
            if swap:
                r.wait_ticks += 1
                # swap-yield means I stay while higher moves into me -> vacate
                if r.pos in claimed:
                    esc = self._escape_cell(r, claimed)
                    if esc is not None:
                        claimed[esc] = r.rid
                        moves[r.rid] = esc
                        r.path = []
                    else:
                        intruder = claimed[r.pos]
                        moves[intruder] = None
                        for rr in self.robots:
                            if rr.rid == intruder:
                                rr.wait_ticks += 1
                                claimed.pop(r.pos, None)
                                claimed.setdefault(rr.pos, rr.rid)
                                break
                        claimed.setdefault(r.pos, r.rid)
                        moves[r.rid] = None
                else:
                    claimed.setdefault(r.pos, r.rid)
                    moves[r.rid] = None
                continue
            claimed[m] = r.rid
            moves[r.rid] = m
        # 3b. deadlock instrumentation: wait-for cycle among yielding robots.
        # Read-only (resolution is the priority arbitration above); feeds the
        # dashboard KPI + event feed.
        wait_for = {}
        for r in order:
            if moves.get(r.rid) is None and r.goal is not None and r.path:
                nxt = r.path[0]
                blk = claimed.get(nxt)
                if blk is not None and blk != r.rid:
                    wait_for[r.rid] = blk
                else:
                    for q in self.robots:
                        if q.rid == r.rid:
                            continue
                        if q.pos == nxt or nxt in q.intent():
                            wait_for[r.rid] = q.rid
                            break
        cycle = None
        seen_all = set()
        for start in list(wait_for):
            seen, cur = set(), start
            while cur in wait_for and cur not in seen and cur not in seen_all:
                seen.add(cur)
                cur = wait_for[cur]
            seen_all |= seen
            if cur in seen:
                cyc = []
                c = cur
                while True:
                    cyc.append(c)
                    c = wait_for[c]
                    if c == cur:
                        break
                cycle = tuple(sorted(cyc))
                break
        if cycle and cycle != self._last_deadlock:
            self.deadlocks += 1
            self.events.append((self.tick,
                                f"Deadlock {list(cycle)} — priority arbitration resolves",
                                "alert"))
            for r in self.robots:
                if r.rid in cycle:
                    r.log.append(f"DEADLOCK {list(cycle)}: R{min(cycle)} keeps priority")
        self._last_deadlock = cycle  # episode counting only; recovery uses the stall set
        # stall set: everyone waiting on someone (edge direction may morph
        # tick to tick while the same robots stay stuck — track the SET)
        involved_now = frozenset(wait_for) | frozenset(wait_for.values())
        self._last_involved = involved_now
        # 3c. pull-over recovery (smart policy only): a wait-for cycle that
        # survives 12+ ticks cannot clear itself — someone must back out.
        # Lowest-priority capable member retreats to a nearby roomy cell at
        # yield priority until the cluster drains, then resumes its goal.
        # (The vacate-or-revert rule alone re-locks these every tick.)
        self._pullover_update(wait_for)
        # stall streak: same robots tangled with nobody moving. Resets when
        # the tangle clears; decays when there is motion (busy but alive).
        moved = any(r.pos != self._prev_positions.get(r.rid) for r in self.robots)
        if involved_now and not moved:
            self._deadlock_streak += 1
        elif not involved_now:
            self._deadlock_streak = 0
        else:
            self._deadlock_streak = max(0, self._deadlock_streak - 2)
        if (involved_now and self._deadlock_streak >= 12
                and not any(r.pull_cell is not None for r in self.robots)
                and self.robots and self.robots[0].policy == "smart"):
            members = [r for r in self.robots if r.rid in involved_now]
            # Who steps aside? Queuing chargers first (they're just waiting —
            # e.g. two chargers pinning a robot onto the dock), then workers
            # lowest-priority-first, on-dock chargers last. Entombed members
            # are skipped via the walls check below.
            def pull_rank(r):
                if r.charging and r.pos != self.wmap.dock:
                    group = 0
                elif not r.charging:
                    group = 1
                else:
                    group = 2
                return (group, tuple(-v for v in r.priority()))
            members.sort(key=pull_rank)
            forbid = {self.wmap.dock}
            forbid |= {p for t in self.tasks for p in (t["pickup"], t["drop"])}
            forbid |= {c for q in self.robots for c in q.path}
            for m in members:
                # fellow members are walls: only a robot that can actually
                # get out (not the entombed one) is sent to pull over
                walls = frozenset(q.pos for q in members if q is not m)
                cell = self._pullover_cell(m, forbid, walls)
                if cell is None:
                    continue
                m.pull_goal, m.pull_urg = m.goal, m.task_urgency
                m.pull_cell, m.pull_timer, m.pull_calm = cell, 60, 0
                m.task_urgency = 501  # yield to real work while hiding
                m.goal, m.path = cell, []
                m.plan()
                m.log.append(f"DEADLOCK recovery: R{m.rid} pulls over to {cell}")
                self.events.append(
                    (self.tick, f"Stall {sorted(involved_now)}: R{m.rid} pulls over",
                     "warn"))
                self.recoveries += 1
                break
        # 4. collision check BEFORE applying (vertex + edge-swap) — safety monitor
        positions = {r.rid: r.pos for r in self.robots}
        targets = {rid: (moves[rid] or positions[rid]) for rid in positions}
        # vertex collision: two robots target same cell
        seen = {}
        for rid, t in targets.items():
            if t in seen:
                self.collisions += 1
            seen.setdefault(t, rid)
        # edge swap: A->B while B->A
        rids = list(positions.keys())
        for i in range(len(rids)):
            for j in range(i + 1, len(rids)):
                a, b = rids[i], rids[j]
                if targets[a] == positions[b] and targets[b] == positions[a]:
                    self.collisions += 1
        # 5. apply moves
        for r in self.robots:
            r.advance(moves[r.rid])
            if r.goal is not None and r.pos == r.goal:
                if r.charging:
                    pass  # dock arrival: battery phase handles it, task untouched
                elif r.task_id is None:
                    r.goal, r.path = None, []  # clearing nudge arrived; stay idle
                elif r.phase == "pickup":
                    # arrived at pickup: collect, then either deliver or
                    # (top-up plan) dock first for exactly what delivery needs
                    r.phase = "drop"
                    r.goal = r.drop
                    r.plan()
                    for t in self.tasks:
                        if t["id"] == r.task_id and not t.get("done"):
                            t["picked"] = True
                            t["carrier"] = r.rid
                    if r.topup_after_pickup:
                        r.topup_after_pickup = False
                        target = r.charge_needed([(dock, r.drop)])
                        r.start_charging_trip(dock, target)
                        r.log.append(f"R{r.rid} topped up to {target:.0f}% "
                                     f"for T{r.task_id} delivery")
                    else:
                        r.log.append(f"R{r.rid} picked up T{r.task_id} -> delivering")
                else:
                    for t in self.tasks:
                        if t["id"] == r.task_id and not t.get("done"):
                            t["done"] = True
                            r.done_tasks += 1
                    r.task_id, r.goal = None, None
                    r.pickup, r.drop, r.phase = None, None, "pickup"
                    r.path = []
                    r.task_urgency = 999  # idle: lowest priority, vacates for active robots
        assigns = self._assign_remaining()
        self.history.append({r.rid: r.pos for r in self.robots})
        self._prev_positions = {r.rid: r.pos for r in self.robots}
        self.tick += 1
        # contention/blockage replans = all replans minus fresh task
        # assignments and charge-resume plans (those are routine, not reroutes)
        self.reroutes += max(0, sum(r.replans for r in self.robots)
                             - replans_before - assigns - resumes)

    def _pullover_cell(self, robot, forbid, block_pos=frozenset()):
        """Nearest cell with room to wait (>=2 free neighbors), BFS over
        free non-blocked cells. block_pos are treated as walls — callers
        pass the OTHER stuck members' cells, so the chosen robot is one
        that can actually get out (not the entombed one). Never own cell."""
        from collections import deque
        seen = {robot.pos}
        dq = deque([robot.pos])
        while dq:
            c = dq.popleft()
            if c != robot.pos and c not in forbid:
                free_n = 0
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    n = (c[0] + dx, c[1] + dy)
                    if (0 <= n[0] < self.wmap.width and 0 <= n[1] < self.wmap.height
                            and self.wmap.grid[n[1]][n[0]] == 0
                            and n not in robot.blocked_known
                            and n not in block_pos):
                        free_n += 1
                if free_n >= 2:
                    return c
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (c[0] + dx, c[1] + dy)
                if (n not in seen and n not in block_pos
                        and 0 <= n[0] < self.wmap.width
                        and 0 <= n[1] < self.wmap.height
                        and self.wmap.grid[n[1]][n[0]] == 0
                        and n not in robot.blocked_known):
                    seen.add(n)
                    dq.append(n)
        return None

    def _pullover_update(self, wait_for):
        """Progress active pull-overs; release on calm cluster or timeout."""
        involved = set(wait_for) | set(wait_for.values())
        for r in self.robots:
            if r.pull_cell is None:
                continue
            r.pull_timer -= 1
            if r.pos == r.pull_cell:
                # dock-clearing nudge holds nothing to resume: release at once
                if r.pull_goal is None and r.task_id is None:
                    r.goal, r.task_urgency, r.path = None, 999, []
                    r.pull_cell, r.pull_goal, r.pull_calm = None, None, 0
                    r.pull_timer = 0
                    continue
                r.pull_calm = r.pull_calm + 1 if r.rid not in involved else 0
            if (r.pos == r.pull_cell and r.pull_calm >= 3) or r.pull_timer <= 0:
                r.goal, r.task_urgency, r.path = r.pull_goal, r.pull_urg, []
                r.pull_cell, r.pull_goal, r.pull_calm = None, None, 0
                r.log.append(f"R{r.rid} pull-over done -> resume {r.goal}")

    def _escape_cell(self, robot, claimed):
        """Find a free neighbor for a robot forced to vacate its cell."""
        occupied = {r.pos for r in self.robots}
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            c = (robot.pos[0] + dx, robot.pos[1] + dy)
            if not self.wmap.is_free(*c, blocked=robot.blocked_known):
                continue
            if c in occupied or c in claimed:
                continue
            return c
        return None

    def all_done(self):
        return all(t.get("done") for t in self.tasks)

    def run(self, verbose=False):
        while self.tick < self.max_ticks and not self.all_done():
            self.step()
        makespan = self.tick
        waits = {r.rid: r.waited_total for r in self.robots}
        if verbose:
            print(f"policy={self.robots[0].policy} makespan={makespan} "
                  f"collisions={self.collisions} waits={waits}")
        return {"makespan": makespan, "collisions": self.collisions,
                "waits": waits, "ticks": self.tick,
                "reroutes": self.reroutes, "deadlocks": self.deadlocks,
                "recoveries": self.recoveries,
                "replans": {r.rid: r.replans for r in self.robots}}


def _check_disjoint(starts, pairs, dock, where):
    """Guarantee robots never spawn on pickups/drops/dock and no two
    markers share a cell. Raises loudly instead of shipping a bad map."""
    starts, dock = set(starts), tuple(dock)
    seen = {}
    problems = []
    if dock in starts:
        problems.append(f"dock {dock} under a robot start")
    for tid, (p, d) in enumerate(pairs):
        for cell, kind in ((p, "pickup"), (d, "drop")):
            if cell in starts:
                problems.append(f"T{tid} {kind} {cell} under a robot start")
            if cell == dock:
                problems.append(f"T{tid} {kind} {cell} on the dock")
            if cell in seen:
                problems.append(f"T{tid} {kind} {cell} stacks on {seen[cell]}")
            else:
                seen[cell] = f"T{tid} {kind}"
    if problems:
        raise AssertionError(f"[{where}] bad task layout: " + "; ".join(problems))


def make_fixed_scenario(policy="smart"):
    """Deterministic overlapping-paths stress test (success-criteria proof).

    Each task = pickup (red) -> drop-off (green); pairs span the map so
    routes cross at the center choke.
    """
    from warehouse_sim.config import build_default_map
    wmap = build_default_map()
    starts = [(1, 1), (26, 1), (1, 16)]
    pairs = [((2, 2), (25, 15)), ((2, 15), (25, 2)), ((25, 5), (2, 14)),
             ((9, 1), (14, 16)), ((2, 8), (25, 11))]
    _check_disjoint(starts, pairs, wmap.dock, "fixed")
    tasks = [
        {"id": i, "pickup": p, "drop": d, "urgency": i, "assigned": None, "done": False, "picked": False, "carrier": None}
        for i, (p, d) in enumerate(pairs)
    ]
    # t=40: blocked aisle (x=5, gap at y=8 keeps a detour open).
    # t=55/t=75: sudden obstacles (spill, fallen pallet) spawn mid-run.
    block_event = {
        40: [(5, y) for y in range(1, 17) if y != 8],
        55: [(15, 5), (16, 5)],
        75: [(7, 14), (8, 14)],
    }
    sim = Simulator(wmap, starts, tasks, policy=policy, block_event=block_event)
    # demo: R2 starts low so it must dock mid-run
    for r, b in zip(sim.robots, (100.0, 100.0, 35.0)):
        r.battery = b
    return sim


def make_scenario(policy="smart", seed=None, n_tasks=5):
    import random
    from warehouse_sim.config import build_default_map
    from warehouse_sim.planner import astar
    rng = random.Random(seed) if seed is not None else random.Random()
    wmap = build_default_map()
    free = [(x, y) for y in range(wmap.height) for x in range(wmap.width)
            if wmap.grid[y][x] == 0]
    # blocked aisle cells (mid-run event) — keep goals/starts clear of these
    blocked_cells = {(5, y) for y in range(1, wmap.height - 1) if y != 8}  # gap at y=8: detour stays open
    usable = [c for c in free if c not in blocked_cells and c != wmap.dock]
    # Random every run, but biased so paths overlap through the center choke
    # (else random maps often have zero contention and prove nothing).
    # 70%: pickups on one side, drop-offs on the opposite side -> forced crossing.
    starts, pairs = [], []
    if rng.random() < 0.7:
        left = [c for c in usable if c[0] <= 5]
        right = [c for c in usable if c[0] >= 22]
        top = [c for c in usable if c[1] <= 3]
        bottom = [c for c in usable if c[1] >= 14]
        if rng.random() < 0.5 and len(left) >= 3 and len(right) >= n_tasks:
            starts = rng.sample(left, 3)
            pick_pool = [c for c in left if c not in starts]
            drop_pool = [c for c in right if c not in starts]
        else:
            starts = rng.sample(top + bottom or usable, 3)
            pick_pool = [c for c in usable if c not in starts and abs(c[0] - starts[0][0]) >= 12]
            drop_pool = [c for c in usable if c not in starts]
            if len(pick_pool) < n_tasks:
                pick_pool = [c for c in usable if c not in starts]
        rng.shuffle(pick_pool)
        rng.shuffle(drop_pool)
        # used = every taken cell: starts, dock, chosen pickups AND drops —
        # no robot spawns on a marker, no two markers stack.
        used = set(starts) | {wmap.dock}
        for p in pick_pool:
            if len(pairs) >= n_tasks:
                break
            if p in used:
                continue
            if not any(astar(s, p, wmap) for s in starts):
                continue
            for d in drop_pool:
                if d in used or d == p:
                    continue
                if astar(p, d, wmap):
                    pairs.append((p, d))
                    used |= {p, d}
                    break
    if not starts or len(pairs) < n_tasks:
        # fallback: fully random reachable pickup->drop pairs
        starts = rng.sample(usable, 3)
        pairs = []
        used = set(starts) | {wmap.dock}
        cands = [c for c in usable if c not in starts]
        rng.shuffle(cands)
        for p in cands:
            if len(pairs) >= n_tasks:
                break
            if p in used:
                continue
            if not any(astar(s, p, wmap) for s in starts):
                continue
            drops = [c for c in usable if c not in starts and c != p]
            rng.shuffle(drops)
            for d in drops:
                if d in used:
                    continue
                if astar(p, d, wmap):
                    pairs.append((p, d))
                    used |= {p, d}
                    break
    _check_disjoint(starts, pairs, wmap.dock, "random")
    tasks = [
        {"id": i, "pickup": p, "drop": d, "urgency": i, "assigned": None, "done": False, "picked": False, "carrier": None}
        for i, (p, d) in enumerate(pairs)
    ]
    # Dynamic obstacle spawns (spill / fallen pallet): small 2-3 cell patches
    # popping up mid-run at random free cells, away from tasks/dock/starts
    # and never in the 1-wide choke (a patch there would seal the map).
    task_cells = set(starts) | {wmap.dock} | set(blocked_cells)
    for p, d in pairs:
        task_cells |= {p, d}
    choke = set(wmap.choke_cells or [])
    # NOTE: the whole x=5 column is off-limits — the t=40 aisle block leaves
    # exactly one gap at (5,8), and a spawn covering it would seal the map.
    spawn_cands = [c for c in usable
                   if c not in task_cells and c not in choke and c[0] != 5]
    rng.shuffle(spawn_cands)
    spawns = {}
    for tick in (70, 120):
        for c in spawn_cands:
            if c in task_cells:
                continue
            # grow a 2-3 cell patch right/down from c while cells stay valid
            patch = [c]
            for nb in ((c[0] + 1, c[1]), (c[0], c[1] + 1)):
                if len(patch) >= 3:
                    break
                if nb in usable and nb not in task_cells \
                        and nb not in choke and nb[0] != 5:
                    patch.append(nb)
            if len(patch) < 2:
                continue
            # backstop: only keep the spawn if every task cell AND the dock
            # stay reachable from every start with it applied
            trial = set(blocked_cells) | {x for v in spawns.values() for x in v} | set(patch)
            keep = True
            for s in starts:
                for cell in [p for pair in pairs for p in pair] + [wmap.dock]:
                    if not astar(s, cell, wmap, blocked=trial):
                        keep = False
                        break
                if not keep:
                    break
            if keep:
                spawns[tick] = patch
                task_cells |= set(patch)
                break
    # blocked aisle mid-run (t=40) + sudden obstacle spawns (t=70, t=120)
    block_event = {40: sorted(blocked_cells)}
    block_event.update(spawns)
    sim = Simulator(wmap, starts, tasks, policy=policy, block_event=block_event)
    # random starting charge so docking trips happen on some runs
    for r in sim.robots:
        r.battery = round(rng.uniform(50.0, 100.0), 1)
    return sim

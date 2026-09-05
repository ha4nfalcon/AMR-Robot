"""Decentralized P2P communication stack.

Design goal: NO central server. Each robot broadcasts its state/intent
locally (edge). Two transports are provided:

1. LocalBus (in-process pub/sub) — used for single-machine simulation.
   API-identical to a network bus: robots only call broadcast()/inbox.
   Models a REAL radio channel: per-message loss, fixed delivery delay,
   and dead zones (rects where radios go deaf). Metrics: .dropped.
2. UdpPeer (real socket broadcast) — drop-in for Raspberry Pi / Jetson Nano.
   Run one instance per robot (even across Pis on same LAN) and messages
   propagate without any server. JSON-encoded, <1KB, 5-10 Hz. On hardware,
   loss/delay/dead-zones are LAN properties, not simulated.

Message types: STATE (pos/intent), TASK_BID, TASK_ASSIGN, BLOCKED, HEARTBEAT.
"""
import json
import random
import socket
import threading
import time
import queue
from dataclasses import dataclass, field


@dataclass
class Message:
    sender: int
    type: str            # STATE, BLOCKED, TASK_BID, TASK_ANNOUNCE, HEARTBEAT
    payload: dict = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


class LocalBus:
    """In-process P2P bus simulating a real radio channel (no central node).

    drop_rate: per-receiver loss probability. delay_ticks: delivery lag.
    dead_zones: [(x0,y0,x1,y1), ...] — senders inside cannot transmit;
    receivers inside hold messages until they drive out (TTL, then drop).
    """
    HOLD_TTL = 10

    def __init__(self, drop_rate=0.0, delay_ticks=0, dead_zones=(), seed=None):
        self._subs: dict[int, queue.Queue] = {}
        self.drop_rate = drop_rate
        self.delay_ticks = delay_ticks
        self.dead_zones = [tuple(z) for z in dead_zones]
        self._pending: list = []  # [deliver_tick, rid, msg, holds]
        self._now = 0
        self._rng = random.Random(seed)
        self.dropped = 0  # lost-message counter (dashboard honesty metric)

    def register(self, robot_id: int) -> queue.Queue:
        q: queue.Queue = queue.Queue()
        self._subs[robot_id] = q
        return q

    def in_dead_zone(self, pos) -> bool:
        if pos is None:
            return False
        x, y = pos
        return any(x0 <= x <= x1 and y0 <= y <= y1
                   for x0, y0, x1, y1 in self.dead_zones)

    def broadcast(self, msg: Message, pos=None):
        # peer-to-peer fan-out: every robot except sender gets a copy,
        # unless the sender is deaf (dead zone) or the packet is lost.
        if self.in_dead_zone(pos):
            self.dropped += len(self._subs) - 1
            return
        for rid, q in self._subs.items():
            if rid != msg.sender:
                if self._rng.random() < self.drop_rate:
                    self.dropped += 1
                    continue
                if self.delay_ticks > 0:
                    self._pending.append(
                        [self._now + self.delay_ticks, rid, msg, 0])
                else:
                    q.put(msg)

    def pump(self, tick, positions):
        """Deliver due messages; hold those addressed into dead zones."""
        self._now = tick
        still = []
        for due, rid, msg, holds in self._pending:
            if due > tick:
                still.append([due, rid, msg, holds])
                continue
            if self.in_dead_zone(positions.get(rid)):
                if holds >= self.HOLD_TTL:
                    self.dropped += 1
                else:
                    still.append([tick + 1, rid, msg, holds + 1])
                continue
            self._subs[rid].put(msg)
        self._pending = still

    def peer_count(self) -> int:
        return len(self._subs)


class UdpPeer:
    """Real UDP-broadcast peer for edge hardware (Pi/Jetson).

    Usage on each robot:
        peer = UdpPeer(robot_id=1, port=5005)
        peer.start(on_message_callback)
        peer.send({"type": "STATE", ...})
    No server, no broker — pure broadcast on LAN.
    """
    def __init__(self, robot_id: int, port: int = 5005):
        self.robot_id = robot_id
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind(("", port))
        except OSError:
            pass  # another peer on same host; still can send
        self.sock.settimeout(0.2)
        self._running = False

    def send(self, msg_dict: dict):
        msg_dict["sender"] = self.robot_id
        data = json.dumps(msg_dict).encode()[:1400]
        try:
            self.sock.sendto(data, ("<broadcast>", self.port))
        except OSError:
            pass

    def start(self, on_msg, rate_hz: float = 5.0):
        self._running = True
        def loop():
            while self._running:
                try:
                    data, _ = self.sock.recvfrom(2048)
                    m = json.loads(data.decode())
                    if m.get("sender") != self.robot_id:
                        on_msg(m)
                except (socket.timeout, ValueError, OSError):
                    pass
        t = threading.Thread(target=loop, daemon=True)
        t.start()
        return t

    def stop(self):
        self._running = False
        try:
            self.sock.close()
        except OSError:
            pass

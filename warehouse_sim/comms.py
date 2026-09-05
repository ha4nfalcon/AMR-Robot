"""Decentralized P2P communication stack.

Design goal: NO central server. Each robot broadcasts its state/intent
locally (edge). Two transports are provided:

1. LocalBus (in-process pub/sub) — used for single-machine simulation.
   API-identical to a network bus: robots only call broadcast()/inbox.
2. UdpPeer (real socket broadcast) — drop-in for Raspberry Pi / Jetson Nano.
   Run one instance per robot (even across Pis on same LAN) and messages
   propagate without any server. JSON-encoded, <1KB, 5-10 Hz.

Message types: STATE (pos/intent), TASK_BID, TASK_ASSIGN, BLOCKED, HEARTBEAT.
"""
import json
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
    """In-process P2P bus simulating local broadcast (no central node)."""
    def __init__(self):
        self._subs: dict[int, queue.Queue] = {}

    def register(self, robot_id: int) -> queue.Queue:
        q: queue.Queue = queue.Queue()
        self._subs[robot_id] = q
        return q

    def broadcast(self, msg: Message):
        # peer-to-peer fan-out: every robot except sender gets a copy
        for rid, q in self._subs.items():
            if rid != msg.sender:
                q.put(msg)

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

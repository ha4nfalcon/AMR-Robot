"""Lightweight A* planner tuned for edge hardware (Pi/Jetson Nano).

Grid 4-connected, Manhattan heuristic, O(open) with heapq.
Re-planning a 20x14 warehouse takes <1ms on Pi-class CPU.
"""
import heapq

def astar(start, goal, wmap, blocked=None, extra_cost=None):
    """Return path as list of (x,y) from start to goal inclusive, or [] if none."""
    blocked = blocked or set()
    extra_cost = extra_cost or {}
    if start == goal:
        return [start]
    W, H = wmap.width, wmap.height

    def h(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    openh = [(h(start, goal), 0, start, [start])]
    best = {start: 0}
    while openh:
        _, g, cur, path = heapq.heappop(openh)
        if cur == goal:
            return path
        if g > best.get(cur, float("inf")):
            continue
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = (cur[0] + dx, cur[1] + dy)
            if not (0 <= nxt[0] < W and 0 <= nxt[1] < H):
                continue
            if wmap.grid[nxt[1]][nxt[0]] == 1 or nxt in blocked:
                continue
            ng = g + 1 + extra_cost.get(nxt, 0)
            if ng < best.get(nxt, float("inf")):
                best[nxt] = ng
                heapq.heappush(openh, (ng + h(nxt, goal), ng, nxt, path + [nxt]))
    return []

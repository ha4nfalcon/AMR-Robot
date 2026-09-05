"""Decentralized task allocation via single-round auction (no central scheduler).

Each round, every free robot bids its A* distance to every unassigned
pickup; the globally nearest (robot, pickup) pair wins, then the next —
robots always chase their nearest pickup regardless of charge level.
Ties -> lower robot id, then lower task id.
"""
from warehouse_sim.planner import astar

def bid(robot, task):
    """A* distance robot -> pickup (pure proximity, charge ignored)."""
    p1 = astar(robot.pos, task["pickup"], robot.wmap, blocked=robot.blocked_known)
    if not p1:
        return float("inf")
    return len(p1)

def auction(tasks, robots):
    """tasks: list of dicts {id, pickup, drop, urgency, assigned}.
    Greedy global nearest-pickup matching. Returns {task_id: rid}."""
    assign = {}
    free = [r for r in robots
            if not r.charging and (r.goal is None or r.pos == r.goal)]
    open_tasks = [t for t in tasks if t.get("assigned") is None]
    while open_tasks and free:
        best = None  # ((cost, rid, tid), robot, task)
        for t in open_tasks:
            for r in free:
                key = (bid(r, t), r.rid, t["id"])
                if best is None or key < best[0]:
                    best = (key, r, t)
        if best is None or best[0][0] == float("inf"):
            break  # nothing reachable right now
        _, r, t = best
        assign[t["id"]] = r.rid
        free.remove(r)
        open_tasks.remove(t)
    return assign

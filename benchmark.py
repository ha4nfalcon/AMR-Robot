"""Benchmark: smart decentralized policy vs traditional stop-and-wait.

A) Fixed overlapping-paths stress scenario (deterministic success-criteria proof).
B) Mean over N_SEEDS random scenarios (robustness; random maps often have
   little contention, so per-seed reduction varies — expected).
Success criteria: zero collisions + >=20% makespan reduction on overlapping paths.
"""
import random
from warehouse_sim.simulator import make_fixed_scenario, make_scenario

N_SEEDS = 5

def run_fixed(policy):
    return make_fixed_scenario(policy=policy).run()

def run_random(policy, seed):
    return make_scenario(policy=policy, seed=seed).run()

if __name__ == "__main__":
    smart = run_fixed("smart")
    base = run_fixed("stopwait")
    red = 100 * (base["makespan"] - smart["makespan"]) / max(base["makespan"], 1)
    cap = " [STOPWAIT DEADLOCKED at 600-tick cap]" if base["makespan"] >= 600 else ""
    print(f"FIXED overlapping: SMART makespan={smart['makespan']} coll={smart['collisions']} | "
          f"STOPWAIT makespan={base['makespan']} coll={base['collisions']} | reduction={red:.1f}%{cap}")
    ok_fixed = smart["collisions"] == 0 and red >= 20.0
    print("FIXED: " + ("PASS" if ok_fixed else "FAIL"))

    seeds = [random.randint(0, 10**6) for _ in range(N_SEEDS)]
    reds, scoll = [], 0
    for s in seeds:
        a = run_random("smart", s)
        b = run_random("stopwait", s)
        r = 100 * (b["makespan"] - a["makespan"]) / max(b["makespan"], 1)
        reds.append(r)
        scoll += a["collisions"]
        print(f"seed={s} SMART={a['makespan']}/{a['collisions']} STOPWAIT={b['makespan']}/{b['collisions']} red={r:.1f}%")
    print(f"Random mean reduction: {sum(reds)/len(reds):.1f}%, SMART collisions: {scoll}")
    print("OVERALL: " + ("PASS" if (ok_fixed and scoll == 0) else "FAIL"))

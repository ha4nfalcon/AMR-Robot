"""Entry point: run simulation with live dashboard or headless."""
import argparse
from warehouse_sim.simulator import make_scenario
from warehouse_sim.dashboard import Dashboard

def main():
    ap = argparse.ArgumentParser(description="Decentralized AMR fleet sim (edge-ready, no cloud)")
    ap.add_argument("--policy", choices=["smart", "stopwait"], default="smart")
    ap.add_argument("--headless", action="store_true", help="run without UI")
    args = ap.parse_args()
    sim = make_scenario(policy=args.policy)
    if args.headless:
        res = sim.run(verbose=True)
        print(res)
    else:
        Dashboard(sim).show()

if __name__ == "__main__":
    main()

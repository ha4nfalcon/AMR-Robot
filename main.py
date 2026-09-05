"""Entry point: run simulation with live dashboard or headless."""
import argparse
from warehouse_sim.simulator import make_scenario
from warehouse_sim.dashboard import Dashboard

def main():
    ap = argparse.ArgumentParser(description="Decentralized AMR fleet sim (edge-ready, no cloud)")
    ap.add_argument("--policy", choices=["smart", "stopwait"], default="smart")
    ap.add_argument("--headless", action="store_true", help="run without UI")
    ap.add_argument("--ui", choices=["tk", "ctk"], default="tk",
                    help="dashboard flavour (ctk needs: pip install customtkinter)")
    args = ap.parse_args()
    sim = make_scenario(policy=args.policy)
    if args.headless:
        res = sim.run(verbose=True)
        print(res)
    elif args.ui == "ctk":
        try:
            from warehouse_sim.dashboard_ctk import CtkDashboard
        except ImportError:
            raise SystemExit("customtkinter missing: pip install customtkinter")
        CtkDashboard(sim).show()
    else:
        Dashboard(sim).show()

if __name__ == "__main__":
    main()

# Decentralized AMR Fleet — Edge Coordination & Collision Avoidance

Peer-to-peer multi-robot warehouse simulation (28×18). No cloud, no central server.
Each AMR runs its own A* planner + P2P intent sharing, exactly as it would on
a Raspberry Pi / Jetson Nano. Fixed machinery/pallet obstacles plus spill and
fallen-pallet patches that spawn mid-run keep every run dynamic.

## Mapping to problem statement
1. **Decentralized Communication** — `warehouse_sim/comms.py`: `LocalBus` (sim)
   + `UdpPeer` (real UDP broadcast sockets for Pi/Jetson LAN, JSON <1KB, 5–10Hz).
   Robots share `STATE {pos, intent[3], priority, battery, task}` + `BLOCKED` alerts.
   The sim channel is realistic: per-message loss, delivery delay, and Wi-Fi
   dead zones (see `DEAD_ZONES`); drops are counted (`comm_dropped` metric).
2. **Dynamic Conflict Resolution** — `robot.py` + `simulator.py`: time-window
   intent reservations (ETA-matched cells), priority `(urgency, battery, id)`,
   yield-or-reroute after 4 waits, head-on deadlock vacate (sidestep / revert),
   pull-over recovery with stall-set tracking. Zero collisions verified.
3. **Task Allocation & Re-routing** — `allocator.py`: decentralized single-round
   auction (nearest-pickup greedy matching + low-battery dock-detour factor);
   auto re-plan + re-auction on blocked-aisle broadcast. Tasks arrive
   continuously (Poisson spawner); headline metric is throughput (tasks/tick).
- **Multi-Agent Path Planning (edge)** — `planner.py`: heapq A*, Manhattan,
  <1ms replan on 28×18 grid, extra-cost penalties for contested cells.
- **Fleet Dashboard** — `dashboard.py` (stdlib Tkinter) or `dashboard_ctk.py`
  (`--ui ctk`, customtkinter): KPI cards, live map, fleet cards with battery
  bars + status pills, task board, visual legend, color-coded event feed,
  live telemetry strip (deliveries + fleet battery). Controls: RUN / STEP /
  RESET / speed / LIGHT-DARK toggle.
- **Charging docks (x2)** — `config.py` docks with per-dock reservations;
  task assignment is pure nearest pickup regardless of charge. Below 25% a
  robot trips to the nearest free dock with its task stashed (carriers keep
  the package — dock arrival never counts as delivery) and resumes at a
  computed charge target. One robot per dock; non-critical robots defer and
  keep working; idle robots keep clear of dock zones. Load-scaled drain
  (0.6 empty / 1.0 loaded); 0% robots die in place and their task re-queues.

## Run
```bash
pip install -r requirements.txt   # sim is stdlib-only; ctk UI needs customtkinter
python benchmark.py               # success-criteria check (smart vs stop-and-wait)
python benchmark.py --seeds 20 --csv results.csv   # batch + 95% CI table
python main.py --policy smart             # live dashboard (Tkinter)
python main.py --policy smart --ui ctk    # modern dashboard (customtkinter)
python main.py --policy smart --headless  # no UI (edge / CI)
```

## Result (overlapping-paths scenario, 3 AMRs, 5 pickup→drop tasks, blocked aisle + mid-run spawns)
- SMART (decentralized): makespan **89**, collisions **0**
- STOPWAIT (traditional): makespan **115**, collisions **0**
- **Reduction 22.6% ≥ 20% — PASS, zero collisions.**

## Deploy on Pi (one process per robot)
Replace `LocalBus` with `UdpPeer(robot_id, port=5005)` — same
`send()/broadcast()/inbox` API. Each Pi runs `main.py` with its `robot_id`;
discovery is pure LAN broadcast, survives Wi-Fi dead zones / cloud outage.

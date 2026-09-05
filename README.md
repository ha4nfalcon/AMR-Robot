# Decentralized AMR Fleet — Edge Coordination & Collision Avoidance

Peer-to-peer multi-robot warehouse simulation (28×18). No cloud, no central server.
Each AMR runs its own A* planner + P2P intent sharing, exactly as it would on
a Raspberry Pi / Jetson Nano. Fixed machinery/pallet obstacles plus spill and
fallen-pallet patches that spawn mid-run keep every run dynamic.

## Mapping to problem statement
1. **Decentralized Communication** — `warehouse_sim/comms.py`: `LocalBus` (sim)
   + `UdpPeer` (real UDP broadcast sockets for Pi/Jetson LAN, JSON <1KB, 5–10Hz).
   Robots share `STATE {pos, intent[3], priority, battery, task}` + `BLOCKED` alerts.
2. **Dynamic Conflict Resolution** — `robot.py` + `simulator.py`: space-time intent
   reservation, priority `(urgency, battery, id)`, yield-or-reroute after 4 waits,
   head-on deadlock vacate (sidestep / revert). Zero collisions verified.
3. **Task Allocation & Re-routing** — `allocator.py`: decentralized single-round
   auction (bid = A* length + battery penalty); auto re-plan + re-auction on
   blocked-aisle broadcast.
- **Multi-Agent Path Planning (edge)** — `planner.py`: heapq A*, Manhattan,
  <1ms replan on 20×14 grid, extra-cost penalties for contested cells.
- **Fleet Dashboard** — `dashboard.py`: dark ops-console UI (stdlib Tkinter).
  KPI cards (mode / steps / tasks / collisions / replans-deadlocks), live map,
  fleet cards with battery bars + status pills, task board, visual legend,
  color-coded event feed. Controls: RUN / STEP / RESET / speed.
- **Charging dock** — `config.py` dock; task assignment is pure nearest
  pickup regardless of charge. Below 25% a robot trips to the dock with its
  task stashed (carriers keep the package — dock arrival never counts as
  delivery) and resumes at a full green 100%. One charges at a time;
  non-critical robots defer and keep working; idle robots keep clear of
  the dock zone while anyone charges; persistent stalls trigger pull-over
  recovery with stall-set tracking.

## Run
```bash
pip install -r requirements.txt   # only for optional plots; sim is stdlib-only
python benchmark.py               # success-criteria check (smart vs stop-and-wait)
python main.py --policy smart             # live dashboard
python main.py --policy smart --headless  # no UI (edge / CI)
python main.py --policy stopwait --headless
```

## Result (overlapping-paths scenario, 3 AMRs, 5 pickup→drop tasks, blocked aisle + mid-run spawns)
- SMART (decentralized): makespan **89**, collisions **0**
- STOPWAIT (traditional): makespan **115**, collisions **0**
- **Reduction 22.6% ≥ 20% — PASS, zero collisions.**

## Deploy on Pi (one process per robot)
Replace `LocalBus` with `UdpPeer(robot_id, port=5005)` — same
`send()/broadcast()/inbox` API. Each Pi runs `main.py` with its `robot_id`;
discovery is pure LAN broadcast, survives Wi-Fi dead zones / cloud outage.

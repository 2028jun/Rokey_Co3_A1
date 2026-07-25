# nav_robot

Isaac Sim 5.1 + ROS 2 Humble **Nav2** stack for restaurant base navigation
(kitchen ↔ tables). See [docs/NAV2_QUICKSTART.md](docs/NAV2_QUICKSTART.md).

- Domain: inherited from the machine-local `ROS_DOMAIN_ID` in `~/.bashrc`
- Mapping (phase 1): Occupancy Map Generator — [docs/OCCUPANCY_MAP.md](docs/OCCUPANCY_MAP.md)
- Controller: Regulated Pure Pursuit
- Planner: SmacPlanner2D (holonomic / mecanum)

# Rokey_Co3_A1 - Serving Robot & Hand Safety Workspace

## Workspace Structure

```text
Rokey_Co3_A1/
├── serving_robot/     # Serving Robot ROS 2, Isaac Sim & HMI Web Dashboard
│   ├── isaacpjt/     # Isaac Sim 5.1 Standalone Python Scripts
│   ├── src/          # ROS 2 Workspace Packages (serving_hmi, etc.)
│   └── run_hmi.sh    # HMI Web Dashboard Runner
├── hand_safety/       # Hand Detection & Safety Intrusion ROS 2 Package
├── .gitignore
└── README.md
```

## Quick Start (HMI Web Dashboard)

```bash
# Run HMI Web Dashboard
./run_hmi.sh
# Open Browser: http://localhost:8000
```

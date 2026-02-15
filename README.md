# ros2kit

Rust utility library for ROS 2 workspace management and ament environment discovery.

## Modules

- **ament** — Discover installed packages, executables, launch files, and interfaces from `AMENT_PREFIX_PATH`
- **workspace** — Scan workspaces for source packages, detect build status, manage overlay paths
- **process** — Launch and manage ROS 2 node/launch file processes
- **launch** — Parse launch file arguments via Python helper
- **log** — Find and tail ROS 2 log files
- **build** — Colcon build integration (requires `colcon` feature)

## Usage

```toml
[dependencies]
ros2kit = { path = "../ros2kit" }

# Enable colcon build support
ros2kit = { path = "../ros2kit", features = ["colcon"] }
```

## Features

| Feature  | Description                          |
|----------|--------------------------------------|
| `colcon` | Enables the `build` module for colcon build integration |

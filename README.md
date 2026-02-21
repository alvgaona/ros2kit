# ros2kit

Rust utility library for ROS 2 workspace management and ament environment discovery.

## Modules

- **ament** — Discover installed packages, executables, launch files, and interfaces from `AMENT_PREFIX_PATH`
- **build** — Colcon build integration (requires `colcon` feature)
- **launch** — Parse launch file arguments via Python helper
- **log** — Find and tail ROS 2 log files
- **process** — Launch and manage ROS 2 node/launch file processes
- **record** — MCAP bag recording (requires `mcap` feature)
- **workspace** — Scan workspaces for source packages, detect build status, manage overlay paths
  (requires `workspace` feature)

## Usage

```toml
[dependencies]
ros2kit = "0.2"

# Enable specific features
ros2kit = { version = "0.2", features = ["colcon", "workspace", "mcap"] }
```

## Features

| Feature     | Description                                                          |
|-------------|----------------------------------------------------------------------|
| `colcon`    | Enables the `build` module for colcon build integration              |
| `mcap`      | Enables the `record` module for MCAP bag recording                   |
| `workspace` | Enables the `workspace` module for package discovery and overlay management |

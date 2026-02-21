# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 workspace integrating Marvelmind Indoor GPS/RTLS with ROS 2 Humble. Contains two packages that communicate with Marvelmind hedgehog beacons/modems over USB serial and publish positioning, IMU, telemetry, and other sensor data as ROS 2 topics.

## Build & Run

```bash
# Build (from workspace root: /home/alb/workspace/indoor_gps/marvelmind/)
source /opt/ros/humble/setup.bash
colcon build

# Launch main positioning node
. install/setup.bash
ros2 launch marvelmind_ros2 marvelmind_ros2.launch.py

# Launch API service node
. install/setup.bash
ros2 launch marvelmind_ros2 marvelmind_api_ros2.launch.py

# Verify data
ros2 topic echo /hedgehog_pos_ang
```

There are no tests configured in this project.

## Architecture

### Packages

- **marvelmind_ros2** (`src/marvelmind_ros2/`) — ament_cmake package (C++14/C99). Contains two nodes and two C libraries.
- **marvelmind_ros2_msgs** (`src/marvelmind_ros2_msgs/`) — ament_cmake message package. Defines 11 message types and 1 service.

### Nodes

**marvelmind_ros2** — Main node. Connects to a hedgehog/modem via serial, polls data buffers at 200 Hz (configurable), and publishes to 11 topics:
- `/hedgehog_pos`, `/hedgehog_pos_addressed`, `/hedgehog_pos_ang` — position data
- `/hedgehog_imu_raw`, `/hedgehog_imu_fusion` — IMU data
- `/beacon_raw_distance`, `/beacons_pos_addressed` — beacon data
- `/hedgehog_telemetry`, `/hedgehog_quality` — device health
- `/marvelmind_waypoint`, `/marvelmind_user_data` — waypoints and user payload

**marvelmind_api_ros2** — API service node. Exposes `/marvelmind_api` service (MarvelmindAPI.srv) for advanced device control commands.

### Key Source Files

- `src/marvelmind_ros2/src/marvelmind_ros2.cpp` — Main node implementation (publishers, timer callback, data polling)
- `src/marvelmind_ros2/src/marvelmind_api_ros2.cpp` — API service node
- `src/marvelmind_ros2/src/marvelmind_hedge.c` — Low-level C library for hedgehog serial protocol
- `src/marvelmind_ros2/src/marvelmind_api.c` — Low-level C library for Marvelmind API
- `src/marvelmind_ros2/include/marvelmind_ros2/` — Headers (hpp for nodes, .h for C libs)

### Configuration

All parameters in `src/marvelmind_ros2/config/marvelmind_ros2_config.yaml`. Key settings:
- `marvelmind_tty_filename` — serial port (default: `/dev/ttyACM0`)
- `marvelmind_tty_baudrate` — baud rate (default: 9600)
- `marvelmind_publish_rate_in_hz` — polling/publish rate (default: 200)
- All topic names are configurable via parameters

### Data Flow

USB serial device → C library (marvelmind_hedge.c) with semaphore-based notification → C++ node polls buffers in timer callback → publishes to ROS 2 topics

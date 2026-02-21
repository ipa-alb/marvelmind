# TODO: RViz Visualization for Marvelmind Indoor GPS

## Goal

Visualize 4 static beacons + 1 mobile hedgehog in RViz2 in real-time, showing:
- Fixed beacon positions as labeled markers
- Moving hedgehog position as a distinct marker
- Hedgehog path trail
- Distance lines between hedgehog and beacons (optional)

## Hardware Setup (before we start)

- 4 stationary beacons mounted at known positions, powered on
- 1 mobile hedgehog, powered on
- 1 modem plugged into PC via USB (`/dev/ttyACM0`)
- All devices paired and tracking in the Marvelmind Dashboard software

## Implementation Plan

### Step 1: Create the visualizer Python package

Create a new lightweight ROS 2 Python package:

```
src/marvelmind_rviz/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/marvelmind_rviz
├── config/
│   └── marvelmind_rviz.rviz       # saved RViz config
├── launch/
│   └── visualize.launch.py        # launches everything together
└── marvelmind_rviz/
    ├── __init__.py
    └── visualizer_node.py          # the visualizer node
```

Using a Python `ament_python` package — no C++ build needed, fast to iterate on.

### Step 2: Write the visualizer node (`visualizer_node.py`)

**Subscriptions** (reads from existing marvelmind_ros2 topics):
| Topic | Message Type | Purpose |
|-------|-------------|---------|
| `/beacons_pos_addressed` | `BeaconPositionAddressed` | Static beacon positions (x, y, z + address) |
| `/hedgehog_pos_ang` | `HedgePositionAngle` | Mobile hedgehog position + heading (x, y, z, angle + address) |
| `/beacon_raw_distance` | `BeaconDistance` | Distances between hedgehog and each beacon |

**Publications** (new topics for RViz):
| Topic | Message Type | Purpose |
|-------|-------------|---------|
| `/visualization/beacons` | `visualization_msgs/MarkerArray` | Cube markers for each beacon, color-coded, with text labels showing address |
| `/visualization/hedgehog` | `visualization_msgs/Marker` | Sphere marker for the mobile hedgehog |
| `/visualization/hedgehog_path` | `nav_msgs/Path` | Trailing path of the hedgehog's movement |
| `/visualization/distances` | `visualization_msgs/MarkerArray` | LINE_LIST markers showing distance to each beacon (optional, togglable) |

**TF broadcasts:**
| Frame | Parent | Type | Source |
|-------|--------|------|--------|
| `map` | — | Fixed frame | Identity (world origin = Marvelmind coordinate origin) |
| `beacon_N` | `map` | Static | From `/beacons_pos_addressed` — published once per beacon when first seen |
| `hedgehog` | `map` | Dynamic | From `/hedgehog_pos_ang` — updated every position message |

**Node parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `beacon_marker_scale` | 0.15 | Size of beacon cubes in meters |
| `hedgehog_marker_scale` | 0.1 | Size of hedgehog sphere in meters |
| `path_max_points` | 500 | Max points in the trail before oldest are dropped |
| `show_distance_lines` | true | Whether to draw distance lines |
| `hedgehog_color` | [0.0, 1.0, 0.0, 1.0] | RGBA for hedgehog (green) |

**Implementation details:**
- Beacon positions: cache by address, publish static TF + Marker on first receipt, update if position changes
- Hedgehog: on each `/hedgehog_pos_ang` callback, broadcast TF `map→hedgehog`, publish Marker, append to Path
- Use `angle` from `HedgePositionAngle` to set hedgehog marker orientation (convert heading to quaternion)
- Beacon colors: auto-assign from a fixed color palette based on address order
- Marker text labels: show `"B<address>"` for beacons, `"H<address>"` for hedgehog
- All markers use `frame_id = "map"`

### Step 3: Create the launch file (`visualize.launch.py`)

Launches three things together:
1. **marvelmind_ros2 node** — the existing positioning node (with its config yaml)
2. **visualizer_node** — the new Python node
3. **rviz2** — with the saved config file

Accept a launch argument for the serial port so it can be overridden:
```
ros2 launch marvelmind_rviz visualize.launch.py
ros2 launch marvelmind_rviz visualize.launch.py tty:=/dev/ttyACM1
```

### Step 4: Create the RViz config file (`marvelmind_rviz.rviz`)

Pre-configured displays:
- **Fixed frame:** `map`
- **Grid:** 1m spacing, 10x10m, subtle gray
- **MarkerArray** display for `/visualization/beacons`
- **Marker** display for `/visualization/hedgehog`
- **Path** display for `/visualization/hedgehog_path`
- **MarkerArray** display for `/visualization/distances` (can be toggled off)
- **TF** display showing all frames (beacon_N, hedgehog)
- Camera: top-down orthographic view

### Step 5: Update `package.xml` dependencies

The new `marvelmind_rviz` package needs:
- `rclpy`
- `marvelmind_ros2_msgs`
- `visualization_msgs`
- `geometry_msgs`
- `nav_msgs`
- `tf2_ros`
- `rviz2` (exec_depend)

No changes needed to the existing `marvelmind_ros2` or `marvelmind_ros2_msgs` packages.

### Step 6: Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# Launch everything at once
ros2 launch marvelmind_rviz visualize.launch.py

# Or launch separately for debugging:
# Terminal 1: positioning node
ros2 launch marvelmind_ros2 marvelmind_ros2.launch.py

# Terminal 2: visualizer only
ros2 run marvelmind_rviz visualizer_node

# Terminal 3: rviz
rviz2 -d src/marvelmind_rviz/config/marvelmind_rviz.rviz
```

**Verification checklist:**
- [ ] 4 beacon cubes appear at their real-world positions with address labels
- [ ] Hedgehog sphere moves in real-time as you carry it around
- [ ] Path trail draws behind the hedgehog
- [ ] TF tree shows `map → beacon_1..4` and `map → hedgehog`
- [ ] Distance lines connect hedgehog to beacons (if enabled)
- [ ] `ros2 topic list` shows all visualization topics publishing

## File Summary

| File | Action | Description |
|------|--------|-------------|
| `src/marvelmind_rviz/package.xml` | **Create** | Package manifest with dependencies |
| `src/marvelmind_rviz/setup.py` | **Create** | Python package setup |
| `src/marvelmind_rviz/setup.cfg` | **Create** | Entry point config |
| `src/marvelmind_rviz/resource/marvelmind_rviz` | **Create** | Empty ament resource marker |
| `src/marvelmind_rviz/marvelmind_rviz/__init__.py` | **Create** | Package init |
| `src/marvelmind_rviz/marvelmind_rviz/visualizer_node.py` | **Create** | Main visualizer node (~150-200 lines) |
| `src/marvelmind_rviz/launch/visualize.launch.py` | **Create** | Combined launch file |
| `src/marvelmind_rviz/config/marvelmind_rviz.rviz` | **Create** | RViz saved config |
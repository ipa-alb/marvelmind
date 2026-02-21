# marvelmind_ros2_msgs

ROS 2 message and service definitions for the [`marvelmind_ros2`](../marvelmind_ros2/) package.

## Messages

| Message | Fields | Description |
| --- | --- | --- |
| `HedgePosition` | `timestamp_ms`, `x_m`, `y_m`, `z_m`, `flags` | Hedgehog position |
| `HedgePositionAddressed` | `address`, `timestamp_ms`, `x_m`, `y_m`, `z_m`, `flags` | Hedgehog position with beacon address |
| `HedgePositionAngle` | `address`, `timestamp_ms`, `x_m`, `y_m`, `z_m`, `flags`, `angle` | Hedgehog position with orientation angle |
| `HedgeImuRaw` | `timestamp_ms`, `acc_{x,y,z}`, `gyro_{x,y,z}`, `compass_{x,y,z}` | Raw IMU data (accelerometer, gyroscope, compass) |
| `HedgeImuFusion` | `timestamp_ms`, `x_m`, `y_m`, `z_m`, `q{w,x,y,z}`, `v{x,y,z}`, `a{x,y,z}` | Fused IMU (position, quaternion, velocity, acceleration) |
| `HedgeTelemetry` | `battery_voltage`, `rssi_dbm` | Battery voltage and signal strength |
| `HedgeQuality` | `address`, `quality_percents` | Positioning quality percentage |
| `BeaconDistance` | `address_hedge`, `address_beacon`, `distance_m` | Raw distance between hedge and beacon |
| `BeaconPositionAddressed` | `address`, `x_m`, `y_m`, `z_m` | Stationary beacon position |
| `MarvelmindWaypoint` | `total_items`, `item_index`, `movement_type`, `param1`, `param2`, `param3` | Waypoint definition |
| `MarvelmindUserData` | `timestamp_ms`, `data` | User-defined payload |

## Services

| Service | Request | Response | Description |
| --- | --- | --- | --- |
| `MarvelmindAPI` | `command_id` (int64), `request` (uint8[]) | `success` (bool), `error_code` (int32), `response` (uint8[]) | Send commands to the Marvelmind device |

## Build

This package is built automatically alongside `marvelmind_ros2`:

```bash
source /opt/ros/humble/setup.bash
colcon build
```

## License

BSD-2-Clause.

# marvelmind_ros2

ROS 2 package for [Marvelmind Indoor GPS/RTLS](https://marvelmind.com/). Connects to a Marvelmind hedgehog beacon or modem over USB serial and publishes positioning, IMU, telemetry, and other sensor data as ROS 2 topics.

Tested with **ROS 2 Humble** on Ubuntu 22.04.

## Prerequisites

- ROS 2 Humble ([installation guide](https://docs.ros.org/en/humble/Installation.html))
- A configured Marvelmind system with tracking active (see [operating manual](http://marvelmind.com/pics/marvelmind_navigation_system_manual.pdf))
- The companion message package [`marvelmind_ros2_msgs`](../marvelmind_ros2_msgs/)

## Setup

### 1. Connect the device

Plug a mobile beacon or modem into the computer via USB and find the serial port:

```bash
ls /dev/ttyACM*    # most common
ls /dev/ttyUSB*    # fallback
```

### 2. Set serial port permissions

```bash
sudo chmod 0777 /dev/ttyACM0
```

For permanent access, add your user to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
# log out and back in for the change to take effect
```

### 3. Configure

Edit [`config/marvelmind_ros2_config.yaml`](config/marvelmind_ros2_config.yaml) to set the serial port and other parameters:

| Parameter | Default | Description |
| --- | --- | --- |
| `marvelmind_tty_filename` | `/dev/ttyACM0` | Serial port path |
| `marvelmind_tty_baudrate` | `9600` | Baud rate (rarely needs changing) |
| `marvelmind_publish_rate_in_hz` | `200` | Polling/publish rate in Hz |

Topic names are also configurable in the same file.

### 4. Build

```bash
cd ~/your_workspace
source /opt/ros/humble/setup.bash
colcon build
```

### 5. Launch

**Main positioning node:**

```bash
. install/setup.bash
ros2 launch marvelmind_ros2 marvelmind_ros2.launch.py
```

**API service node** (for advanced device control):

```bash
. install/setup.bash
ros2 launch marvelmind_ros2 marvelmind_api_ros2.launch.py
```

### 6. Verify

In a separate terminal:

```bash
. install/setup.bash
ros2 topic echo /hedgehog_pos_ang
```

## Nodes

### marvelmind_ros2 node

Main node. Polls data from the hedgehog/modem and publishes to the following topics:

| Topic | Message Type | Description |
| --- | --- | --- |
| `/hedgehog_pos` | `HedgePosition` | Position (x, y, z) with flags |
| `/hedgehog_pos_addressed` | `HedgePositionAddressed` | Position with beacon address |
| `/hedgehog_pos_ang` | `HedgePositionAngle` | Position with orientation angle |
| `/hedgehog_imu_raw` | `HedgeImuRaw` | Raw accelerometer, gyroscope, compass |
| `/hedgehog_imu_fusion` | `HedgeImuFusion` | Fused IMU (position, quaternion, velocity, acceleration) |
| `/beacon_raw_distance` | `BeaconDistance` | Raw distance between hedge and beacon |
| `/beacons_pos_addressed` | `BeaconPositionAddressed` | Stationary beacon positions |
| `/hedgehog_telemetry` | `HedgeTelemetry` | Battery voltage and RSSI |
| `/hedgehog_quality` | `HedgeQuality` | Positioning quality percentage |
| `/marvelmind_waypoint` | `MarvelmindWaypoint` | Waypoint data |
| `/marvelmind_user_data` | `MarvelmindUserData` | User-defined payload data |

### marvelmind_api_ros2

API service node. Exposes the `/marvelmind_api` service (`MarvelmindAPI.srv`) for sending commands directly to the Marvelmind device.

## Credits

Original ROS 2 package implementation by [Carson Loyal](mailto:ctl0021@auburn.edu).
Based on upstream code from [Marvelmind Robotics](https://github.com/MarvelmindRobotics).

## License

BSD-2-Clause. See source file headers for the full license text.

# Marvelmind ROS 2 Installation Guide

Marvelmind supplies the ROS 2 package `marvelmind_ros2` (with a separate messages package `marvelmind_ros2_msgs`), which communicates with a mobile beacon or modem and provides location and other data. The latest version has been tested with **ROS 2 Humble** on Ubuntu 22.04 and Windows 10.

The package source is available in the following repositories:

- [marvelmind_ros2_upstream](https://github.com/MarvelmindRobotics/marvelmind_ros2_upstream)
- [marvelmind_ros2_msgs_upstream](https://github.com/MarvelmindRobotics/marvelmind_ros2_msgs_upstream)

## Before You Begin

1. **Install ROS 2** (if not already installed): [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)
2. **Prepare the Marvelmind system** — use another PC with the dashboard software to build a map as described in the [Marvelmind Operating Manual](http://marvelmind.com/pics/marvelmind_navigation_system_manual.pdf). You should see tracking in the dashboard. If tracking is working, the location data will be available through the ROS 2 package.
3. **Create a workspace folder** (e.g., `ros2_ws`) and place the two package source folders inside it: `marvelmind_ros2` and `marvelmind_ros2_msgs`.

## 1.1 Installation and Running under Linux

### Prerequisites

1. Open the terminal.
2. Navigate to your ROS 2 workspace and source the environment:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
```

### Connect the Device

1. Connect a mobile beacon or modem to the computer via USB.
2. Find the virtual serial port used by the modem or mobile beacon:

```bash
ls /dev/ttyACM*
```

In most cases, the hedgehog connects to `/dev/ttyACM0`, which is the default port used by the Marvelmind ROS software. If no ports are found, try:

```bash
ls /dev/ttyUSB*
```

### Set Serial Port Permissions

Make sure you have permissions to access the port. You can grant full permissions with:

```bash
sudo chmod 0777 /dev/ttyACM0
```

> **Note:** These permissions will be lost after a reboot. For permanent permissions, add your user to the `dialout` group as described in
> [Changing permissions on serial port](https://askubuntu.com/questions/58119/changing-permissions-on-serial-port).

### Configure the Serial Port

Set the serial port setting `marvelmind_tty_filename` in the configuration file:

```yaml
/marvelmind_ros2/config/marvelmind_ros2_config.yaml
```

After installation, you can also modify this setting via ROS 2 parameter commands.

### Build and Launch

1. Install dependencies and build the workspace:

```bash
sudo apt install python3-colcon-common-extensions
colcon build
```

`colcon build` should report a successful build of the packages.

1. Source the workspace and launch the node:

```bash
. install/setup.bash
ros2 launch marvelmind_ros2 marvelmind_ros2.launch.py
```

> **Note:** Don't forget the space between the dot `.` and `install/setup.bash`.

A `Hedgehog is running` message should appear, meaning the ROS 2 package is connected to the hedgehog (or modem).

### Verify Position Data

Open **another terminal** in the workspace directory and run:

```bash
. install/setup.bash
source /opt/ros/humble/setup.bash
ros2 topic echo /hedgehog_pos_ang
```

You should see location data from the active mobile beacon on the `/hedgehog_pos_ang` topic.

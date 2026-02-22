# Marvelmind Physical Setup Guide

Step-by-step guide for setting up a Marvelmind Indoor GPS system with 4 stationary beacons, 1 mobile hedgehog, and 1 modem connected to a robot running ROS 2.

---

## What You Need

| Item | Quantity | Purpose |
| --- | --- | --- |
| Stationary beacon | 4 | Fixed reference points — they triangulate the hedgehog's position |
| Mobile hedgehog | 1 | Mounted on your robot — this is what gets tracked |
| Modem | 1 | Plugged into your robot's PC via USB — bridges the Marvelmind network to ROS 2 |
| USB cable | 1 | Connects the modem to your robot's computer |
| Separate PC/laptop | 1 | For running the Marvelmind Dashboard software (initial setup only) |
| Power for beacons | 4 | USB chargers or battery — beacons need power |
| Mounting hardware | — | Screws, double-sided tape, brackets, etc. |

---

## Step 1: Install the Marvelmind Dashboard

On a **separate PC** (not the robot), download and install the Marvelmind Dashboard:

- Download from: https://marvelmind.com/download/
- Available for Windows (recommended for initial setup) and Linux
- This is where you configure beacons, build the map, and verify tracking

---

## Step 2: Mount the Stationary Beacons

### Placement rules

- Mount **4 beacons** around the perimeter of your operating area
- Place them **high up** (2-3 meters) — ceiling or high on walls, angled downward
- They need **line of sight** to the area where the hedgehog will move
- Avoid placing all 4 in a straight line — spread them out in a **rectangular or trapezoidal** layout for best accuracy
- Keep beacons **at least 1 meter apart** from each other
- Avoid placing beacons directly next to large metal surfaces or reflective objects

### Example layout (top-down view)

```
    B1 ──────────────────── B2
    |                        |
    |                        |
    |     operating area     |
    |                        |
    |                        |
    B3 ──────────────────── B4
```

### Mounting tips

- Use the mounting brackets that come with the beacons, or double-sided tape for quick testing
- The ultrasonic transducer (the round part) should face **downward** into the operating area
- Make sure they are stable — vibration will degrade accuracy
- Power each beacon via USB (wall charger, power bank, or long USB cable)

---

## Step 3: Mount the Mobile Hedgehog on Your Robot

- Attach the hedgehog to the **top** of your robot with a clear view upward to the stationary beacons
- Do not bury it under a shell or cover — it needs line of sight to the ceiling beacons
- Keep it away from motors, metal chassis parts, and other sources of electromagnetic noise if possible
- The hedgehog runs on its internal battery — charge it via USB before use
- Make note of its **address** (printed on the device or shown in the dashboard) — you will need it later

---

## Step 4: Connect the Modem to the Dashboard PC

1. Plug the modem into the **dashboard PC** via USB
2. Open the Marvelmind Dashboard software
3. The dashboard should detect the modem and show connected devices
4. If it does not detect anything:
   - Check that the modem LED is blinking
   - Try a different USB port or cable
   - On Linux, check `ls /dev/ttyACM*` and set permissions: `sudo chmod 0777 /dev/ttyACM0`

---

## Step 5: Configure Beacons in the Dashboard

### Power on all beacons

Turn on all 4 stationary beacons and the mobile hedgehog. Wait about 30 seconds for them to join the network.

### Verify devices appear

In the dashboard, you should see all 5 devices listed (4 stationary + 1 mobile). Each has a unique address number.

### Set beacon roles

- Mark the 4 beacons as **stationary**
- Mark the hedgehog as **mobile** (it may already be set this way by default)

### Enter beacon coordinates (manual submap)

For each stationary beacon, enter its **real-world coordinates** (x, y, z in meters). Measure these from a common origin point in your space.

Example:

| Beacon | X (m) | Y (m) | Z (m) |
| --- | --- | --- | --- |
| B1 (addr 1) | 0.0 | 0.0 | 2.5 |
| B2 (addr 2) | 5.0 | 0.0 | 2.5 |
| B3 (addr 3) | 0.0 | 4.0 | 2.5 |
| B4 (addr 4) | 5.0 | 4.0 | 2.5 |

Alternatively, use the dashboard's **automatic submap building** — it will figure out relative positions from ultrasonic measurements. Manual entry is more reliable if you can measure accurately.

### Freeze the submap

Once the beacon coordinates are set and tracking looks stable, **freeze the submap** in the dashboard so it does not drift.

---

## Step 6: Verify Tracking in the Dashboard

- Pick up the hedgehog and walk it around your operating area
- You should see the hedgehog's position update in real time on the dashboard map
- Check that the coordinates make sense relative to your beacon positions
- If tracking is jumpy or not working:
  - Check line of sight between hedgehog and beacons
  - Make sure beacons are not too close together or in a line
  - Re-freeze the submap after adjustments

**Do not proceed to the ROS 2 setup until tracking works reliably in the dashboard.**

---

## Step 7: Move the Modem to the Robot

Once the map is built and tracking is verified:

1. **Disconnect** the modem from the dashboard PC
2. **Connect** the modem to your robot's computer via USB
3. The beacons and hedgehog continue operating on their own — the modem just receives data

> The modem does not need the dashboard PC to work. The dashboard was only needed for initial map configuration. The modem will continue receiving position data independently.

---

## Step 8: Set Up the Robot's Computer

### Find the serial port

```bash
ls /dev/ttyACM*
```

You should see `/dev/ttyACM0` (or similar).

### Set permissions

```bash
# Temporary (resets on reboot)
sudo chmod 0777 /dev/ttyACM0

# Permanent (recommended)
sudo usermod -aG dialout $USER
# Log out and back in for this to take effect
```

### Update the config if needed

If your modem is not on `/dev/ttyACM0`, edit the config file:

```bash
# Edit this file:
# src/marvelmind_ros2/config/marvelmind_ros2_config.yaml
#
# Change this line:
# marvelmind_tty_filename: "/dev/ttyACM0"
# to your actual port, e.g.:
# marvelmind_tty_filename: "/dev/ttyUSB0"
```

---

## Step 9: Build and Launch ROS 2

```bash
cd /home/alb/workspace/indoor_gps/marvelmind
source /opt/ros/humble/setup.bash
colcon build
```

Launch the positioning node:

```bash
. install/setup.bash
ros2 launch marvelmind_ros2 marvelmind_ros2.launch.py
```

You should see: **`Hedgehog is running`**

---

## Step 10: Verify Data in ROS 2

Open a **second terminal**:

```bash
cd /home/alb/workspace/indoor_gps/marvelmind
. install/setup.bash
ros2 topic echo /hedgehog_pos_ang
```

You should see position data streaming. Move the hedgehog around and confirm the coordinates change.

### Other useful topics to check

```bash
# List all active topics
ros2 topic list

# Check beacon positions
ros2 topic echo /beacons_pos_addressed

# Check raw distances
ros2 topic echo /beacon_raw_distance

# Check IMU data (if your hedgehog has IMU)
ros2 topic echo /hedgehog_imu_fusion

# Check battery and signal
ros2 topic echo /hedgehog_telemetry
```

---

## Troubleshooting

| Problem | Fix |
| --- | --- |
| `ls /dev/ttyACM*` shows nothing | Try a different USB port/cable. Check `dmesg \| tail` for errors. Try `ls /dev/ttyUSB*`. |
| Node starts but no data | Is tracking working in the dashboard? The modem only relays data — if beacons aren't tracking, there's nothing to relay. |
| Permission denied on serial port | Run `sudo chmod 0777 /dev/ttyACM0` or add yourself to the `dialout` group. |
| Position data is jumpy | Check line of sight. Move beacons further apart. Re-freeze the submap in the dashboard. |
| `colcon build` fails | Make sure you ran `source /opt/ros/humble/setup.bash` first. Check that both `marvelmind_ros2` and `marvelmind_ros2_msgs` packages are in `src/`. |
| Hedgehog not detected by dashboard | Charge the hedgehog. Make sure it's powered on (LED blinking). Try bringing it closer to the modem. |
| Multiple USB devices — wrong port | Unplug everything except the modem, run `ls /dev/ttyACM*`, note the port, then update the config. |

---

## Quick Reference: System Architecture

```
                    ┌──────────────┐
                    │  Stationary  │  x4, mounted high
                    │   Beacons    │  on walls/ceiling
                    └──────┬───────┘
                           │ ultrasonic + radio
                           ▼
                    ┌──────────────┐
                    │    Mobile    │  mounted on robot
                    │  Hedgehog    │  gets tracked
                    └──────┬───────┘
                           │ radio
                           ▼
                    ┌──────────────┐
                    │    Modem     │  plugged into robot PC
                    │   (USB)      │  via USB serial
                    └──────┬───────┘
                           │ /dev/ttyACM0
                           ▼
                    ┌──────────────┐
                    │  ROS 2 Node  │  marvelmind_ros2
                    │              │  reads serial data
                    └──────┬───────┘
                           │ ROS 2 topics
                           ▼
                    ┌──────────────┐
                    │  Your Robot  │  navigation, SLAM,
                    │   Software   │  visualization, etc.
                    └──────────────┘
```

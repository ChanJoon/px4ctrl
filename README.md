# PX4 Controller

Original repo:
1. https://github.com/aphasiayc/px4ctrl
2. https://github.com/ZJU-FAST-Lab/Fast-Drone-250


---
*Original README.md*

In Chinese, Read [CHINESE.md](./CHINESE.md)

- [Flight Controller Setup](#flight-controller-setup)
  * [1. Briefly Read the Official Documentation](#1-briefly-read-the-official-documentation)
  * [2. Firmware Flashing and Basic Setup](#2-firmware-flashing-and-basic-setup)
  * [3. Parameter Tuning](#3-parameter-tuning)
- [MAVROS Installation and Configuration](#mavros-installation-and-configuration)
  * [1. Wiring](#1-wiring)
  * [2. Installation](#2-installation)
  * [3. Configuration](#3-configuration)
- [PX4 Controller Quick Start Guide](#px4-controller-quick-start-guide)
  * [Controller Overview](#controller-overview)
  * [Coordinate System Definition](#coordinate-system-definition)
  * [RC Channel Mapping (in QGroundControl)](#rc-channel-mapping--in-qgroundcontrol-)
  * [Correspondence in px4ctrl](#correspondence-in-px4ctrl)
  * [Controller Topics & Services](#controller-topics---services)
  * [Basic Usage (Low Precision Control)](#basic-usage--low-precision-control-)
    + [Quick Steps:](#quick-steps-)
  * [Auto Takeoff & Landing](#auto-takeoff---landing)
- [PX4 Controller Quick Start Guide](#px4-controller-quick-start-guide-1)
  * [Controller Overview](#controller-overview-1)
  * [Coordinate System Definition](#coordinate-system-definition-1)
  * [RC Channel Mapping (in QGroundControl)](#rc-channel-mapping--in-qgroundcontrol--1)
  * [Correspondence in px4ctrl](#correspondence-in-px4ctrl-1)
  * [Controller Topics & Services](#controller-topics---services-1)
  * [Basic Usage (Low Precision Control)](#basic-usage--low-precision-control--1)
    + [Quick Steps:](#quick-steps--1)
  * [Auto Takeoff & Landing](#auto-takeoff---landing-1)
  * [Cascaded PID Feedback Control](#cascaded-pid-feedback-control)
  * [Simple Thrust Model](#simple-thrust-model)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>


## Flight Controller Setup

### 1. Briefly Read the Official Documentation

PX4 flight controllers are **not** plug-and-play and require considerable initial setup. It's recommended to spend at least half a day skimming through the PX4 official documentation from the "Introduction" to the "Flight Log Analysis" sections at [docs.px4.io](https://docs.px4.io).
It is highly recommended to purchase a cheap flight controller simulator (e.g., Phoenix simulator for \~10 RMB on Taobao) to practice flying on a PC before testing on real hardware.

### 2. Firmware Flashing and Basic Setup

1. Use QGroundControl (QGC) to flash the `.px4` firmware provided in the codebase. This firmware modifies the sending rate of several MAVLink messages. If flashing fails, contact the maintainer—the firmware versioning can be quite complicated.
2. Select the correct airframe and reboot.

   * For 250mm: Generic 250 Racer
   * For 330mm: DJI F330
   * For 450mm: Generic Quadcopter or DJI F450
   * Choose the closest match for other sizes.
3. Calibrate the sensors and radio. Assign channel 5 for flight mode switching, and channel 7 for Emergency Stop (recommended).
4. Change the following parameters:

   ```
   CBRK_IO_SAFETY = 22027  
   CBRK_USB_CHK = 197848  
   MAV_1_CONFIG = TELEM 2  
   ```
5. If using DShot ESCs (**recommended**), update parameters:

   ```
   SYS_USE_IO = 0  
   DSHOT_CONFIG = DShot**  // Replace ** with the highest supported by your ESC
   ```

   To test and reverse motor direction without soldering, use the MAVLink Console in QGC:

   ```
   dshot reverse -m 1  
   dshot save -m 1  
   ```

   Replace "1" with the appropriate motor ID. Refer to [DShot setup guide](https://docs.px4.io/master/en/peripherals/dshot.html) for details.
6. If using PWM ESCs, remember to perform throttle calibration. You can find many guides by searching online.
7. PX4’s built-in `ekf2` (the **preferred** estimator) requires a magnetometer and barometer for state estimation. These sensors can be unreliable on small drones due to noise, often leading to poor estimation, unstable control, and frequent reboots.
   If you need better control precision and don’t want to reboot often, consider replacing `ekf2` with the built-in complementary filter by setting:

   ```
   SYS_MC_EST_GROUP = Q attitude estimator(no position)  
   SYS_HAS_BARO = 0  
   SYS_HAS_MAG = 0  
   ```

   Then reboot.
   **Note:** This disables altitude/position hold, leaving only Stabilized (attitude control) and Acro (rate control) modes, both of which require manual throttle control and more flight practice.

### 3. Parameter Tuning

1. (**Required**) On the "Tuning" page in QGC, adjust the "Hover Throttle" such that in Stabilize mode, with the throttle stick centered, the drone hovers steadily without significant vertical drift. Incorrect values here can cause the drone to shoot upward dangerously in altitude hold or position hold modes.
   For racing drones, this value is typically around 20%; for larger, heavier drones, it's closer to the default 50%.
2. (**Optional but recommended**) Click “Advanced” on the Tuning page to access the PID tuning interface. Adjust the PID parameters as needed. Refer to:

   * [https://docs.px4.io/master/en/config\_mc/pid\_tuning\_guide\_multicopter.html](https://docs.px4.io/master/en/config_mc/pid_tuning_guide_multicopter.html)
   * [https://docs.px4.io/master/en/config\_mc/racer\_setup.html](https://docs.px4.io/master/en/config_mc/racer_setup.html)
3. A good controller configuration should feel responsive and intuitive in Angle (Stabilized) mode. With the drone fully assembled and properly balanced, it should remain stable even under aggressive stick movements.
   **Note:** PID tuning requires manual flying skills since you must test both Acro (rate) and Stabilized (angle) modes. If high-precision control is needed, PID tuning is essential.
   **Incorrect tuning may cause the drone to flip during rapid maneuvers.**
   The `THR_MDL_FAC` parameter mentioned in PX4's docs is optional, as it will be auto-identified later during thrust modeling.


## MAVROS Installation and Configuration

### 1. Wiring

Connect your computer’s serial port to the TELEM2 port of the flight controller.
If your device supports hardware serial flow control (TX, RX, RTS, CTS), it is recommended to enable flow control to avoid blocking and data loss.

### 2. Installation

MAVROS is a ROS package for communication between PX4 and ROS. It is recommended to install it via `apt` (may require internet access):

```bash
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

Refer to the installation guide:
[https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)

### 3. Configuration

```bash
roscd mavros
cd launch
```

Navigate to the `launch` directory under `/opt/ros/.../share/mavros/`. Use `sudo` to modify the following two files:

* `px4_pluginlists.yaml`
* `px4.launch`

You can refer to the example versions provided here:
[https://github.com/ZJU-FAST-Lab/PX4-controller/tree/master/mavros\_launch\_files](https://github.com/ZJU-FAST-Lab/PX4-controller/tree/master/mavros_launch_files)

* `px4_pluginlists.yaml` disables unnecessary plugins to reduce message traffic.
* `px4.launch` configures MAVROS to communicate with PX4 and QGC:

  * Set `fcu_url` to the correct serial device name of your system and baud rate to `921600`.
  * Set `gcs_url` to `"udp-b://@"` to enable broadcast mode, allowing any computer on the same LAN to connect to PX4 via QGroundControl.

After successfully connecting MAVROS to the flight controller, you should see the firmware version printed in the terminal. You can verify the IMU data stream with:

```bash
rostopic hz /mavros/imu/data
```

This should return a frequency of 200Hz, confirming that the connection is working properly.

If using hardware flow control, set:

```bash
fcu_url = serial-hwfc:///path/to/serial/device[:baudrate]
```

For detailed usage, refer to:

* [http://wiki.ros.org/mavros](http://wiki.ros.org/mavros)
* [https://github.com/mavlink/mavros/blob/master/mavros/README.md](https://github.com/mavlink/mavros/blob/master/mavros/README.md)

## PX4 Controller Quick Start Guide

The PX4 Controller node is named `px4ctrl`. It uses Odometry and onboard IMU as feedback, and receives desired pose and orientation commands to perform high-precision control of the drone.

### Controller Overview

The `px4ctrl` controller shares a similar code structure with the previously used `n3ctrl`, but the internal state machine and control algorithms are completely rewritten to enhance stability and precision.

The controller supports three modes:

* **Manual Mode**: Displays "MANUAL\_CTRL(L1)" in green. The controller is inactive and the drone is fully controlled via RC.
* **Hover Mode**: Displays "AUTO\_HOVER(L2)" in green. The controller takes over in "Offboard" mode and performs hover control based on odometry feedback. RC inputs result in gentle motion, similar to DJI Mavic hover mode. No user commands are accepted in this mode.
* **Command Mode**: Displays "CMD\_CTRL(L3)" in green. The controller accepts external commands only after switching to Hover Mode and flipping channel 6, ensuring safety checks are passed.

### Coordinate System Definition

The odometry pose uses the convention: X-forward, Y-left, Z-up. The drone’s nose aligns with the positive X-axis, and thrust acts in the positive Z-axis. Make sure coordinate alignment is exact.

Unlike the [ROS nav\_msgs/Odometry](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html), which uses body-frame velocity, most open-source VIO outputs world-frame velocity. The controller expects world-frame by default. If your system outputs body-frame velocity, set `#define VEL_IN_BODY 0` to `1` in `input.cpp`.

### RC Channel Mapping (in QGroundControl)

* Channel 5: Flight mode switch
* Channel 6: Toggle command mode (assign a switch but leave it unconfigured)
* Channel 7: Emergency Stop
* Channel 8: Reboot controller (assign a momentary switch)

### Correspondence in px4ctrl

* Ch. 5: Switch between px4ctrl (Offboard) and normal PX4 modes
* Ch. 6: Allow external control commands
* Ch. 7: Emergency Stop
* Ch. 8: Soft reboot (commonly used with EKF2 estimation)

### Controller Topics & Services

Check `px4ctrl_node.cpp` for all topics and services. Modify `run_ctrl.launch` to reflect your topic names. Use `rqt_graph` to verify connections to:

* `/mavros/state`
* `/mavros/imu/data`
* `/mavros/rc/in`
* `/mavros/setpoint_raw/attitude`
* Your odometry topic

Ensure `/mavros/imu/data` and your odometry publish at over 200 Hz. `/mavros/rc/in` should be around 10 Hz.

### Basic Usage (Low Precision Control)

The controller runs in a self-adaptive low-precision mode by default. The default parameters generally work well.

#### Quick Steps:

1. Launch MAVROS and your odometry node:

   ```
   roslaunch mavros px4.launch
   roslaunch <your_odometry_node>
   ```
2. Launch px4ctrl:

   ```
   roslaunch px4ctrl run_ctrl.launch
   ```
3. With the drone disarmed, toggle channel 5. If you see `[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)`, hover mode is active. Toggle channel 6. If you see `[px4ctrl] TRIGGER sent, allow user command.`, command mode is now permitted.
4. Arm the drone and take off manually to a suitable height. Toggle channel 5 to enter hover mode, then toggle channel 6 to enter command mode.
5. Once in command mode, the controller publishes a `/traj_start_trigger` topic with the current pose. This can be used as a trigger for the planner.
6. **Troubleshooting**:

   * Errors from PX4 may appear only in QGC. Watch for firmware messages.
   * If MAVROS has serial errors, check port permissions: `sudo chmod 777 /dev/<port>`.
   * Check cable connections and `MAV_1_CONFIG` param.
   * Ensure `geographiclib` is installed.
   * With EKF2, unlock issues after landing are common. Toggle channel 8 to reboot.
   * Drone shoots up/down in hover: throttle stick not centered. Manually center or set `max_manual_vel = 0` to disable RC override.
   * Hover oscillation: likely due to poor vibration damping. Good IMUs have ±0.3 m/s² noise vs. ±5\~10 m/s² on poor ones. Record an IMU bag and analyze. Prefer hardware damping. The parameter `rho2` is locked for safety. If absolutely needed, you can:

     1. Increase `rho2` (e.g., to 0.9995), but this reduces responsiveness.
     2. Modify `estimateThrustModel()` to use velocity instead of acceleration.
     3. (Not recommended) Comment out thrust estimation entirely.

### Auto Takeoff & Landing

Configure the `auto_takeoff_land` parameters and publish to the `/px4ctrl/takeoff_land` topic.

* Use `quadrotor_msgs::TakeoffLand::TAKEOFF` to take off.
* Use `quadrotor_msgs::TakeoffLand::LAND` to land.

Strict safety checks are built in. Always check terminal errors before operation. Landing includes a 10s lock and detection sequence—be patient.

You may regain manual control anytime by toggling channel 5. During descent, toggling channel 6 will switch back to hover mode.

**Key Parameters:**

* `auto_takeoff_land`:
  * `enable`: Enable auto takeoff/landing
  * `enable_auto_arm`: Enable automatic arming. If false, arm manually before sending takeoff
  * `takeoff_height`: Desired takeoff altitude
  * `takeoff_land_speed`: Takeoff/landing vertical speed

**Troubleshooting:**

* Drone doesn't take off: `hover_percentage` too low (thrust model not active)
* Overshoot in height: controller gain (kp/kv) too low
* Crashes at takeoff: check odometry
* Motors keep spinning after landing: `MPC_MANTHR_MIN` is 0. Set it to 0.08

## PX4 Controller Quick Start Guide

The PX4 Controller node is named `px4ctrl`. It uses Odometry and onboard IMU as feedback, and receives desired pose and orientation commands to perform high-precision control of the drone.

### Controller Overview

The `px4ctrl` controller shares a similar code structure with the previously used `n3ctrl`, but the internal state machine and control algorithms are completely rewritten to enhance stability and precision.

The controller supports three modes:

* **Manual Mode**: Displays "MANUAL\_CTRL(L1)" in green. The controller is inactive and the drone is fully controlled via RC.
* **Hover Mode**: Displays "AUTO\_HOVER(L2)" in green. The controller takes over in "Offboard" mode and performs hover control based on odometry feedback. RC inputs result in gentle motion, similar to DJI Mavic hover mode. No user commands are accepted in this mode.
* **Command Mode**: Displays "CMD\_CTRL(L3)" in green. The controller accepts external commands only after switching to Hover Mode and flipping channel 6, ensuring safety checks are passed.

**Mode Priority and Transitions:**

* Priority order is: Manual > Hover > Command.
* Switching channel 5 out of Hover mode will always fall back to Manual.
* Switching channel 6 out of Command mode returns to Hover mode.
* Timeout of odometry, IMU, or RC input will cause fallback to Manual.
* Command timeout will fall back to Hover mode.

**Related Parameters:**

* `ctrl_freq_max`: Control loop frequency (usually left unchanged)
* `max_manual_vel`: Max velocity for manual position tuning in Hover mode, also limits yaw rate
* `rc_reverse`: Reverse RC channel directions if needed
* `msg_timeout`: Timeout durations for each message type

### Coordinate System Definition

The odometry pose uses the convention: X-forward, Y-left, Z-up. The drone’s nose aligns with the positive X-axis, and thrust acts in the positive Z-axis. Make sure coordinate alignment is exact.

Unlike the [ROS nav\_msgs/Odometry](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html), which uses body-frame velocity, most open-source VIO outputs world-frame velocity. The controller expects world-frame by default. If your system outputs body-frame velocity, set `#define VEL_IN_BODY 0` to `1` in `input.cpp`.

### RC Channel Mapping (in QGroundControl)

* Channel 5: Flight mode switch
* Channel 6: Toggle command mode (assign a switch but leave it unconfigured)
* Channel 7: Emergency Stop
* Channel 8: Reboot controller (assign a momentary switch)

### Correspondence in px4ctrl

* Ch. 5: Switch between px4ctrl (Offboard) and normal PX4 modes
* Ch. 6: Allow external control commands
* Ch. 7: Emergency Stop
* Ch. 8: Soft reboot (commonly used with EKF2 estimation)

### Controller Topics & Services

Check `px4ctrl_node.cpp` for all topics and services. Modify `run_ctrl.launch` to reflect your topic names. Use `rqt_graph` to verify connections to:

* `/mavros/state`
* `/mavros/imu/data`
* `/mavros/rc/in`
* `/mavros/setpoint_raw/attitude`
* Your odometry topic

Ensure `/mavros/imu/data` and your odometry publish at over 200 Hz. `/mavros/rc/in` should be around 10 Hz.

### Basic Usage (Low Precision Control)

The controller runs in a self-adaptive low-precision mode by default. The default parameters generally work well.

#### Quick Steps:

1. Launch MAVROS and your odometry node:

   ```
   roslaunch mavros px4.launch
   roslaunch <your_odometry_node>
   ```
2. Launch px4ctrl:

   ```
   roslaunch px4ctrl run_ctrl.launch
   ```
3. With the drone disarmed, toggle channel 5. If you see `[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)`, hover mode is active. Toggle channel 6. If you see `[px4ctrl] TRIGGER sent, allow user command.`, command mode is now permitted.
4. Arm the drone and take off manually to a suitable height. Toggle channel 5 to enter hover mode, then toggle channel 6 to enter command mode.
5. Once in command mode, the controller publishes a `/traj_start_trigger` topic with the current pose. This can be used as a trigger for the planner.
6. **Troubleshooting**:

   * Errors from PX4 may appear only in QGC. Watch for firmware messages.
   * If MAVROS has serial errors, check port permissions: `sudo chmod 777 /dev/<port>`.
   * Check cable connections and `MAV_1_CONFIG` param.
   * Ensure `geographiclib` is installed.
   * With EKF2, unlock issues after landing are common. Toggle channel 8 to reboot.
   * Drone shoots up/down in hover: throttle stick not centered. Manually center or set `max_manual_vel = 0` to disable RC override.
   * Hover oscillation: likely due to poor vibration damping. Good IMUs have ±0.3 m/s² noise vs. ±5\~10 m/s² on poor ones. Record an IMU bag and analyze. Prefer hardware damping. The parameter `rho2` is locked for safety. If absolutely needed, you can:

     1. Increase `rho2` (e.g., to 0.9995), but this reduces responsiveness.
     2. Modify `estimateThrustModel()` to use velocity instead of acceleration.
     3. (Not recommended) Comment out thrust estimation entirely.

### Auto Takeoff & Landing

Configure the `auto_takeoff_land` parameters and publish to the `/px4ctrl/takeoff_land` topic.

* Use `quadrotor_msgs::TakeoffLand::TAKEOFF` to take off.
* Use `quadrotor_msgs::TakeoffLand::LAND` to land.

Strict safety checks are built in. Always check terminal errors before operation. Landing includes a 10s lock and detection sequence—be patient.

You may regain manual control anytime by toggling channel 5. During descent, toggling channel 6 will switch back to hover mode.

**Key Parameters:**

* `enable`: Enable auto takeoff/landing
* `enable_auto_arm`: Enable automatic arming. If false, arm manually before sending takeoff
* `takeoff_height`: Desired takeoff altitude
* `takeoff_land_speed`: Takeoff/landing vertical speed

**Troubleshooting:**

* Drone doesn't take off: `hover_percentage` too low (thrust model not active)
* Overshoot in height: controller gain (kp/kv) too low
* Crashes at takeoff: check odometry
* Motors keep spinning after landing: `MPC_MANTHR_MIN` is 0. Set it to 0.08

### Cascaded PID Feedback Control

The control framework calculates attitude or rate commands and throttle feedforward using higher-order derivatives (accel for attitude mode, jerk for rate mode). Lower-order terms (position/velocity) are used for cascaded PID feedback. Currently, only P-terms are used.

**Gain Parameters:**
* gain:
  * `Kp0~Kp2`: Position proportional gains
  * `Kv0~Kv2`: Velocity proportional gains
  * `Kvi0~Kvd2`: Velocity I/D gains (unused, set to 0)
  * `KAngR~KAngY`: Angular error gains (used only in rate mode)

### Simple Thrust Model

A linear model maps desired body-z acceleration to throttle (0\~1). The slope is estimated online.

**Main Parameter:**

* `hover_percentage`: Estimated hover throttle (manual Acro hover level), rough initial value is sufficient (within ±30%). Start with 0.2 for 250mm racers; larger drones may need \~0.5. Set `print_value=true`, switch to Hover mode, and monitor the estimated value. Record the stabilized value and use it as your new `hover_percentage`. Disable printing during normal use.

**If misconfigured:**

* Drone rises on mode switch → value too high
* Drone sinks → value too low
* Ideally, there should be no motion on switching modes

**Other Parameters:**

* `print_value`: Display hover\_percentage or thr\_scale\_compensate during flight
* `accurate_thrust_model`: Enable precise thrust model
* `mass`: Used in accurate model only
* `low_voltage`: Recommended threshold = cell count × 3.3V, used only in accurate model

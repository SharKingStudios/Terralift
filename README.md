# Terralift

Terralift is a mini warehouse automation robot with mecanum drive, a pallet lift, onboard sensing, and a ROS 2 navigation stack. It started as my junior year engineering capstone and is now also the real-hardware platform for comparing Nav2 controller/planner behavior in narrow aisle pallet pickup tasks.

Project background:
- Blog overview: https://loganpeterson.org/blog/en/terralift/

## Features

- Steel chassis with four high-torque drive motors
- Mecanum drivetrain with field-oriented Xbox teleop
- Pallet lift controlled from the robot stack
- RPLIDAR, IMU, camera, AprilTag, and EKF/odom support
- Nav2 bringup for mapping, localization, and goal navigation
- Demo mode with controller-selected preset goals
- Research mode for repeatable Nav2 trials and bag/log capture

## Repo Map

- `terralift/`: Python ROS 2 nodes for drivetrain, lift, odom, demo control, research trials, and helpers
- `launch/`: robot bringup, demo, research, mapping, lidar, and support launch files
- `config/`: EKF, SLAM, camera, and AprilTag configuration
- `nav2/`: Nav2 parameter sets used for demos and research runs
- `scripts/`: robot/laptop launch helpers and research runner scripts
- `desktop/`: Linux desktop shortcut templates for the teleop laptop

## ROS Setup

The current robot/laptop setup assumes:

- ROS 2 Jazzy
- `ROS_DOMAIN_ID=17`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- Robot workspace: `~/terralift_ws`
- Robot source checkout: `~/terralift_ws/src/Terralift`
- Teleop laptop launcher path: `~/terralift_ws/start_terralift_teleop.sh`
- Robot helper path used by the laptop: `~/terralift_ws/bin/start_demo_mode.sh`

The laptop launcher writes a temporary CycloneDDS config that pins discovery to the active `10.42.0.x` robot-network interface when that network is available.

## Build

From the robot or another ROS 2 workspace:

```bash
cd ~/terralift_ws
colcon build --symlink-install
source install/setup.bash
```

Core runtime dependencies used by this package include:

- `nav2_bringup`
- `slam_toolbox`
- `robot_localization`
- `apriltag_ros`
- `v4l2_camera`
- `joy`
- `teleop_twist_joy`
- `rviz2`

Optional hardware backends:

- LEDs: `rpi_ws281x`
- Lift servo: `pigpio` preferred, then `gpiozero`, then `RPi.GPIO`

## Daily Demo Startup

Install the robot-side helper scripts once on the robot:

```bash
cd ~/terralift_ws/src/Terralift
bash scripts/install_robot_tools.sh
```

Start the full demo stack on the robot:

```bash
~/terralift_ws/bin/start_demo_mode.sh \
  --nav2-params dwb_smac2d_course_valid.yaml \
  --use-apriltags false
```

Stop it cleanly:

```bash
~/terralift_ws/bin/stop_demo_mode.sh
```

The helper starts a tmux session named `terralift_demo`, builds the workspace, cleans stale ROS processes, preflights RPLIDAR until `/scan` is working, then launches `terralift_demo.launch.py`. You can find logs in `~/terralift_run_logs/`.

## Teleop Laptop

Install the laptop launcher and desktop shortcuts once on the teleop laptop:

```bash
cd ~/terralift_ws/src/Terralift
bash scripts/install_laptop_tools.sh
```

This creates two shortcuts:

- `Terralift Launcher`: opens a terminal menu, lets you pick a Nav2 config, jumpstarts the robot, opens a robot-output terminal, starts Xbox teleop, and launches RViz.
- `Terralift Wi-Fi Fix`: switches the main Wi-Fi adapter to the Terralift robot network and, if a secondary adapter is connected, tries to keep home Wi-Fi available there.

The launcher uses your existing Xbox config at `~/teleop_mecanum.yaml` and publishes teleop velocity to `/cmd_vel_teleop`. The robot-side arbiter mixes that with Nav2 output and sends the final command to `/cmd_mecanum`.

Nav2 configs exposed by the menu:

- `dwb_smac2d_course_valid.yaml`
- `rpp_smachybrid.yaml`
- `dwb_smac2d.yaml`
- `dwb_smachybrid.yaml`

## Research Mode

Research mode launches the full navigation stack plus the automated trial runner that waits for Nav2 to become active, sends a goal, logs events, and optionally records a bag.

```bash
ros2 launch terralift terralift_research.launch.py \
  use_apriltags:=false \
  nav2_params:=dwb_smac2d_course_valid.yaml
```

Useful launch arguments:

- `nav2_params:=rpp_smac2d.yaml`
- `goal_x:=1.0 goal_y:=0.0 goal_yaw:=0.0`
- `record_bag:=true`
- `auto_close:=true`
- `use_slam:=true`
- `use_lidar:=true`
- `use_apriltags:=true`

## Demo Mode Internals

Demo mode launches the same navigation stack, but it does not send an initial goal. Goals are sent from the Xbox controller D-pad instead, and manual teleop can override Nav2.

```bash
ros2 launch terralift terralift_demo.launch.py
```

Command flow in demo mode:

- The laptop runs `joy_node` and `teleop_twist_joy`.
- The robot listens to remote `/joy` for D-pad preset goals and trigger-based lift control.
- Remote `teleop_twist_joy` publishes to `/cmd_vel_teleop`.
- Nav2 publishes to `/cmd_vel_nav_safe`.
- `cmd_vel_arbiter` publishes the selected command to `/cmd_mecanum`.
- `open_loop_odom` follows the same commanded motion, so forced teleop movement remains reflected in the pose estimate.

Default Xbox mapping assumes the common Linux Xbox layout:

- right trigger: raise lift
- left trigger: lower lift
- D-pad down: send home goal `(0, 0, 0)` in `map`
- D-pad up: tap to send the `pov_up_*` waypoint, hold to save the current pose into that slot
- D-pad left: tap to send the `pov_left_*` waypoint, hold to save the current pose into that slot
- D-pad right: tap to send the `pov_right_*` waypoint, hold to save the current pose into that slot
- `Y`: reset the field-oriented teleop heading so the robot's current facing becomes forward

## Startup Notes

The demo stack is deliberately sequenced because the robot can otherwise start in a half-valid ROS graph:

- RPLIDAR is started first and must publish `/scan` before the heavier stack starts.
- SLAM waits for both `/scan` and `/odom` before configuring and activating.
- Nav2 waits for `/map` before it starts.
- Demo controls wait for `/scan` so controller/lift/arbiter nodes do not race the sensor stack.

The intermittent `frame ID "map" passed to canTransform is invalid` failure was a startup race. When the lidar or SLAM side did not fully come up, Nav2 was able to start before a valid map frame existed, then it spammed TF errors and sometimes killed the stack. Retrying or rebooting sometimes fixed it because the lidar serial negotiation and ROS graph happened to come up in a healthier order.

## Useful Commands

Check the robot helper status:

```bash
~/terralift_ws/bin/start_demo_mode.sh --status
```

Follow the robot log:

```bash
tail -f ~/terralift_run_logs/terralift_demo.log
```

Fresh manual start without the helper:

```bash
cd ~/terralift_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch terralift terralift_demo.launch.py \
  use_apriltags:=false \
  nav2_params:=dwb_smac2d_course_valid.yaml
```

Check the ROS graph from the laptop:

```bash
export ROS_DOMAIN_ID=17
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
ros2 topic list | egrep '(/map$|/tf$|/scan$|/odom$)'
```

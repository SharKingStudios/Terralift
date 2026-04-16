# Terralift

Terralift is a mini warehouse automation robot with mecanum drive, a pallet lift, onboard sensing, and a ROS 2 navigation stack. The platform started as a capstone robot and is also being used to compare Nav2 controller/planner combinations on real hardware in narrow-aisle pallet pickup scenarios with AprilTag-assisted relocalization.

Project background:
- Blog overview: https://loganpeterson.org/blog/en/terralift/

## What This Package Contains

This package owns the Terralift robot-side stack:
- drivetrain control
- lift / servo control
- IMU and odometry helpers
- AprilTag odom reset support
- Nav2 launch integration
- research trial automation
- demo mode teleop, preset goals, and LED feedback

## Build

```bash
colcon build --packages-select terralift
source install/setup.bash
```

Core runtime dependencies used by this package include:
- `nav2_bringup`
- `slam_toolbox`
- `robot_localization`
- `apriltag_ros`
- `v4l2_camera`

Optional hardware backends:
- LEDs: `rpi_ws281x`
- Lift servo: `pigpio` preferred, then `gpiozero`, then `RPi.GPIO`

## Research Mode

Research mode launches the full navigation stack plus the automated trial runner that waits for Nav2 to become active, sends a goal, logs events, and optionally records a bag.

```bash
ros2 launch terralift terralift_research.launch.py
```

Useful launch arguments:
- `nav2_params:=rpp_smac2d.yaml`
- `goal_x:=1.0 goal_y:=0.0 goal_yaw:=0.0`
- `record_bag:=true`
- `auto_close:=true`
- `use_slam:=true`
- `use_apriltags:=true`

## Demo Mode

Demo mode launches the same navigation stack, but it does not send an initial goal. Goals are sent from the Xbox controller D-pad instead, and manual teleop can override Nav2.

```bash
ros2 launch terralift terralift_demo.launch.py
```

Command flow in demo mode:
- A separate laptop runs `joy_node` and `teleop_twist_joy`.
- The robot listens to remote `/joy` for D-pad preset goals and trigger-based lift control.
- Remote `teleop_twist_joy` should publish to a dedicated teleop topic such as `/cmd_vel_teleop`.
- The arbiter output is what drives the robot and what feeds `open_loop_odom`, so forced teleop movement is still reflected in the pose estimate.

### Xbox Mapping

Default mapping assumes `joy_node` publishes the common Linux Xbox layout:
- right trigger: raise lift
- left trigger: lower lift
- D-pad down: send home goal `(0, 0, 0)` in `map`
- D-pad up: send the `pov_up_*` preset
- D-pad left: send the `pov_left_*` preset

### Demo Laptop Side

Typical laptop-side operations:

```bash
ros2 run joy joy_node
ros2 run teleop_twist_joy teleop_node --ros-args --params-file path/to/teleop.yaml -r cmd_vel:=/cmd_vel_teleop
```

The robot-side demo launch defaults to:
- `joy_topic:=/joy`
- `teleop_cmd_topic:=/cmd_vel_teleop`
- `nav_cmd_topic:=/cmd_vel_nav_safe`

### Demo Preset Goals

Default preset goals are:
- `home = (0.0, 0.0, 0.0)`
- `pov_up = (1.0, 0.0, 0.0)`
- `pov_left = (0.0, 1.0, 0.0)`

Example:

```bash
ros2 launch terralift terralift_demo.launch.py \
  pov_up_x:=2.4 pov_up_y:=0.0 pov_up_yaw:=0.0 \
  pov_left_x:=0.0 pov_left_y:=1.8 pov_left_yaw:=1.57
```

## Common Launch Files

- `terralift_research.launch.py`: automated research trial stack
- `terralift_demo.launch.py`: manual demo stack with Xbox control and preset goals
- `terralift_master.launch.py`: older bringup path
- `map_generation.launch.py`: SLAM toolbox mapping support

## Notes

- Demo mode assumes `joy_node` and `teleop_twist_joy` are running on another ROS 2 machine on the same network/domain.
- The D-pad and trigger mappings are parameters in `demo_mode_node.py`, so controller remapping can be done without touching the arbiter or Nav2 wiring.

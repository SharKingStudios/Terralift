"""Gazebo Harmonic + Nav2 bringup for the isolated spatial-belief experiment."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("terralift")
    world = os.path.join(share, "worlds", "spatial_warehouse.sdf")
    nav_params = os.path.join(share, "nav2", "spatial_sim_nav2.yaml")
    scenario = os.path.join(share, "config", "spatial_scenarios", "distance_ranking_flip.json")

    gui = DeclareLaunchArgument("gui", default_value="true")
    scenario_path = DeclareLaunchArgument("scenario_path", default_value=scenario)
    map_yaml = DeclareLaunchArgument("map_yaml", description="Generated static map YAML")
    rviz = DeclareLaunchArgument("rviz", default_value="true", description="Open the RViz sensor/map view")
    map_to_odom_x = DeclareLaunchArgument("map_to_odom_x", default_value="-17.3")
    map_to_odom_y = DeclareLaunchArgument("map_to_odom_y", default_value="-12.0")
    model = DeclareLaunchArgument("model", default_value="")
    results_dir = DeclareLaunchArgument("results_dir", default_value="")
    mock = DeclareLaunchArgument("mock", default_value="false")
    open_chat_terminal = DeclareLaunchArgument("open_chat_terminal", default_value="true")
    auto_close = DeclareLaunchArgument("auto_close", default_value="false")
    run_trial = DeclareLaunchArgument("run_trial", default_value="true")

    gazebo_gui = ExecuteProcess(cmd=["gz", "sim", "-r", world], output="screen", condition=IfCondition(LaunchConfiguration("gui")))
    gazebo_headless = ExecuteProcess(cmd=["gz", "sim", "-r", "-s", world], output="screen", condition=UnlessCondition(LaunchConfiguration("gui")))
    # Launch the bridge directly instead of through ``ros2 run`` so the ROS
    # launch supervisor owns the actual bridge process and no child survives
    # an auto-close trial.
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="spatial_gz_bridge", output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/world/spatial_warehouse/create@ros_gz_interfaces/srv/SpawnEntity",
            "/world/spatial_warehouse/remove@ros_gz_interfaces/srv/DeleteEntity",
            "/world/spatial_warehouse/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ],
    )
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server", output="screen",
        parameters=[{"use_sim_time": True, "yaml_filename": LaunchConfiguration("map_yaml")}],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager", name="map_lifecycle_manager", output="screen",
        parameters=[{"use_sim_time": True, "autostart": True, "node_names": ["map_server"]}],
    )
    # The project-owned odometry bridge publishes both the fixed map -> odom
    # origin and the dynamic odom -> base_link edge.  Keeping these together
    # prevents a second static source from resetting the map origin to zero.
    odom_tf = Node(
        package="terralift", executable="spatial_odom_tf", name="spatial_odom_tf", output="screen",
        parameters=[{
            "use_sim_time": True, "map_to_odom_x": LaunchConfiguration("map_to_odom_x"),
            "map_to_odom_y": LaunchConfiguration("map_to_odom_y"),
        }],
    )
    # Gazebo reports the scan in its scoped sensor frame.  Gazebo's native TF
    # stream is intentionally not bridged; publish this fixed sensor mount so
    # Nav2 can transform /scan into base_link.
    scan_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="spatial_scan_mount",
        # ros_gz_bridge normalizes Gazebo's scoped ``::`` names to slashes
        # in the ROS LaserScan header.
        arguments=["0.55", "0", "0.60", "0", "0", "0", "base_link", "terralift_mecanum/base_link/scan_sensor"],
    )
    # Jazzy's stock navigation_launch also starts route, docking, and collision
    # monitor nodes. The experiment needs only NavigateToPose and uses Gazebo
    # LiDAR directly, so a small explicit lifecycle group avoids unrelated
    # advanced-node configuration and keeps the simulated command topic direct.
    nav_parameters = [nav_params, {"use_sim_time": True}]
    controller = Node(
        package="nav2_controller", executable="controller_server", name="controller_server",
        output="screen", parameters=nav_parameters,
    )
    planner = Node(
        package="nav2_planner", executable="planner_server", name="planner_server",
        output="screen", parameters=nav_parameters,
    )
    behavior = Node(
        package="nav2_behaviors", executable="behavior_server", name="behavior_server",
        output="screen", parameters=nav_parameters,
    )
    navigator = Node(
        package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
        output="screen", parameters=nav_parameters,
    )
    navigation_lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="spatial_navigation_lifecycle_manager", output="screen",
        parameters=[{
            "use_sim_time": True, "autostart": True,
            "node_names": ["controller_server", "planner_server", "behavior_server", "bt_navigator"],
        }],
    )
    rviz_view = Node(
        package="rviz2", executable="rviz2", name="spatial_experiment_rviz", output="screen",
        arguments=["-d", os.path.join(share, "rviz", "spatial_experiment.rviz")],
        parameters=[{"use_sim_time": True}], condition=IfCondition(LaunchConfiguration("rviz")),
    )
    runner = Node(
        package="terralift", executable="spatial_trial_runner", name="spatial_trial_runner", output="screen",
        condition=IfCondition(LaunchConfiguration("run_trial")),
        parameters=[{
            "use_sim_time": True, "scenario_path": LaunchConfiguration("scenario_path"),
            "results_dir": LaunchConfiguration("results_dir"), "model": LaunchConfiguration("model"),
            "mock": LaunchConfiguration("mock"), "open_chat_terminal": LaunchConfiguration("open_chat_terminal"),
            "resource_root": share, "auto_close": LaunchConfiguration("auto_close"),
        }],
    )
    begin_ros = TimerAction(
        period=2.0,
        actions=[bridge, odom_tf, scan_tf, map_server, map_lifecycle,
                 controller, planner, behavior, navigator, navigation_lifecycle],
    )
    begin_trial = TimerAction(period=8.0, actions=[runner])
    shutdown_after_runner = RegisterEventHandler(
        condition=IfCondition(LaunchConfiguration("auto_close")),
        event_handler=OnProcessExit(target_action=runner, on_exit=[EmitEvent(event=Shutdown(reason="spatial trial completed"))]),
    )
    return LaunchDescription([
        gui, scenario_path, map_yaml, rviz, map_to_odom_x, map_to_odom_y, model, results_dir, mock,
        open_chat_terminal, auto_close, run_trial, gazebo_gui, gazebo_headless, begin_ros, rviz_view,
        begin_trial, shutdown_after_runner,
    ])

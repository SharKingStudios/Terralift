#!/usr/bin/env python3
"""Visible, no-AI Nav2 tour through every configured warehouse room."""

from __future__ import annotations

import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .spatial_models import default_scenario_path, load_scenario


class SpatialRoomTour(Node):
    """Drive the simulated robot through B, C, D, E, then back to A."""

    def __init__(self) -> None:
        super().__init__("spatial_room_tour")
        self.declare_parameter("dwell_sec", 2.0)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # An action server can appear before Gazebo's bridged sensor streams
        # have produced the odom -> base_link transform Nav2 needs.  Waiting
        # for both avoids sending the first tour goal during that short window.
        self._odom_ready = threading.Event()
        self._scan_ready = threading.Event()
        self.create_subscription(Odometry, "/odom", lambda _: self._odom_ready.set(), 10)
        self.create_subscription(LaserScan, "/scan", lambda _: self._scan_ready.set(), 10)
        scenario = load_scenario(default_scenario_path())
        # Keep this visibility-only tour aligned with the active scenario
        # instead of duplicating room coordinates in a second file.
        self.waypoints = [
            (room_id, scenario.rooms[room_id].pose.x, scenario.rooms[room_id].pose.y,
             scenario.rooms[room_id].pose.yaw)
            for room_id in ("B", "C", "D", "E", "A")
        ]

    def _wait(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def _wait_for_sensor_streams(self) -> bool:
        deadline = time.monotonic() + 60.0
        while rclpy.ok() and time.monotonic() < deadline:
            if self._odom_ready.is_set() and self._scan_ready.is_set():
                # Permit the odometry-TF bridge and Nav2 costmaps to consume
                # the first valid messages before asking for a plan.
                settle_deadline = time.monotonic() + 2.0
                while rclpy.ok() and time.monotonic() < settle_deadline:
                    rclpy.spin_once(self, timeout_sec=0.1)
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _send_goal(self, room_id: str, x: float, y: float, yaw: float) -> bool:
        """Navigate once, retrying brief startup rejections from Nav2."""
        for attempt in range(1, 4):
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.get_logger().info(
                f"Tour goal: room {room_id} at ({x:.1f}, {y:.1f}) (attempt {attempt}/3)"
            )
            sent = self.client.send_goal_async(goal)
            if not self._wait(sent, 20.0):
                self.get_logger().warn(f"Timed out sending room {room_id}; retrying")
                continue
            handle = sent.result()
            if handle is None or not handle.accepted:
                self.get_logger().warn(f"Nav2 rejected room {room_id}; waiting before retry")
                time.sleep(2.0)
                continue
            result = handle.get_result_async()
            if not self._wait(result, 150.0):
                handle.cancel_goal_async()
                self.get_logger().warn(f"Timed out reaching room {room_id}; retrying")
                continue
            if result.result().status == GoalStatus.STATUS_SUCCEEDED:
                return True
            self.get_logger().warn(
                f"Nav2 returned status {result.result().status} for room {room_id}; retrying"
            )
            time.sleep(2.0)
        self.get_logger().error(f"Nav2 could not reach room {room_id} after three attempts")
        return False

    def drive(self) -> bool:
        self.get_logger().info("Waiting for Nav2 before beginning visible room tour...")
        if not self.client.wait_for_server(timeout_sec=90.0):
            self.get_logger().error("NavigateToPose action server was unavailable")
            return False
        self.get_logger().info("Waiting for bridged odometry and LiDAR before sending the first goal...")
        if not self._wait_for_sensor_streams():
            self.get_logger().error("Timed out waiting for bridged odometry and LiDAR")
            return False
        for room_id, x, y, yaw in self.waypoints:
            if not self._send_goal(room_id, x, y, yaw):
                return False
            self.get_logger().info(f"Reached room {room_id}; holding position for visual inspection")
            time.sleep(float(self.get_parameter("dwell_sec").value))
        self.get_logger().info("Room tour complete. The robot is stopped in room A.")
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpatialRoomTour()
    try:
        if not node.drive():
            raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()

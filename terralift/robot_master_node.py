#!/usr/bin/env python3

import rclpy
import os
import json
import time
from enum import Enum
from rclpy.node import Node

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

from lifecycle_msgs.srv import ChangeState
from slam_toolbox.srv import Pause, Resume, SerializePoseGraph

from builtin_interfaces.msg import Time


class RobotState(Enum):
    DISABLED = 0
    READY = 1
    RUNNING = 2
    DOCKING = 3
    TRIAL_COMPLETE = 4
    FAULT = 5


NAV2_CONFIGS = [
    ("rpp", "smac_2d"),
    ("rpp", "smac_hybrid"),
    ("dwb", "smac_2d"),
    ("dwb", "smac_hybrid"),
]


class RobotMaster(Node):

    def __init__(self):
        super().__init__('robot_master_node')

        # -----------------------------
        # State
        # -----------------------------
        self.state = RobotState.DISABLED
        self.enabled = False
        self.teleop_alive = False
        self.apriltag_visible = False
        self.last_teleop_time = time.time()

        self.current_config_index = 0
        self.trial_count = {cfg: 0 for cfg in NAV2_CONFIGS}
        self.retry_count = 0

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('bag_dir', '/home/pi/terralift_bags')
        self.declare_parameter('teleop_timeout', 1.5)

        self.bag_dir = self.get_parameter('bag_dir').value
        self.teleop_timeout = self.get_parameter('teleop_timeout').value

        os.makedirs(self.bag_dir, exist_ok=True)

        # -----------------------------
        # Publishers
        # -----------------------------
        self.led_pub = self.create_publisher(String, '/led/state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # -----------------------------
        # Subscriptions
        # -----------------------------
        self.create_subscription(Bool, '/teleop/alive', self.teleop_alive_cb, 10)
        self.create_subscription(Bool, '/teleop/enable', self.enable_cb, 10)
        self.create_subscription(Bool, '/teleop/estop', self.estop_cb, 10)

        self.create_subscription(Bool, '/apriltag/detected', self.apriltag_cb, 10)
        self.create_subscription(PoseStamped, '/apriltag/pose', self.apriltag_pose_cb, 10)

        # -----------------------------
        # Services
        # -----------------------------
        self.pause_slam = self.create_client(Pause, '/slam_toolbox/pause_localization')
        self.resume_slam = self.create_client(Resume, '/slam_toolbox/resume_localization')
        self.serialize_slam = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map'
        )

        # -----------------------------
        # Timers
        # -----------------------------
        self.create_timer(0.1, self.watchdog)
        self.create_timer(0.2, self.update_leds)

        self.get_logger().info('Robot Master Node initialized')

    # ============================================================
    # Callbacks
    # ============================================================

    def teleop_alive_cb(self, msg):
        self.teleop_alive = msg.data
        self.last_teleop_time = time.time()

    def enable_cb(self, msg):
        if msg.data:
            self.enable_robot()
        else:
            self.disable_robot()

    def estop_cb(self, msg):
        if msg.data:
            self.enter_fault("Emergency stop")

    def apriltag_cb(self, msg):
        self.apriltag_visible = msg.data

    def apriltag_pose_cb(self, msg):
        if self.state == RobotState.DOCKING:
            if self.docking_success(msg):
                self.complete_trial()

    # ============================================================
    # State Control
    # ============================================================

    def enable_robot(self):
        if self.state in [RobotState.DISABLED, RobotState.TRIAL_COMPLETE]:
            self.enabled = True
            self.state = RobotState.READY
            self.retry_count = 0
            self.get_logger().info("Robot enabled")

    def disable_robot(self):
        self.enabled = False
        self.stop_robot()
        self.state = RobotState.DISABLED
        self.get_logger().warn("Robot disabled")

    def enter_fault(self, reason):
        self.stop_robot()
        self.state = RobotState.FAULT
        self.get_logger().error(f"FAULT: {reason}")

    # ============================================================
    # Trial Logic
    # ============================================================

    def start_trial(self):
        cfg = NAV2_CONFIGS[self.current_config_index]
        self.trial_count[cfg] += 1

        self.current_trial_id = self.generate_trial_id(cfg)
        self.start_rosbag()
        self.resume_slam.call_async(Resume.Request())

        self.state = RobotState.RUNNING
        self.get_logger().info(f"Starting trial {self.current_trial_id}")

    def complete_trial(self):
        self.stop_robot()
        self.pause_slam.call_async(Pause.Request())
        self.stop_rosbag()

        self.state = RobotState.TRIAL_COMPLETE
        self.current_config_index = (self.current_config_index + 1) % len(NAV2_CONFIGS)
        self.get_logger().info("Trial completed successfully")

    def retry_trial(self):
        self.retry_count += 1
        self.start_trial()

    # ============================================================
    # Watchdog
    # ============================================================

    def watchdog(self):
        if not self.enabled:
            return

        if time.time() - self.last_teleop_time > self.teleop_timeout:
            self.enter_fault("Teleop timeout")

        if self.state == RobotState.READY:
            self.start_trial()

        if self.state == RobotState.RUNNING and self.apriltag_visible:
            self.state = RobotState.DOCKING

    # ============================================================
    # Helpers
    # ============================================================

    def docking_success(self, pose):
        dx = pose.pose.position.x
        dy = pose.pose.position.y
        yaw = self.get_yaw(pose.pose.orientation)

        return (dx**2 + dy**2) ** 0.5 < 0.05 and abs(yaw) < 0.07

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def update_leds(self):
        if self.state == RobotState.READY:
            self.led_pub.publish(String(data='GREEN'))
        elif self.state == RobotState.RUNNING:
            self.led_pub.publish(String(data='GREEN_PULSE'))
        elif self.state == RobotState.DOCKING:
            self.led_pub.publish(String(data='GREEN_FAST'))
        elif self.state == RobotState.TRIAL_COMPLETE:
            self.led_pub.publish(String(data='YELLOW'))
        elif self.state == RobotState.FAULT:
            self.led_pub.publish(String(data='RED'))
        else:
            self.led_pub.publish(String(data='OFF'))

    def start_rosbag(self):
        self.bag_path = os.path.join(self.bag_dir, self.current_trial_id)
        os.system(f"ros2 bag record -a -o {self.bag_path} &")
        self.write_metadata()

    def stop_rosbag(self):
        os.system("pkill -f 'ros2 bag record'")

    def write_metadata(self):
        meta = {
            "trial_id": self.current_trial_id,
            "config": NAV2_CONFIGS[self.current_config_index],
            "retry": self.retry_count,
            "timestamp": time.time(),
        }

        with open(os.path.join(self.bag_path, "trial_metadata.json"), "w") as f:
            json.dump(meta, f, indent=2)

    def generate_trial_id(self, cfg):
        return f"REAL_{cfg[0].upper()}_{cfg[1].upper()}_T{self.trial_count[cfg]:02d}"

    @staticmethod
    def get_yaw(q):
        import math
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )


def main():
    rclpy.init()
    node = RobotMaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

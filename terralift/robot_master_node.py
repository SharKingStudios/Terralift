#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

REQUIRED_TOPICS = [
    '/scan',
    '/imu/data',
]


class RobotMasterNode(Node):
    def __init__(self):
        super().__init__('robot_master')

        # ---------------- Parameters ----------------
        self.declare_parameter('teleop_timeout', 0.5)
        self.teleop_timeout = self.get_parameter('teleop_timeout').value

        # ---------------- State ----------------
        self.state = 'BOOTING'
        self.mode = 'teleop'   # or "test"
        self.enabled = False
        self.apriltag_locked = False

        self.last_teleop_time = None

        # ---------------- Publishers ----------------
        self.cmd_pub = self.create_publisher(Twist, 'cmd_mecanum', 10)
        self.state_pub = self.create_publisher(String, 'robot/state', 10)

        # ---------------- Subscriptions ----------------
        self.create_subscription(Twist, 'teleop/cmd_vel', self.teleop_cmd_cb, 10)
        self.create_subscription(Bool, 'teleop/heartbeat', self.teleop_heartbeat_cb, 10)
        self.create_subscription(Bool, 'teleop/enable', self.enable_cb, 10)
        self.create_subscription(Bool, 'teleop/estop', self.estop_cb, 10)
        self.create_subscription(String, 'teleop/mode', self.mode_cb, 10)
        self.create_subscription(Bool, 'teleop/follow_path', self.follow_path_cb, 10)

        # Nav2 velocity input
        self.create_subscription(Twist, 'cmd_vel', self.nav2_cmd_cb, 10)

        # ---------------- Timers ----------------
        self.health_timer = self.create_timer(0.2, self.health_check)
        self.state_timer = self.create_timer(0.1, self.publish_state)

        # ---------------- Internal ----------------
        self.last_nav2_cmd = Twist()
        self.last_teleop_cmd = Twist()
        self.following_path = False

        self.get_logger().info("Robot master node online")

    # =========================================================
    # Teleop
    # =========================================================

    def teleop_heartbeat_cb(self, msg: Bool):
        if msg.data:
            self.last_teleop_time = time.time()

    def enable_cb(self, msg: Bool):
        if msg.data and self.state != 'FAULT':
            self.enabled = True
            self.state = 'ENABLED'
        elif not msg.data:
            self.enabled = False
            self.stop_motion()
            self.state = 'DISABLED'

    def estop_cb(self, msg: Bool):
        if msg.data:
            self.enter_fault("E-STOP")

    def mode_cb(self, msg: String):
        self.mode = msg.data

    def follow_path_cb(self, msg: Bool):
        self.following_path = msg.data

    def teleop_cmd_cb(self, msg: Twist):
        self.last_teleop_cmd = msg

    # =========================================================
    # Nav2
    # =========================================================

    def nav2_cmd_cb(self, msg: Twist):
        self.last_nav2_cmd = msg

    # =========================================================
    # Health + State
    # =========================================================

    def health_check(self):
        # Check teleop timeout
        if self.last_teleop_time:
            if time.time() - self.last_teleop_time > self.teleop_timeout:
                self.enter_fault("TELEOP LOST")

        # BOOTING → READY
        if self.state == 'BOOTING':
            self.state = 'DISABLED'

        # Motion arbitration
        if self.state in ['ENABLED', 'READY']:
            if self.mode == 'teleop':
                if self.following_path:
                    self.cmd_pub.publish(self.last_nav2_cmd)
                else:
                    self.cmd_pub.publish(self.last_teleop_cmd)
            elif self.mode == 'test':
                self.stop_motion()

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    # =========================================================
    # Helpers
    # =========================================================

    def stop_motion(self):
        self.cmd_pub.publish(Twist())

    def enter_fault(self, reason):
        if self.state != 'FAULT':
            self.get_logger().error(f"FAULT: {reason}")
        self.state = 'FAULT'
        self.enabled = False
        self.stop_motion()


def main(args=None):
    rclpy.init(args=args)
    node = RobotMasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

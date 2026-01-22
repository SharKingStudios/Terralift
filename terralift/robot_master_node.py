#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import time

class RobotMasterNode(Node):
    """
    Master control node for Terralift robot.
    Handles:
      - Node monitoring / health
      - Robot state machine
      - LED control
      - Nav2 lifecycle gating
      - Teleop / E-stop / enable
      - Rosbag2 trigger
      - AprilTag lock placeholder
    """

    REQUIRED_NODES = [
        'drivetrain', 'lift_arm', 'imu', 'ekf_filter_node', 'rplidar', 'arducam', 'teleop_node'
    ]

    def __init__(self):
        super().__init__('robot_master_node')

        # ----------------------------
        # Parameters
        # ----------------------------
        self.declare_parameter('heartbeat_timeout', 1.0)
        self.declare_parameter('led_topic', '/led/control')
        self.declare_parameter('rosbag_topic', '/robot/recording/command')
        self.declare_parameter('april_tag_topic', '/apriltag_detected')
        self.declare_parameter('nav2_lifecycle', True)

        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        self.led_topic = self.get_parameter('led_topic').value
        self.rosbag_topic = self.get_parameter('rosbag_topic').value
        self.april_tag_topic = self.get_parameter('april_tag_topic').value
        self.nav2_lifecycle = self.get_parameter('nav2_lifecycle').value

        # ----------------------------
        # Publishers
        # ----------------------------
        self.led_pub = self.create_publisher(String, self.led_topic, 10)
        self.robot_state_pub = self.create_publisher(String, '/robot/state', 10)
        self.rosbag_pub = self.create_publisher(String, self.rosbag_topic, 10)

        # ----------------------------
        # Subscriptions
        # ----------------------------
        # Teleop heartbeat
        self.teleop_heartbeat_sub = self.create_subscription(
            Bool, '/teleop/heartbeat', self.teleop_heartbeat_cb, 10
        )

        # AprilTag detection
        self.april_tag_sub = self.create_subscription(
            Bool, self.april_tag_topic, self.april_cb, 10
        )

        # ----------------------------
        # Internal state
        # ----------------------------
        self.node_last_seen = {node: 0.0 for node in self.REQUIRED_NODES}
        self.teleop_alive = False
        self.robot_state = 'DISABLED'
        self.april_lock = False
        self.enabled = False
        self.in_fault = False
        self.last_teleop = time.time()

        # ----------------------------
        # Timer loops
        # ----------------------------
        self.create_timer(0.1, self.state_machine_loop)
        self.create_timer(0.5, self.led_loop)
        self.create_timer(0.2, self.node_monitor_loop)

        self.get_logger().info("Robot master node initialized")

    # ----------------------------
    # Teleop heartbeat
    # ----------------------------
    def teleop_heartbeat_cb(self, msg: Bool):
        self.teleop_alive = msg.data
        self.last_teleop = time.time()

    # ----------------------------
    # AprilTag detection
    # ----------------------------
    def april_cb(self, msg: Bool):
        self.april_lock = msg.data

    # ----------------------------
    # Node health monitoring
    # ----------------------------
    def node_monitor_loop(self):
        # Placeholder for actual node health checking via rclpy.node_names()
        # For now, we assume nodes publish their heartbeat or master monitors topics
        now = time.time()
        for node in self.node_last_seen.keys():
            # If no heartbeat for 3 sec → mark offline
            last = self.node_last_seen[node]
            if now - last > 3.0:
                self.get_logger().warn(f"Node {node} heartbeat timeout")
                self.set_robot_state('FAULT')
                return

    # ----------------------------
    # State machine loop
    # ----------------------------
    def state_machine_loop(self):
        now = time.time()

        # Teleop disconnect → estop
        if self.enabled and now - self.last_teleop > self.heartbeat_timeout:
            self.get_logger().warn("Teleop disconnected → disabling robot")
            self.enabled = False
            self.in_fault = True
            self.set_robot_state('FAULT')
            # Stop any motion
            self.publish_stop()

        # Fault state
        if self.in_fault:
            self.enabled = False
            self.set_robot_state('FAULT')
            return

        # Check all nodes online
        all_online = all(now - self.node_last_seen[n] < 3.0 for n in self.REQUIRED_NODES)
        if not all_online:
            self.set_robot_state('FAULT')
            return

        # Ready state
        if self.enabled:
            self.set_robot_state('READY')
        else:
            self.set_robot_state('DISABLED')

    # ----------------------------
    # Publish stop motion
    # ----------------------------
    def publish_stop(self):
        twist_pub = self.create_publisher(Twist, '/cmd_mecanum', 10)
        twist_pub.publish(Twist())

    # ----------------------------
    # LED control
    # ----------------------------
    def led_loop(self):
        msg = String()

        if self.robot_state == 'FAULT':
            msg.data = 'RED'
        elif self.robot_state == 'DISABLED':
            msg.data = 'YELLOW_PULSE'
        elif self.robot_state == 'READY':
            msg.data = 'GREEN_FAST' if self.april_lock else 'GREEN_SLOW'
        else:
            msg.data = 'OFF'

        self.led_pub.publish(msg)

    # ----------------------------
    # Robot state helper
    # ----------------------------
    def set_robot_state(self, state: str):
        if self.robot_state != state:
            self.robot_state = state
            msg = String()
            msg.data = state
            self.robot_state_pub.publish(msg)
            self.get_logger().info(f"Robot state: {state}")

    # ----------------------------
    # Rosbag trigger helper
    # ----------------------------
    def trigger_rosbag(self, start: bool):
        msg = String()
        msg.data = 'START' if start else 'STOP'
        self.rosbag_pub.publish(msg)


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

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

import time
import threading

# Optional BLE library for controllers
try:
    from inputs import get_gamepad  # pip install inputs
    BLE_AVAILABLE = True
except ImportError:
    BLE_AVAILABLE = False

# Optional keyboard support
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        # ----------------------------
        # Parameters
        # ----------------------------
        self.declare_parameter('input_mode', 'BLE')  # BLE or KEYBOARD
        self.declare_parameter('linear_scale', 0.5)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('estop_timeout', 2.0)

        self.input_mode = self.get_parameter('input_mode').value.upper()
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.estop_timeout = self.get_parameter('estop_timeout').value

        # ----------------------------
        # Publishers
        # ----------------------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_mecanum', 10)
        self.state_pub = self.create_publisher(String, '/robot/state', 10)
        self.heartbeat_pub = self.create_publisher(Bool, '/teleop/heartbeat', 10)

        # ----------------------------
        # Internal state
        # ----------------------------
        self.enabled = False
        self.last_input_time = time.time()
        self.estop_active = False
        self.twist = Twist()

        # Start input thread
        if self.input_mode == 'BLE' and BLE_AVAILABLE:
            self.get_logger().info("Teleop using BLE controller")
            threading.Thread(target=self.ble_loop, daemon=True).start()
        elif self.input_mode == 'KEYBOARD' and PYGAME_AVAILABLE:
            self.get_logger().info("Teleop using keyboard")
            threading.Thread(target=self.keyboard_loop, daemon=True).start()
        else:
            self.get_logger().warn("No valid input method available")

        # Timer for publishing commands
        self.create_timer(0.05, self.publish_loop)

        # Timer for monitoring input timeout
        self.create_timer(0.2, self.timeout_check)

    # ----------------------------
    # BLE input loop
    # ----------------------------
    def ble_loop(self):
        while rclpy.ok():
            events = get_gamepad()
            for e in events:
                self.handle_ble_event(e)

    def handle_ble_event(self, e):
        # Example mapping, adjust to your controller
        if e.ev_type == 'Absolute':
            if e.code == 'ABS_X':      # Left stick horizontal → strafe
                self.twist.linear.y = self.linear_scale * e.state / 32768
            elif e.code == 'ABS_Y':    # Left stick vertical → forward/back
                self.twist.linear.x = -self.linear_scale * e.state / 32768
            elif e.code == 'ABS_RX':   # Right stick horizontal → rotation
                self.twist.angular.z = self.angular_scale * e.state / 32768

        elif e.ev_type == 'Key':
            if e.code == 'BTN_SOUTH':  # "A" button → enable
                self.enabled = True
                self.estop_active = False
                self.publish_state('READY')
            elif e.code == 'BTN_EAST':  # "B" button → estop
                self.enabled = False
                self.estop_active = True
                self.publish_state('FAULT')

        self.last_input_time = time.time()

    # ----------------------------
    # Keyboard loop
    # ----------------------------
    def keyboard_loop(self):
        pygame.init()
        screen = pygame.display.set_mode((100, 100))
        pygame.display.set_caption("Teleop Keyboard Input")
        clock = pygame.time.Clock()

        while rclpy.ok():
            for event in pygame.event.get():
                self.handle_key_event(event)
            clock.tick(50)

    def handle_key_event(self, event):
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                self.twist.linear.x = self.linear_scale
            elif event.key == pygame.K_s:
                self.twist.linear.x = -self.linear_scale
            elif event.key == pygame.K_a:
                self.twist.linear.y = self.linear_scale
            elif event.key == pygame.K_d:
                self.twist.linear.y = -self.linear_scale
            elif event.key == pygame.K_LEFT:
                self.twist.angular.z = self.angular_scale
            elif event.key == pygame.K_RIGHT:
                self.twist.angular.z = -self.angular_scale
            elif event.key == pygame.K_e:
                self.enabled = True
                self.estop_active = False
                self.publish_state('READY')
            elif event.key == pygame.K_SPACE:
                self.enabled = False
                self.estop_active = True
                self.publish_state('FAULT')
        elif event.type == pygame.KEYUP:
            # Stop motion on release
            self.twist = Twist()
        self.last_input_time = time.time()

    # ----------------------------
    # Publish commands
    # ----------------------------
    def publish_loop(self):
        # If disabled or estop → zero twist
        if not self.enabled or self.estop_active:
            safe_twist = Twist()
            self.cmd_pub.publish(safe_twist)
        else:
            self.cmd_pub.publish(self.twist)

        # Publish heartbeat
        hb = Bool()
        hb.data = True
        self.heartbeat_pub.publish(hb)

    # ----------------------------
    # Input timeout check
    # ----------------------------
    def timeout_check(self):
        if time.time() - self.last_input_time > self.estop_timeout:
            if self.enabled:
                self.get_logger().warn("Input timeout → estop activated")
                self.enabled = False
                self.estop_active = True
                self.publish_state('FAULT')

    # ----------------------------
    # State helper
    # ----------------------------
    def publish_state(self, state: str):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.get_logger().info(f"Robot state: {state}")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

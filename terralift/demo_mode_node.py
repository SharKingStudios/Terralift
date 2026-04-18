#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String


LED_IDLE = 'IDLE_CONFETTI'
LED_PLANNING = 'PLANNING_YELLOW'
LED_FOLLOWING = 'FOLLOWING_GREEN_FLASH'


def clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def twist_magnitude(msg: Twist) -> float:
    return math.sqrt(
        msg.linear.x * msg.linear.x +
        msg.linear.y * msg.linear.y +
        msg.angular.z * msg.angular.z
    )


class DemoModeNode(Node):
    def __init__(self) -> None:
        super().__init__('demo_mode')

        self.joy_topic = str(self.declare_parameter('joy_topic', '/joy').value)
        self.nav_cmd_topic = str(self.declare_parameter('nav_cmd_topic', '/cmd_vel_nav_safe').value)
        self.lift_topic = str(self.declare_parameter('lift_topic', '/lift_arm/command').value)
        self.led_topic = str(self.declare_parameter('led_topic', '/led/state').value)
        self.goal_pose_topic = str(self.declare_parameter('goal_pose_topic', '/demo/goal_pose').value)
        self.goal_frame = str(self.declare_parameter('goal_frame', 'map').value)

        self.publish_hz = float(self.declare_parameter('publish_hz', 50.0).value)
        self.joy_timeout_sec = float(self.declare_parameter('joy_timeout_sec', 0.5).value)
        self.following_cmd_threshold = float(
            self.declare_parameter('following_cmd_threshold', 0.05).value
        )

        self.axis_left_trigger = int(self.declare_parameter('axis_left_trigger', 2).value)
        self.axis_right_trigger = int(self.declare_parameter('axis_right_trigger', 5).value)
        self.axis_dpad_x = int(self.declare_parameter('axis_dpad_x', 6).value)
        self.axis_dpad_y = int(self.declare_parameter('axis_dpad_y', 7).value)

        self.trigger_released_value = float(
            self.declare_parameter('trigger_released_value', 1.0).value
        )
        self.trigger_pressed_value = float(
            self.declare_parameter('trigger_pressed_value', -1.0).value
        )
        self.trigger_deadband = float(self.declare_parameter('trigger_deadband', 0.08).value)
        self.dpad_threshold = float(self.declare_parameter('dpad_threshold', 0.5).value)
        self.dpad_left_negative = bool(self.declare_parameter('dpad_left_negative', True).value)
        self.dpad_up_positive = bool(self.declare_parameter('dpad_up_positive', True).value)

        self.lift_min_deg = float(self.declare_parameter('lift_min_deg', 0.0).value)
        self.lift_max_deg = float(self.declare_parameter('lift_max_deg', 180.0).value)
        self.lift_initial_deg = float(self.declare_parameter('lift_initial_deg', 90.0).value)
        self.lift_rate_deg_per_sec = float(
            self.declare_parameter('lift_rate_deg_per_sec', 160.0).value
        )

        self.home_goal = self._load_goal_param('home', 0.0, 0.0, 0.0)
        self.pov_up_goal = self._load_goal_param('pov_up', 1.0, 0.0, 0.0)
        self.pov_left_goal = self._load_goal_param('pov_left', 0.0, 1.0, 0.0)

        self.lift_pub = self.create_publisher(Float32, self.lift_topic, 10)
        self.led_pub = self.create_publisher(String, self.led_topic, 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, self.goal_pose_topic, 10)

        self.create_subscription(Joy, self.joy_topic, self._joy_cb, 10)
        self.create_subscription(Twist, self.nav_cmd_topic, self._nav_cmd_cb, 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.last_joy = Joy()
        self.last_joy_stamp = None
        self.last_tick = self.get_clock().now()
        self.prev_dpad = {'up': False, 'down': False, 'left': False}

        self.lift_target_deg = clamp(self.lift_initial_deg, self.lift_min_deg, self.lift_max_deg)
        self.last_led_mode = ''
        self.goal_active = False
        self.goal_cancel_requested = False
        self.following_started = False
        self.goal_request_pending = False
        self.goal_handle = None
        self.pending_goal: Optional[Tuple[str, Tuple[float, float, float]]] = None

        self.timer = self.create_timer(1.0 / max(1.0, self.publish_hz), self._tick)

        self._set_led_mode(LED_IDLE)
        self._publish_lift_target()

        self.get_logger().info(
            'Demo mode ready: D-pad down=home, up=preset, left=preset, '
            'right trigger raises lift, left trigger lowers lift'
        )

    def _load_goal_param(self, prefix: str, default_x: float, default_y: float, default_yaw: float):
        x = float(self.declare_parameter(f'{prefix}_x', default_x).value)
        y = float(self.declare_parameter(f'{prefix}_y', default_y).value)
        yaw = float(self.declare_parameter(f'{prefix}_yaw', default_yaw).value)
        return (x, y, yaw)

    def _joy_cb(self, msg: Joy) -> None:
        self.last_joy = msg
        self.last_joy_stamp = self.get_clock().now()

        dpad = self._read_dpad(msg)
        if dpad['down'] and not self.prev_dpad['down']:
            self._request_goal('home', self.home_goal)
        if dpad['up'] and not self.prev_dpad['up']:
            self._request_goal('pov_up', self.pov_up_goal)
        if dpad['left'] and not self.prev_dpad['left']:
            self._request_goal('pov_left', self.pov_left_goal)
        self.prev_dpad = dpad

    def _nav_cmd_cb(self, msg: Twist) -> None:
        if self.goal_active and not self.following_started and twist_magnitude(msg) > self.following_cmd_threshold:
            self.following_started = True
            self._set_led_mode(LED_FOLLOWING)

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = max(0.0, (now - self.last_tick).nanoseconds * 1e-9)
        self.last_tick = now

        joy_fresh = self._joy_is_fresh(now)
        if joy_fresh:
            raise_amount = self._trigger_amount(self.last_joy, self.axis_right_trigger)
            lower_amount = self._trigger_amount(self.last_joy, self.axis_left_trigger)
            lift_delta = (raise_amount - lower_amount) * self.lift_rate_deg_per_sec * dt
            if abs(lift_delta) > 1e-6:
                self.lift_target_deg = clamp(
                    self.lift_target_deg + lift_delta,
                    self.lift_min_deg,
                    self.lift_max_deg,
                )
                self._publish_lift_target()

    def _joy_is_fresh(self, now) -> bool:
        if self.last_joy_stamp is None:
            return False
        age = (now - self.last_joy_stamp).nanoseconds * 1e-9
        return age <= self.joy_timeout_sec

    def _trigger_amount(self, joy: Joy, index: int) -> float:
        raw = self._axis_value(joy, index, self.trigger_released_value)
        span = self.trigger_released_value - self.trigger_pressed_value
        if abs(span) < 1e-6:
            return 0.0
        amount = (self.trigger_released_value - raw) / span
        amount = clamp(amount, 0.0, 1.0)
        if amount <= self.trigger_deadband:
            return 0.0
        scaled_span = max(1e-6, 1.0 - self.trigger_deadband)
        return clamp((amount - self.trigger_deadband) / scaled_span, 0.0, 1.0)

    def _axis_value(self, joy: Joy, index: int, default: float) -> float:
        if index < 0 or index >= len(joy.axes):
            return default
        return float(joy.axes[index])

    def _read_dpad(self, joy: Joy):
        dpad_x = self._axis_value(joy, self.axis_dpad_x, 0.0)
        dpad_y = self._axis_value(joy, self.axis_dpad_y, 0.0)
        up_pressed = dpad_y >= self.dpad_threshold if self.dpad_up_positive else dpad_y <= -self.dpad_threshold
        down_pressed = dpad_y <= -self.dpad_threshold if self.dpad_up_positive else dpad_y >= self.dpad_threshold
        left_pressed = dpad_x <= -self.dpad_threshold if self.dpad_left_negative else dpad_x >= self.dpad_threshold
        return {
            'up': up_pressed,
            'down': down_pressed,
            'left': left_pressed,
        }

    def _request_goal(self, name: str, pose_tuple: Tuple[float, float, float]) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=0.25):
            self.get_logger().warn('navigate_to_pose is not available yet')
            return

        if self.goal_active or self.goal_request_pending:
            self.pending_goal = (name, pose_tuple)
            if self.goal_handle is not None and not self.goal_cancel_requested:
                self.goal_cancel_requested = True
                self.get_logger().info(f'Canceling current goal before sending {name}')
                self.goal_handle.cancel_goal_async()
            return

        self._send_goal(name, pose_tuple)

    def _send_goal(self, name: str, pose_tuple: Tuple[float, float, float]) -> None:
        pose = self._build_pose(*pose_tuple)
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.goal_request_pending = True
        self.goal_active = True
        self.goal_cancel_requested = False
        self.following_started = False
        self.goal_pose_pub.publish(pose)
        self._set_led_mode(LED_PLANNING)

        self.get_logger().info(
            f'Sending {name} goal -> x={pose_tuple[0]:.3f}, y={pose_tuple[1]:.3f}, yaw={pose_tuple[2]:.3f}'
        )
        future = self.nav_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _build_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.goal_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(0.5 * yaw)
        msg.pose.orientation.w = math.cos(0.5 * yaw)
        return msg

    def _goal_response_cb(self, future) -> None:
        self.goal_request_pending = False
        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self.goal_active = False
            self.goal_request_pending = False
            self.goal_handle = None
            self._set_led_mode(LED_IDLE)
            self.get_logger().error(f'Goal request failed: {exc}')
            if self.pending_goal is not None:
                next_goal = self.pending_goal
                self.pending_goal = None
                self._send_goal(*next_goal)
            return

        if self.goal_handle is None or not self.goal_handle.accepted:
            self.goal_active = False
            self.goal_request_pending = False
            self.goal_handle = None
            self._set_led_mode(LED_IDLE)
            self.get_logger().warn('Demo goal rejected by Nav2')
            if self.pending_goal is not None:
                next_goal = self.pending_goal
                self.pending_goal = None
                self._send_goal(*next_goal)
            return

        if self.pending_goal is not None and not self.goal_cancel_requested:
            self.goal_cancel_requested = True
            self.get_logger().info('Preempting newly accepted goal with queued preset request')
            self.goal_handle.cancel_goal_async()

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _feedback_cb(self, _feedback_msg) -> None:
        if self.goal_active and not self.following_started:
            self.following_started = True
            self._set_led_mode(LED_FOLLOWING)

    def _goal_result_cb(self, future) -> None:
        self.goal_active = False
        self.goal_request_pending = False
        self.goal_cancel_requested = False
        self.following_started = False

        try:
            wrapped = future.result()
            status = int(wrapped.status)
            result = wrapped.result
            error_code = int(getattr(result, 'error_code', 0))
            error_msg = str(getattr(result, 'error_msg', ''))
        except Exception as exc:
            status = GoalStatus.STATUS_UNKNOWN
            error_code = -1
            error_msg = str(exc)

        if status == GoalStatus.STATUS_SUCCEEDED and error_code == 0:
            self.get_logger().info('Demo goal completed successfully')
        else:
            self.get_logger().warn(
                f'Demo goal ended with {self._goal_status_name(status)} '
                f'(error_code={error_code}, error_msg="{error_msg}")'
            )

        self.goal_handle = None
        self._set_led_mode(LED_IDLE)

        if self.pending_goal is not None:
            next_goal = self.pending_goal
            self.pending_goal = None
            self._send_goal(*next_goal)

    def _publish_lift_target(self) -> None:
        self.lift_pub.publish(Float32(data=float(self.lift_target_deg)))

    def _set_led_mode(self, mode: str) -> None:
        if mode == self.last_led_mode:
            return
        self.last_led_mode = mode
        self.led_pub.publish(String(data=mode))

    @staticmethod
    def _goal_status_name(status: int) -> str:
        mapping = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return mapping.get(status, f'CODE_{status}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoModeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

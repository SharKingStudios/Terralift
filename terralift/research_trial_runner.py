#!/usr/bin/env python3
"""Automated research trial runner for Terralift.

This node is responsible for:
- creating a unique trial_id
- writing a metadata JSON file for the trial
- starting and stopping ros2 bag recording
- publishing a configurable NavigateToPose goal
- logging high-level trial events to a topic that is also bagged
- optionally exiting when the trial succeeds or fails so the launch file can auto-close

The node is intentionally launch-driven so repeated experiments can be run from a shell
script by varying only launch arguments.
"""

from __future__ import annotations

import datetime
import json
import math
import os
import re
import signal
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from std_msgs.msg import String


ACTION_TYPE_NAMES = {
    0: 'DO_NOTHING',
    1: 'STOP',
    2: 'SLOWDOWN',
    3: 'APPROACH',
    4: 'LIMIT',
}


@dataclass
class TrialResult:
    success: bool
    reason: str
    error_code: int = 0
    error_msg: str = ''


class ResearchTrialRunner(Node):
    def __init__(self) -> None:
        super().__init__('research_trial_runner')

        # ----------------------------
        # Parameters
        # ----------------------------
        self.declare_parameter('record_bag', True)
        self.declare_parameter('record_all_topics', False)
        self.declare_parameter('bag_base_dir', '~/terralift_bags')
        self.declare_parameter('trial_prefix', 'trial')
        self.declare_parameter('environment_id', 'default_env')
        self.declare_parameter('trial_notes', '')
        self.declare_parameter('nav2_params_name', 'rpp_smac2d.yaml')
        self.declare_parameter('parameter_set_id', '')
        self.declare_parameter('nav2_version', 'unknown')
        self.declare_parameter('robot_model', 'terralift')
        self.declare_parameter('robot_firmware_driver_versions', 'unknown')
        self.declare_parameter('use_apriltags', True)
        self.declare_parameter('use_slam', True)
        self.declare_parameter('auto_close', True)
        self.declare_parameter('startup_delay_sec', 4.0)
        self.declare_parameter('goal_timeout_sec', 90.0)
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('goal_pose_topic', '/research/goal_pose')
        self.declare_parameter('event_topic', '/research/trial_event')
        self.declare_parameter('led_topic', '/led/state')
        self.declare_parameter('collision_monitor_state_topic', '/collision_monitor_state')
        self.declare_parameter('cmd_vel_smoothed_topic', '/cmd_vel_smoothed')
        self.declare_parameter('cmd_vel_safe_topic', '/cmd_vel_nav_safe')

        # Wait until bt_navigator is fully ACTIVE before sending the goal
        self.declare_parameter('bt_navigator_state_service', '/bt_navigator/get_state')
        self.declare_parameter('wait_for_nav_active_timeout_sec', 30.0)
        self.declare_parameter('nav_active_poll_period_sec', 0.5)

        self.declare_parameter('record_topics', [
            '/tf',
            '/tf_static',
            '/map',
            '/odom',
            '/scan',
            '/imu/data',
            '/cmd_vel',
            '/cmd_vel_smoothed',
            '/cmd_vel_nav_safe',
            '/cmd_mecanum',
            '/plan',
            '/local_plan',
            '/global_costmap/costmap',
            '/global_costmap/costmap_raw',
            '/local_costmap/costmap',
            '/local_costmap/costmap_raw',
            '/collision_monitor_state',
            '/research/goal_pose',
            '/research/trial_event',
            '/apriltag/detections',
            '/tag_reset_pose',
        ])

        self.record_bag = bool(self.get_parameter('record_bag').value)
        self.record_all_topics = bool(self.get_parameter('record_all_topics').value)
        self.bag_base_dir = Path(os.path.expanduser(str(self.get_parameter('bag_base_dir').value)))
        self.trial_prefix = str(self.get_parameter('trial_prefix').value).strip() or 'trial'
        self.environment_id = str(self.get_parameter('environment_id').value).strip() or 'default_env'
        self.trial_notes = str(self.get_parameter('trial_notes').value)
        self.nav2_params_name = str(self.get_parameter('nav2_params_name').value)
        self.parameter_set_id = str(self.get_parameter('parameter_set_id').value).strip() or self.nav2_params_name
        self.nav2_version = str(self.get_parameter('nav2_version').value)
        self.robot_model = str(self.get_parameter('robot_model').value)
        self.robot_firmware_driver_versions = str(self.get_parameter('robot_firmware_driver_versions').value)
        self.use_apriltags = bool(self.get_parameter('use_apriltags').value)
        self.use_slam = bool(self.get_parameter('use_slam').value)
        self.auto_close = bool(self.get_parameter('auto_close').value)
        self.startup_delay_sec = float(self.get_parameter('startup_delay_sec').value)
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.goal_frame = str(self.get_parameter('goal_frame').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_yaw = float(self.get_parameter('goal_yaw').value)
        self.goal_pose_topic = str(self.get_parameter('goal_pose_topic').value)
        self.event_topic = str(self.get_parameter('event_topic').value)
        self.led_topic = str(self.get_parameter('led_topic').value)
        self.collision_monitor_state_topic = str(self.get_parameter('collision_monitor_state_topic').value)
        self.cmd_vel_smoothed_topic = str(self.get_parameter('cmd_vel_smoothed_topic').value)
        self.cmd_vel_safe_topic = str(self.get_parameter('cmd_vel_safe_topic').value)
        self.bt_navigator_state_service = str(self.get_parameter('bt_navigator_state_service').value)
        self.wait_for_nav_active_timeout_sec = float(self.get_parameter('wait_for_nav_active_timeout_sec').value)
        self.nav_active_poll_period_sec = float(self.get_parameter('nav_active_poll_period_sec').value)
        self.record_topics = [str(t) for t in self.get_parameter('record_topics').value]

        self.bag_base_dir.mkdir(parents=True, exist_ok=True)

        # ----------------------------
        # Internal state
        # ----------------------------
        self.bag_process: Optional[subprocess.Popen] = None
        self.goal_handle = None
        self.result_future = None
        self.finished = False
        self.start_timer = None
        self.timeout_timer = None
        self.nav_state_timer = None
        self.nav_state_future = None
        self.nav_wait_start_time = None
        self.last_bt_state_label: Optional[str] = None

        self.last_collision_state: Optional[Tuple[int, str]] = None
        self.last_cmd_smoothed = Twist()
        self.last_cmd_safe = Twist()
        self.safety_stop_active = False
        self.pending_exit_code = 0
        self.following_started = False
        self.last_led_mode = ''

        self.controller_id, self.planner_id = self._parse_nav2_config_name(self.nav2_params_name)
        self.config_id = f'{self.controller_id}_{self.planner_id}'
        self.today_tag = datetime.datetime.now().strftime('%Y%m%d')
        self.trial_index = self._next_trial_index()
        self.trial_id = (
            f'{self.trial_prefix}_{self.environment_id}_{self.config_id}_{self.today_tag}_'
            f't{self.trial_index:02d}'
        )
        self.trial_dir = self.bag_base_dir / self.trial_id
        self.trial_dir.mkdir(parents=True, exist_ok=True)
        self.metadata_path = self.trial_dir / 'trial_metadata.json'

        # ----------------------------
        # ROS interfaces
        # ----------------------------
        event_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.event_pub = self.create_publisher(String, self.event_topic, event_qos)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_pose_topic, event_qos)
        self.led_pub = self.create_publisher(String, self.led_topic, 10)

        self.create_subscription(
            CollisionMonitorState,
            self.collision_monitor_state_topic,
            self._collision_monitor_cb,
            10,
        )
        self.create_subscription(Twist, self.cmd_vel_smoothed_topic, self._cmd_smoothed_cb, 10)
        self.create_subscription(Twist, self.cmd_vel_safe_topic, self._cmd_safe_cb, 10)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.bt_state_client = self.create_client(GetState, self.bt_navigator_state_service)

        # ----------------------------
        # Trial startup
        # ----------------------------
        self._write_metadata(status='initialized')
        self._publish_event('trial_initialized', {
            'trial_id': self.trial_id,
            'config_id': self.config_id,
            'trial_dir': str(self.trial_dir),
        })
        self._set_led_mode('IDLE_CONFETTI')

        self.get_logger().info(
            f'Research trial runner ready for {self.trial_id}; '
            f'goal=({self.goal_x:.3f}, {self.goal_y:.3f}, yaw={self.goal_yaw:.3f}) in {self.goal_frame}'
        )

        self.start_timer = self.create_timer(self.startup_delay_sec, self._begin_trial_once)

    # ------------------------------------------------------------------
    # Trial orchestration
    # ------------------------------------------------------------------

    def _begin_trial_once(self) -> None:
        if self.start_timer is not None:
            self.start_timer.cancel()
            self.start_timer = None

        self.get_logger().info('Beginning research trial')
        self._publish_event('trial_start', {
            'trial_id': self.trial_id,
            'startup_delay_sec': self.startup_delay_sec,
        })
        self._write_metadata(status='starting')
        self._set_led_mode('PLANNING_YELLOW')

        if self.record_bag:
            self._start_bag_recording()

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=15.0):
            self._finish_trial(TrialResult(False, 'navigate_to_pose action server unavailable'))
            return

        # Wait until bt_navigator lifecycle node is ACTIVE before sending the goal
        self.nav_wait_start_time = self.get_clock().now()
        self._publish_event('waiting_for_nav2_active', {
            'service': self.bt_navigator_state_service,
            'timeout_sec': self.wait_for_nav_active_timeout_sec,
        })

        self.nav_state_timer = self.create_timer(
            self.nav_active_poll_period_sec,
            self._poll_bt_navigator_active,
        )

    def _poll_bt_navigator_active(self) -> None:
        if self.finished:
            return

        if self.nav_wait_start_time is None:
            self.nav_wait_start_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.nav_wait_start_time).nanoseconds * 1e-9
        if elapsed > self.wait_for_nav_active_timeout_sec:
            if self.nav_state_timer is not None:
                self.nav_state_timer.cancel()
                self.nav_state_timer = None
            self._finish_trial(
                TrialResult(
                    False,
                    f'bt_navigator did not become active within {self.wait_for_nav_active_timeout_sec:.1f}s',
                )
            )
            return

        if not self.bt_state_client.service_is_ready():
            return

        if self.nav_state_future is not None and not self.nav_state_future.done():
            return

        req = GetState.Request()
        self.nav_state_future = self.bt_state_client.call_async(req)
        self.nav_state_future.add_done_callback(self._bt_state_response_cb)

    def _bt_state_response_cb(self, future) -> None:
        if self.finished:
            return

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Failed to query bt_navigator state: {exc}')
            return

        label = str(response.current_state.label).strip().lower()
        state_id = int(response.current_state.id)

        if label != self.last_bt_state_label:
            self.last_bt_state_label = label
            self._publish_event('bt_navigator_state', {
                'state_id': state_id,
                'state_label': label,
            })

        if label != 'active':
            return

        if self.nav_state_timer is not None:
            self.nav_state_timer.cancel()
            self.nav_state_timer = None

        self.get_logger().info('bt_navigator is ACTIVE; sending goal')
        self._send_navigation_goal()

    def _send_navigation_goal(self) -> None:
        goal = self._build_goal_pose()
        self.goal_pub.publish(goal)
        self._publish_event('goal_published', self._pose_dict(goal))
        self.following_started = False
        self._set_led_mode('PLANNING_YELLOW')

        action_goal = NavigateToPose.Goal()
        action_goal.pose = goal

        send_future = self.nav_to_pose_client.send_goal_async(
            action_goal,
            feedback_callback=self._feedback_cb,
        )
        send_future.add_done_callback(self._goal_response_cb)

        self.timeout_timer = self.create_timer(self.goal_timeout_sec, self._goal_timeout_cb)

    def _goal_response_cb(self, future) -> None:
        try:
            self.goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - defensive
            self._finish_trial(TrialResult(False, f'goal request failed: {exc}'))
            return

        if self.goal_handle is None or not self.goal_handle.accepted:
            self._finish_trial(TrialResult(False, 'goal was rejected by navigate_to_pose'))
            return

        self._publish_event('goal_accepted', {'trial_id': self.trial_id})
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        if not self.following_started:
            self.following_started = True
            self._set_led_mode('FOLLOWING_GREEN_FLASH')
        self._publish_event('goal_feedback', {
            'distance_remaining_m': float(feedback.distance_remaining),
            'number_of_recoveries': int(feedback.number_of_recoveries),
            'navigation_time_sec': self._duration_to_sec(feedback.navigation_time),
            'estimated_time_remaining_sec': self._duration_to_sec(feedback.estimated_time_remaining),
            'current_pose': self._pose_dict(feedback.current_pose),
        })

    def _result_cb(self, future) -> None:
        try:
            wrapped_result = future.result()
        except Exception as exc:  # pragma: no cover - defensive
            self._finish_trial(TrialResult(False, f'goal result retrieval failed: {exc}'))
            return

        status = int(wrapped_result.status)
        result = wrapped_result.result
        error_code = int(getattr(result, 'error_code', 0))
        error_msg = str(getattr(result, 'error_msg', ''))

        if status == GoalStatus.STATUS_SUCCEEDED and error_code == 0:
            self._finish_trial(TrialResult(True, 'goal_reached', error_code, error_msg))
            return

        status_text = self._goal_status_name(status)
        reason = f'navigation_finished_with_{status_text.lower()}'
        self._finish_trial(TrialResult(False, reason, error_code, error_msg))

    def _goal_timeout_cb(self) -> None:
        if self.finished:
            return

        self._publish_event('goal_timeout', {
            'goal_timeout_sec': self.goal_timeout_sec,
        })

        if self.goal_handle is not None:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda _f: self._finish_trial(TrialResult(False, 'goal_timeout_cancelled'))
            )
        else:
            self._finish_trial(TrialResult(False, 'goal_timeout_before_accept'))

    def _finish_trial(self, result: TrialResult) -> None:
        if self.finished:
            return
        self.finished = True

        if self.start_timer is not None:
            self.start_timer.cancel()
            self.start_timer = None

        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
            self.timeout_timer = None

        if self.nav_state_timer is not None:
            self.nav_state_timer.cancel()
            self.nav_state_timer = None

        if result.success:
            self._publish_event('trial_complete', {
                'trial_id': self.trial_id,
                'result': 'success',
                'reason': result.reason,
            })
            self.pending_exit_code = 0
            self._set_led_mode('IDLE_CONFETTI')
        else:
            self._publish_event('trial_complete', {
                'trial_id': self.trial_id,
                'result': 'failure',
                'reason': result.reason,
                'error_code': result.error_code,
                'error_msg': result.error_msg,
            })
            self.pending_exit_code = 1
            self._set_led_mode('RED')

        self._write_metadata(
            status='complete',
            final_result={
                'success': result.success,
                'reason': result.reason,
                'error_code': result.error_code,
                'error_msg': result.error_msg,
                'finished_at': datetime.datetime.now().isoformat(),
            },
        )

        self._stop_bag_recording()

        if self.auto_close:
            self.get_logger().info('Trial finished; auto_close enabled, shutting down node')
            self.create_timer(0.5, self._shutdown_once)
        else:
            self.get_logger().info('Trial finished; auto_close disabled, keeping launch alive')

    def _shutdown_once(self) -> None:
        if getattr(self, '_shutdown_started', False):
            return
        self._shutdown_started = True
        rclpy.shutdown()

    # ------------------------------------------------------------------
    # Event / metadata helpers
    # ------------------------------------------------------------------

    def _publish_event(self, name: str, payload: Optional[Dict[str, Any]] = None) -> None:
        msg = String()
        body = {
            'event': name,
            'stamp': datetime.datetime.now().isoformat(),
            'trial_id': self.trial_id,
        }
        if payload:
            body.update(payload)
        msg.data = json.dumps(body, sort_keys=True)
        self.event_pub.publish(msg)
        self.get_logger().info(f'EVENT {name}: {json.dumps(payload or {}, sort_keys=True)}')

    def _set_led_mode(self, mode: str) -> None:
        if mode == self.last_led_mode:
            return
        self.last_led_mode = mode
        self.led_pub.publish(String(data=mode))

    def _write_metadata(self, status: str, final_result: Optional[Dict[str, Any]] = None) -> None:
        metadata = {
            'trial_id': self.trial_id,
            'status': status,
            'created_at': datetime.datetime.now().isoformat(),
            'environment_id': self.environment_id,
            'config_id': self.config_id,
            'controller_id': self.controller_id,
            'planner_id': self.planner_id,
            'nav2_params_name': self.nav2_params_name,
            'parameter_set_id': self.parameter_set_id,
            'nav2_version': self.nav2_version,
            'robot_model': self.robot_model,
            'robot_firmware_driver_versions': self.robot_firmware_driver_versions,
            'use_apriltags': self.use_apriltags,
            'use_slam': self.use_slam,
            'goal': {
                'frame_id': self.goal_frame,
                'x': self.goal_x,
                'y': self.goal_y,
                'yaw': self.goal_yaw,
            },
            'trial_notes': self.trial_notes,
            'bag_enabled': self.record_bag,
            'record_all_topics': self.record_all_topics,
            'record_topics': self.record_topics,
            'auto_close': self.auto_close,
            'goal_timeout_sec': self.goal_timeout_sec,
            'startup_delay_sec': self.startup_delay_sec,
            'bt_navigator_state_service': self.bt_navigator_state_service,
            'wait_for_nav_active_timeout_sec': self.wait_for_nav_active_timeout_sec,
            'nav_active_poll_period_sec': self.nav_active_poll_period_sec,
        }
        if final_result is not None:
            metadata['final_result'] = final_result

        with self.metadata_path.open('w', encoding='utf-8') as f:
            json.dump(metadata, f, indent=2, sort_keys=True)

    # ------------------------------------------------------------------
    # Bag recording
    # ------------------------------------------------------------------

    def _start_bag_recording(self) -> None:
        if self.bag_process is not None:
            return

        bag_uri = str(self.trial_dir / 'bag')
        cmd = ['ros2', 'bag', 'record', '-o', bag_uri]

        if self.record_all_topics:
            cmd.extend(['-a', '--include-hidden-topics'])
        else:
            cmd.extend(self.record_topics)

        self.get_logger().info('Starting bag recording: ' + ' '.join(cmd))
        try:
            self.bag_process = subprocess.Popen(
                cmd,
                cwd=str(self.trial_dir),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
        except Exception as exc:
            self._publish_event('bag_recording_failed', {
                'bag_uri': bag_uri,
                'error': str(exc),
            })
            self.bag_process = None
            return

        self._publish_event('bag_recording_started', {
            'bag_uri': bag_uri,
            'record_all_topics': self.record_all_topics,
        })

    def _stop_bag_recording(self) -> None:
        if self.bag_process is None:
            return

        self.get_logger().info('Stopping bag recording')
        try:
            os.killpg(os.getpgid(self.bag_process.pid), signal.SIGINT)
            self.bag_process.wait(timeout=10.0)
        except Exception:
            try:
                os.killpg(os.getpgid(self.bag_process.pid), signal.SIGTERM)
            except Exception:
                pass
        finally:
            self._publish_event('bag_recording_stopped', {})
            self.bag_process = None

    # ------------------------------------------------------------------
    # Subscriptions for safety / collision milestone events
    # ------------------------------------------------------------------

    def _collision_monitor_cb(self, msg: CollisionMonitorState) -> None:
        state = (int(msg.action_type), str(msg.polygon_name))
        if state == self.last_collision_state:
            return

        self.last_collision_state = state
        self._publish_event('collision_monitor_state', {
            'action_type': int(msg.action_type),
            'action_name': ACTION_TYPE_NAMES.get(int(msg.action_type), 'UNKNOWN'),
            'polygon_name': str(msg.polygon_name),
        })

    def _cmd_smoothed_cb(self, msg: Twist) -> None:
        self.last_cmd_smoothed = msg
        self._update_safety_stop_state()

    def _cmd_safe_cb(self, msg: Twist) -> None:
        self.last_cmd_safe = msg
        self._update_safety_stop_state()

    def _update_safety_stop_state(self) -> None:
        smoothed_mag = self._twist_mag(self.last_cmd_smoothed)
        safe_mag = self._twist_mag(self.last_cmd_safe)

        now_active = smoothed_mag > 0.05 and safe_mag < 0.01
        if now_active == self.safety_stop_active:
            return

        self.safety_stop_active = now_active
        self._publish_event('safety_stop_event', {
            'active': now_active,
            'cmd_smoothed_mag': smoothed_mag,
            'cmd_safe_mag': safe_mag,
        })

    # ------------------------------------------------------------------
    # Small utilities
    # ------------------------------------------------------------------

    def _build_goal_pose(self) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.goal_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.goal_x
        msg.pose.position.y = self.goal_y
        msg.pose.position.z = 0.0

        half_yaw = 0.5 * self.goal_yaw
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)
        return msg

    def _next_trial_index(self) -> int:
        pattern = re.compile(
            rf'^{re.escape(self.trial_prefix)}_{re.escape(self.environment_id)}_'
            rf'{re.escape(self.config_id)}_{self.today_tag}_t(\d+)$'
        )
        max_idx = 0
        for child in self.bag_base_dir.iterdir():
            if not child.is_dir():
                continue
            match = pattern.match(child.name)
            if match:
                max_idx = max(max_idx, int(match.group(1)))
        return max_idx + 1

    @staticmethod
    def _parse_nav2_config_name(name: str) -> Tuple[str, str]:
        stem = Path(name).stem.lower()
        if '_' in stem:
            controller, planner = stem.split('_', 1)
            return controller, planner
        return stem, 'unknown'

    @staticmethod
    def _pose_dict(msg: PoseStamped) -> Dict[str, Any]:
        return {
            'frame_id': msg.header.frame_id,
            'x': float(msg.pose.position.x),
            'y': float(msg.pose.position.y),
            'z': float(msg.pose.position.z),
            'qx': float(msg.pose.orientation.x),
            'qy': float(msg.pose.orientation.y),
            'qz': float(msg.pose.orientation.z),
            'qw': float(msg.pose.orientation.w),
        }

    @staticmethod
    def _duration_to_sec(duration_msg) -> float:
        return float(duration_msg.sec) + float(duration_msg.nanosec) * 1e-9

    @staticmethod
    def _twist_mag(msg: Twist) -> float:
        return math.sqrt(
            msg.linear.x * msg.linear.x +
            msg.linear.y * msg.linear.y +
            msg.angular.z * msg.angular.z
        )

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

    def destroy_node(self) -> bool:
        self._stop_bag_recording()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ResearchTrialRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        exit_code = getattr(node, 'pending_exit_code', 0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()

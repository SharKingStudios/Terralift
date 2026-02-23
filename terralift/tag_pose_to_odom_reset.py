#!/usr/bin/env python3
import math
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer, TransformListener, TransformException

from apriltag_msgs.msg import AprilTagDetectionArray


def _quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz

    return np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),       2*(xz + wy)],
        [2*(xy + wz),         1 - 2*(xx + zz),   2*(yz - wx)],
        [2*(xz - wy),         2*(yz + wx),       1 - 2*(xx + yy)],
    ], dtype=float)


def _yaw_to_rot(yaw: float) -> np.ndarray:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=float)


def _rot_to_quat(R: np.ndarray):
    tr = float(np.trace(R))
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S

    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx/n, qy/n, qz/n, qw/n)


def _tf_to_T(tf_msg) -> np.ndarray:
    t = tf_msg.translation
    r = tf_msg.rotation
    R = _quat_to_rot(r.x, r.y, r.z, r.w)
    T = np.eye(4, dtype=float)
    T[0:3, 0:3] = R
    T[0:3, 3] = [t.x, t.y, t.z]
    return T


def _T_to_pose(T: np.ndarray):
    x, y, z = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])
    qx, qy, qz, qw = _rot_to_quat(T[0:3, 0:3])
    return x, y, z, qx, qy, qz, qw


def _inv(T: np.ndarray) -> np.ndarray:
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    Ti = np.eye(4, dtype=float)
    Ti[0:3, 0:3] = R.T
    Ti[0:3, 3] = -R.T @ t
    return Ti


class TagPoseToOdomReset(Node):
    def __init__(self):
        super().__init__('tag_pose_to_odom_reset')

        self.declare_parameter('tag_map_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('tag_frame_prefix', 'tag_')
        self.declare_parameter('detections_topic', '/apriltag/detections')
        self.declare_parameter('reset_topic', '/tag_reset_pose')
        self.declare_parameter('min_decision_margin', 5.0)
        self.declare_parameter('min_interval_sec', 0.5)
        self.declare_parameter('max_tf_age_sec', 0.25)
        self.declare_parameter('tf_timeout_sec', 0.2)

        self.tag_map_file = str(self.get_parameter('tag_map_file').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.tag_frame_prefix = str(self.get_parameter('tag_frame_prefix').value)
        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.reset_topic = str(self.get_parameter('reset_topic').value)

        self.min_decision_margin = float(self.get_parameter('min_decision_margin').value)
        self.min_interval_sec = float(self.get_parameter('min_interval_sec').value)
        self.max_tf_age_sec = float(self.get_parameter('max_tf_age_sec').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)

        if not self.tag_map_file:
            raise RuntimeError("tag_map_file parameter is empty")

        self.tag_map = self._load_tag_map(self.tag_map_file)
        self.get_logger().info(f"Loaded {len(self.tag_map)} tag(s) from tag_map.yaml")

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.pub_reset = self.create_publisher(PoseStamped, self.reset_topic, 10)
        self.sub = self.create_subscription(AprilTagDetectionArray, self.detections_topic, self._cb, 10)

        self._last_pub_time = self.get_clock().now() - Duration(seconds=999.0)

        self.get_logger().info(
            f"Tag snapper active. tag_map_file={self.tag_map_file} "
            f"map={self.map_frame} odom={self.odom_frame} base={self.base_frame} "
            f"cam={self.camera_frame} reset={self.reset_topic}"
        )

    def _load_tag_map(self, path: str):
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}

        tag_map = {}

        # YOUR format:
        # tags:
        #   1: {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}
        if isinstance(data, dict) and 'tags' in data and isinstance(data['tags'], dict):
            for k, v in data['tags'].items():
                try:
                    tid = int(k)
                except Exception:
                    continue
                if not isinstance(v, dict):
                    continue
                x = float(v.get('x', 0.0))
                y = float(v.get('y', 0.0))
                z = float(v.get('z', 0.0))
                yaw = float(v.get('yaw', 0.0))

                T = np.eye(4, dtype=float)
                T[0:3, 0:3] = _yaw_to_rot(yaw)
                T[0:3, 3] = [x, y, z]
                tag_map[tid] = T
            return tag_map

        # Fallback: allow older formats if you ever switch
        # tags: [ {id, pose}, ... ]
        if isinstance(data, dict) and 'tags' in data and isinstance(data['tags'], list):
            for entry in data['tags']:
                try:
                    tid = int(entry['id'])
                    pose = entry.get('pose', {})
                    # try quaternion pose dict
                    p = pose.get('position', {})
                    o = pose.get('orientation', {})
                    x = float(p.get('x', 0.0))
                    y = float(p.get('y', 0.0))
                    z = float(p.get('z', 0.0))
                    qx = float(o.get('x', 0.0))
                    qy = float(o.get('y', 0.0))
                    qz = float(o.get('z', 0.0))
                    qw = float(o.get('w', 1.0))
                    T = np.eye(4, dtype=float)
                    T[0:3, 0:3] = _quat_to_rot(qx, qy, qz, qw)
                    T[0:3, 3] = [x, y, z]
                    tag_map[tid] = T
                except Exception as ex:
                    self.get_logger().warn(f"Skipping bad tag_map entry {entry}: {ex}")
            return tag_map

        # keys like "tag_1"
        if isinstance(data, dict):
            for k, v in data.items():
                if not isinstance(v, dict):
                    continue
                tid = None
                if isinstance(k, int):
                    tid = int(k)
                elif isinstance(k, str):
                    s = k.strip()
                    if s.startswith(self.tag_frame_prefix):
                        s = s[len(self.tag_frame_prefix):]
                    if s.isdigit():
                        tid = int(s)
                if tid is None:
                    continue

                # support yaw-only here too
                x = float(v.get('x', 0.0))
                y = float(v.get('y', 0.0))
                z = float(v.get('z', 0.0))
                if 'yaw' in v:
                    yaw = float(v.get('yaw', 0.0))
                    R = _yaw_to_rot(yaw)
                else:
                    qx = float(v.get('qx', 0.0))
                    qy = float(v.get('qy', 0.0))
                    qz = float(v.get('qz', 0.0))
                    qw = float(v.get('qw', 1.0))
                    R = _quat_to_rot(qx, qy, qz, qw)

                T = np.eye(4, dtype=float)
                T[0:3, 0:3] = R
                T[0:3, 3] = [x, y, z]
                tag_map[tid] = T

        return tag_map

    def _cb(self, msg: AprilTagDetectionArray):
        if not msg.detections:
            return

        now = self.get_clock().now()
        if (now - self._last_pub_time).nanoseconds * 1e-9 < self.min_interval_sec:
            return

        # Pick best detection by decision margin if present
        best = None
        best_margin = -1e9
        for d in msg.detections:
            margin = None
            if hasattr(d, 'decision_margin'):
                margin = float(d.decision_margin)
            elif hasattr(d, 'pose') and hasattr(d.pose, 'decision_margin'):
                margin = float(d.pose.decision_margin)
            if margin is None:
                margin = 999.0
            if margin > best_margin:
                best_margin = margin
                best = d

        if best is None:
            return
        if best_margin < self.min_decision_margin:
            return

        # Extract ID
        tid = None
        if hasattr(best, 'id') and isinstance(best.id, (list, tuple)) and len(best.id) > 0:
            tid = int(best.id[0])
        else:
            try:
                tid = int(best.id)
            except Exception:
                tid = None

        if tid is None:
            self.get_logger().warn("Detection had no usable tag id")
            return

        if tid not in self.tag_map:
            self.get_logger().warn(f"Tag {tid} not in tag_map; add it to {self.tag_map_file}")
            return

        tag_frame = f"{self.tag_frame_prefix}{tid}"
        stamp = Time.from_msg(msg.header.stamp)
        tf_timeout = Duration(seconds=self.tf_timeout_sec)

        tf_cam_tag = self._lookup(self.camera_frame, tag_frame, stamp, tf_timeout)
        if tf_cam_tag is None:
            return

        tf_base_cam = self._lookup(self.base_frame, self.camera_frame, stamp, tf_timeout)
        if tf_base_cam is None:
            return

        tf_map_odom = self._lookup(self.map_frame, self.odom_frame, Time(), tf_timeout)
        if tf_map_odom is None:
            self.get_logger().warn("No map->odom TF available yet; cannot snap")
            return

        tf_time = Time.from_msg(tf_cam_tag.header.stamp)
        age_sec = (now - tf_time).nanoseconds * 1e-9
        if age_sec > self.max_tf_age_sec:
            self.get_logger().warn(f"Tag TF stale ({age_sec:.3f}s); skipping snap")
            return

        T_cam_tag = _tf_to_T(tf_cam_tag.transform)
        T_base_cam = _tf_to_T(tf_base_cam.transform)
        T_map_odom = _tf_to_T(tf_map_odom.transform)
        T_map_tag = self.tag_map[tid]

        # map->cam = map->tag * inv(cam->tag)
        T_map_cam = T_map_tag @ _inv(T_cam_tag)

        # cam->base = inv(base->cam)
        T_cam_base = _inv(T_base_cam)

        # map->base
        T_map_base = T_map_cam @ T_cam_base

        # odom->base = inv(map->odom) * map->base
        T_odom_base = _inv(T_map_odom) @ T_map_base

        x, y, z, qx, qy, qz, qw = _T_to_pose(T_odom_base)

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.pose.position.x = float(x)
        out.pose.position.y = float(y)
        out.pose.position.z = float(z)
        out.pose.orientation.x = float(qx)
        out.pose.orientation.y = float(qy)
        out.pose.orientation.z = float(qz)
        out.pose.orientation.w = float(qw)

        self.pub_reset.publish(out)
        self._last_pub_time = now

        self.get_logger().info(
            f"SNAP tag={tid} margin={best_margin:.2f} -> odom pose x={x:.3f} y={y:.3f}"
        )

    def _lookup(self, target_frame: str, source_frame: str, when: Time, timeout: Duration):
        try:
            return self.tf_buffer.lookup_transform(target_frame, source_frame, when, timeout=timeout)
        except TransformException:
            try:
                return self.tf_buffer.lookup_transform(target_frame, source_frame, Time(), timeout=timeout)
            except TransformException as ex:
                self.get_logger().warn(f"TF lookup failed: {target_frame} <- {source_frame}: {ex}")
                return None


def main(args=None):
    rclpy.init(args=args)
    node = TagPoseToOdomReset()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped  # RESET HOOK
from tf2_ros import TransformBroadcaster

def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))

class OpenLoopOdom(Node):
    def __init__(self):
        super().__init__('open_loop_odom')

        self.imu_topic   = self.declare_parameter('imu_topic', '/imu/data').value
        self.odom_topic  = self.declare_parameter('odom_topic', '/odom').value
        self.odom_frame  = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame  = self.declare_parameter('base_frame', 'base_link').value
        self.rate_hz     = float(self.declare_parameter('rate_hz', 50.0).value)
        self.publish_tf  = bool(self.declare_parameter('publish_tf', True).value)

        # RESET HOOK: if non-empty, we will snap our internal (x,y,yaw) to this pose
        self.reset_pose_topic = self.declare_parameter('reset_pose_topic', '').value

        self.stationary_accel_thresh = float(self.declare_parameter('stationary_accel_thresh', 0.15).value)
        self.stationary_gyro_thresh  = float(self.declare_parameter('stationary_gyro_thresh', 0.10).value)
        self.stationary_hold_time    = float(self.declare_parameter('stationary_hold_time', 0.20).value)
        self.bias_learn_rate         = float(self.declare_parameter('bias_learn_rate', 0.6).value)
        self.zupt_vel_damp           = float(self.declare_parameter('zupt_vel_damp', 6.0).value)
        self.vel_decay               = float(self.declare_parameter('vel_decay', 0.15).value)
        self.max_speed               = float(self.declare_parameter('max_speed', 2.5).value)
        self.require_imu_yaw         = bool(self.declare_parameter('require_imu_yaw', True).value)

        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.imu_cb, 50)
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # RESET HOOK subscription
        if self.reset_pose_topic:
            self.sub_reset = self.create_subscription(PoseStamped, self.reset_pose_topic, self.reset_pose_cb, 10)
            self.get_logger().info(f"Reset pose enabled: topic={self.reset_pose_topic} frame={self.odom_frame}")

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.bias_ax = 0.0
        self.bias_ay = 0.0
        self.bias_gz = 0.0

        self.imu_yaw_available = False
        self.last_imu = None
        self.last_time = self.get_clock().now()

        self.stationary_timer = 0.0
        self.is_stationary = False

        self.timer = self.create_timer(1.0 / self.rate_hz, self.update)

    def reset_pose_cb(self, msg: PoseStamped):
        # Only accept snaps in our odom frame (keep it simple + safe)
        if msg.header.frame_id and msg.header.frame_id != self.odom_frame:
            self.get_logger().warn(
                f"Ignoring reset_pose in frame '{msg.header.frame_id}' (expected '{self.odom_frame}')"
            )
            return

        self.x = float(msg.pose.position.x)
        self.y = float(msg.pose.position.y)
        self.yaw = yaw_from_quat(msg.pose.orientation)

        # reset velocities so we don't immediately “kick” after a snap
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # prevent a huge dt integration on the next tick
        self.last_time = self.get_clock().now()

        self.get_logger().info(f"open_loop_odom SNAP -> x={self.x:.3f} y={self.y:.3f} yaw={self.yaw:.3f}")

    def imu_cb(self, msg: Imu):
        self.last_imu = msg
        if self.require_imu_yaw:
            self.imu_yaw_available = True

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now

        if self.last_imu is None:
            return

        ax = self.last_imu.linear_acceleration.x
        ay = self.last_imu.linear_acceleration.y
        gz = self.last_imu.angular_velocity.z

        # stationary detection
        accel_mag = math.sqrt(ax*ax + ay*ay)
        gyro_mag = abs(gz)

        stationary_now = (accel_mag < self.stationary_accel_thresh) and (gyro_mag < self.stationary_gyro_thresh)
        if stationary_now:
            self.stationary_timer += dt
        else:
            self.stationary_timer = 0.0

        self.is_stationary = self.stationary_timer >= self.stationary_hold_time

        # bias learning when stationary
        if self.is_stationary:
            self.bias_ax = (1.0 - self.bias_learn_rate) * self.bias_ax + self.bias_learn_rate * ax
            self.bias_ay = (1.0 - self.bias_learn_rate) * self.bias_ay + self.bias_learn_rate * ay
            self.bias_gz = (1.0 - self.bias_learn_rate) * self.bias_gz + self.bias_learn_rate * gz

        ax_u = ax - self.bias_ax
        ay_u = ay - self.bias_ay
        gz_u = gz - self.bias_gz

        if self.require_imu_yaw and not self.imu_yaw_available:
            gz_u = 0.0

        self.yaw += gz_u * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)

        ax_w = cy*ax_u - sy*ay_u
        ay_w = sy*ax_u + cy*ay_u

        self.vx += ax_w * dt
        self.vy += ay_w * dt

        speed = math.sqrt(self.vx*self.vx + self.vy*self.vy)
        if speed > self.max_speed:
            scale = self.max_speed / speed
            self.vx *= scale
            self.vy *= scale

        if self.is_stationary:
            self.vx *= math.exp(-self.zupt_vel_damp * dt)
            self.vy *= math.exp(-self.zupt_vel_damp * dt)
        else:
            self.vx *= math.exp(-self.vel_decay * dt)
            self.vy *= math.exp(-self.vel_decay * dt)

        self.x += self.vx * dt
        self.y += self.vy * dt

        self.publish(now)

    def publish(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = quat_from_yaw(self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = float(self.vx)
        odom.twist.twist.linear.y = float(self.vy)
        odom.twist.twist.angular.z = 0.0

        self.pub_odom.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = OpenLoopOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

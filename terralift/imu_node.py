#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

SIM_DEFAULT = False
IMPORT_ERROR = ""

try:
    import board
    import busio
    import adafruit_bno055
except Exception as e:
    SIM_DEFAULT = True
    IMPORT_ERROR = repr(e)
    board = None
    busio = None
    adafruit_bno055 = None


class BNO055ImuNode(Node):
    def __init__(self):
        super().__init__('imu_bno055')

        # ---------------- Parameters ----------------
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('simulate', SIM_DEFAULT)

        self.frame_id = self.get_parameter('frame_id').value
        self.rate_hz = float(self.get_parameter('publish_hz').value)
        self.simulate = bool(self.get_parameter('simulate').value)

        self.pub = self.create_publisher(Imu, 'imu/data', 10)

        import sys
        self.get_logger().info(f"Python executable: {sys.executable}")
        self.get_logger().info(f"SIM_DEFAULT={SIM_DEFAULT} simulate_param={self.simulate}")
        if SIM_DEFAULT:
            self.get_logger().error(f"IMU imports failed: {IMPORT_ERROR}")

        if self.simulate or SIM_DEFAULT:
            self.get_logger().warn("IMU SIMULATION MODE (no I2C)")
            self.bno = None
        else:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = adafruit_bno055.BNO055_I2C(i2c)

            try:
                self.bno.mode = adafruit_bno055.NDOF_MODE
            except Exception as e:
                self.get_logger().warn(f"Could not set BNO055 mode: {e}")

            self.get_logger().info("BNO055 IMU initialized over I2C (NDOF)")

        self._last_debug = 0.0

        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self.publish_imu)

    # ------------------------------------------------

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Reasonable default covariances (diagonal)
        msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]

        msg.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]

        msg.orientation_covariance = [
            0.05, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.05
        ]

        if self.simulate or self.bno is None:
            msg.orientation.w = 1.0
            self.pub.publish(msg)
            return

        # Quaternion: Adafruit returns (w, x, y, z)
        q = getattr(self.bno, "quaternion", None)
        if q and len(q) == 4 and all(v is not None for v in q):
            msg.orientation.w = float(q[0])
            msg.orientation.x = float(q[1])
            msg.orientation.y = float(q[2])
            msg.orientation.z = float(q[3])
        else:
            msg.orientation_covariance[0] = -1.0

        # Gyro already in rad/s
        gyro = getattr(self.bno, "gyro", None)
        if gyro and all(v is not None for v in gyro):
            msg.angular_velocity.x = float(gyro[0])
            msg.angular_velocity.y = float(gyro[1])
            msg.angular_velocity.z = float(gyro[2])

        # Acceleration m/s^2
        accel = getattr(self.bno, "acceleration", None)
        if accel and all(v is not None for v in accel):
            msg.linear_acceleration.x = float(accel[0])
            msg.linear_acceleration.y = float(accel[1])
            msg.linear_acceleration.z = float(accel[2])

        # Debug once per second
        # now = self.get_clock().now().nanoseconds * 1e-9
        # if now - self._last_debug > 1.0:
        #     self._last_debug = now
        #     cal = getattr(self.bno, "calibration_status", None)
        #     self.get_logger().info(
        #         f"quat={q} gyro={gyro} accel={accel} cal={cal}"
        #     )

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BNO055ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

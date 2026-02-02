#!/usr/bin/env python3
import math
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

        self.frame_id  = self.get_parameter('frame_id').value
        self.rate_hz   = self.get_parameter('publish_hz').value
        self.simulate  = self.get_parameter('simulate').value

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
            self.get_logger().info("BNO055 IMU initialized over I2C")

        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self.publish_imu)

    # ------------------------------------------------

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        if self.simulate or board is None:
            # Flat, stationary IMU
            msg.orientation.w = 1.0
            self.pub.publish(msg)
            return

        # Orientation (quaternion)
        q = self.bno.quaternion
        if q:
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]

        # Angular velocity (deg/s → rad/s)
        gyro = self.bno.gyro
        if gyro:
            msg.angular_velocity.x = math.radians(gyro[0])
            msg.angular_velocity.y = math.radians(gyro[1])
            msg.angular_velocity.z = math.radians(gyro[2])

        # Linear acceleration (m/s^2)
        accel = self.bno.accelerometer
        if accel:
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]

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

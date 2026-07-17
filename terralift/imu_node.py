#!/usr/bin/env python3
import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

SIM_DEFAULT = False
IMPORT_ERROR = ""

DEBUG_QUAT = False
DEBUG_LOG_PERIOD_S = 0.5

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


def _deg2rad(x: float) -> float:
    return x * math.pi / 180.0


def _quat_bno_to_ros(q_raw: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    """
    Adafruit CircuitPython BNO055 returns quaternion in Bosch register order:
      (w, x, y, z)

    ROS expects:
      (x, y, z, w)

    Return normalized ROS-order quaternion.
    """
    w, x, y, z = [float(v) for v in q_raw]

    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 1e-9:
        return (0.0, 0.0, 0.0, 1.0)

    inv = 1.0 / n
    return (x * inv, y * inv, z * inv, w * inv)


def _yaw_deg_from_ros_quat(q_ros: Tuple[float, float, float, float]) -> float:
    """
    q_ros is (x, y, z, w)
    Returns yaw in degrees in [-180, 180].
    """
    x, y, z, w = q_ros

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    yaw_deg = math.degrees(yaw)

    while yaw_deg > 180.0:
        yaw_deg -= 360.0
    while yaw_deg < -180.0:
        yaw_deg += 360.0

    return yaw_deg


class BNO055ImuNode(Node):
    def __init__(self):
        super().__init__('imu_bno055')

        # ---------------- Parameters ----------------
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('read_hz', 30.0)
        self.declare_parameter('simulate', SIM_DEFAULT)

        # Warn if we have not gotten a fresh sample in this many seconds
        self.declare_parameter('stale_warn_s', 0.5)

        # Stop publishing IMU messages entirely if older than this.
        # This allows robot_localization to timeout the IMU cleanly.
        self.declare_parameter('stale_stop_publish_s', 0.2)

        # Reinitialize the BNO after this many consecutive read failures
        self.declare_parameter('reset_after_consecutive_errors', 5)

        # Gyro units from library:
        #  - 'rad' for rad/s
        #  - 'deg' for deg/s
        self.declare_parameter('gyro_units', 'rad')

        # Covariance tuning:
        # Trust yaw and yaw rate strongly.
        # Do not trust roll/pitch or accel for this planar robot.
        self.declare_parameter('yaw_stddev_deg', 1.5)
        self.declare_parameter('yaw_rate_stddev', 0.05)
        self.declare_parameter('roll_pitch_covariance', 99999.0)
        self.declare_parameter('angular_velocity_xy_covariance', 99999.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.read_hz = float(self.get_parameter('read_hz').value)
        self.simulate = bool(self.get_parameter('simulate').value)
        self.stale_warn_s = float(self.get_parameter('stale_warn_s').value)
        self.stale_stop_publish_s = float(self.get_parameter('stale_stop_publish_s').value)
        self.reset_after_consecutive_errors = int(self.get_parameter('reset_after_consecutive_errors').value)
        self.gyro_units = str(self.get_parameter('gyro_units').value).strip().lower()
        self.yaw_stddev_deg = float(self.get_parameter('yaw_stddev_deg').value)
        self.yaw_rate_stddev = float(self.get_parameter('yaw_rate_stddev').value)
        self.roll_pitch_covariance = float(self.get_parameter('roll_pitch_covariance').value)
        self.angular_velocity_xy_covariance = float(self.get_parameter('angular_velocity_xy_covariance').value)

        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        import sys
        self.get_logger().info(f"Python executable: {sys.executable}")
        self.get_logger().info(f"SIM_DEFAULT={SIM_DEFAULT} simulate_param={self.simulate}")
        if SIM_DEFAULT:
            self.get_logger().error(f"IMU imports failed: {IMPORT_ERROR}")

        # ---------------- IMU init ----------------
        self._i2c = None
        self.bno = None
        self._consecutive_errors = 0

        if self.simulate or SIM_DEFAULT or board is None:
            self.get_logger().warn("IMU SIMULATION MODE. Publishing identity orientation.")
        else:
            self._init_sensor()

        # ---------------- Cached state ----------------
        self._lock = threading.Lock()
        self._last_sample_time = 0.0

        # Cached values
        self._q_ros: Optional[Tuple[float, float, float, float]] = None   # (x, y, z, w)
        self._q_raw_wxyz: Optional[Tuple[float, float, float, float]] = None
        self._gyro: Optional[Tuple[float, float, float]] = None           # rad/s
        self._calib: Optional[Tuple[int, int, int, int]] = None           # sys, gyro, accel, mag

        # ---------------- Threading ----------------
        self._stop = False
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        period = 1.0 / max(1.0, self.publish_hz)
        self._warn_ctr = 0
        self._warn_every = max(1, int(round(self.publish_hz)))
        self.timer = self.create_timer(period, self._publish_cached)

        if DEBUG_QUAT:
            self._debug_timer = self.create_timer(DEBUG_LOG_PERIOD_S, self._debug_log)

    # ------------------------------------------------

    def _init_sensor(self) -> bool:
        """Initialize or reinitialize the BNO055 over I2C."""
        try:
            self._i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = adafruit_bno055.BNO055_I2C(self._i2c)
            self._consecutive_errors = 0
            self.get_logger().info("BNO055 IMU initialized over I2C")
            return True
        except Exception as e:
            self.bno = None
            self._i2c = None
            self.get_logger().error(f"Failed to init BNO055 over I2C: {repr(e)}")
            return False

    def _reader_loop(self):
        """
        Continuously read the IMU in a background thread and cache results.

        Important:
        - Read quaternion and gyro only.
        - Do not read linear acceleration.
        - At a very slow Pi I2C bus, keeping bus load down matters.
        """
        dt = 1.0 / max(1.0, self.read_hz)

        while not self._stop:
            if self.simulate or self.bno is None:
                if self.simulate:
                    with self._lock:
                        self._q_ros = (0.0, 0.0, 0.0, 1.0)
                        self._q_raw_wxyz = (1.0, 0.0, 0.0, 0.0)
                        self._gyro = (0.0, 0.0, 0.0)
                        self._calib = (0, 0, 0, 0)
                        self._last_sample_time = time.time()
                else:
                    self._init_sensor()

                time.sleep(dt)
                continue

            try:
                q_ros_out = None
                q_raw_out = None
                gyro_out = None
                calib_out = None

                q_raw = self.bno.quaternion
                if q_raw and len(q_raw) == 4 and all(v is not None for v in q_raw):
                    q_raw_out = tuple(float(v) for v in q_raw)
                    q_ros_out = _quat_bno_to_ros(q_raw_out)

                gyro = getattr(self.bno, 'gyro', None)
                if gyro and len(gyro) == 3 and all(v is not None for v in gyro):
                    gx, gy, gz = float(gyro[0]), float(gyro[1]), float(gyro[2])
                    if self.gyro_units == 'deg':
                        gx, gy, gz = _deg2rad(gx), _deg2rad(gy), _deg2rad(gz)
                    gyro_out = (gx, gy, gz)

                calib = getattr(self.bno, 'calibration_status', None)
                if calib and len(calib) == 4 and all(v is not None for v in calib):
                    calib_out = tuple(int(v) for v in calib)

                if q_ros_out is not None or gyro_out is not None:
                    with self._lock:
                        if q_ros_out is not None:
                            self._q_ros = q_ros_out
                        if q_raw_out is not None:
                            self._q_raw_wxyz = q_raw_out
                        if gyro_out is not None:
                            self._gyro = gyro_out
                        if calib_out is not None:
                            self._calib = calib_out
                        self._last_sample_time = time.time()
                    self._consecutive_errors = 0

            except Exception as e:
                self._consecutive_errors += 1
                self.get_logger().warn(f"IMU read error: {repr(e)}")

                if self._consecutive_errors >= self.reset_after_consecutive_errors:
                    self.get_logger().warn(
                        "Too many consecutive IMU read errors. Reinitializing BNO055."
                    )
                    self._init_sensor()

            time.sleep(dt)

    def _publish_cached(self):
        """
        Publish cached IMU values at publish_hz.

        If the cached sample is stale, publish nothing.
        This lets robot_localization timeout the IMU cleanly.
        """
        now_s = time.time()
        with self._lock:
            age = now_s - self._last_sample_time if self._last_sample_time > 0.0 else 1e9
            q_ros = self._q_ros
            gyro = self._gyro

        self._warn_ctr += 1
        if self._warn_ctr % self._warn_every == 0 and age > self.stale_warn_s:
            self.get_logger().warn(
                f"/imu/data is STALE (no fresh IMU sample for {age:.2f}s). "
                f"Not publishing until a fresh sample arrives."
            )

        if age > self.stale_stop_publish_s:
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation: trust yaw strongly, do not trust roll/pitch for this robot
        if q_ros is not None:
            msg.orientation.x = q_ros[0]
            msg.orientation.y = q_ros[1]
            msg.orientation.z = q_ros[2]
            msg.orientation.w = q_ros[3]

            yaw_var = math.radians(self.yaw_stddev_deg) ** 2
            msg.orientation_covariance = [
                self.roll_pitch_covariance, 0.0, 0.0,
                0.0, self.roll_pitch_covariance, 0.0,
                0.0, 0.0, yaw_var
            ]
        else:
            msg.orientation.w = 1.0
            msg.orientation_covariance = [
                -1.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0
            ]

        # Angular velocity: trust z strongly, x/y not useful here
        if gyro is not None:
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]

            yaw_rate_var = self.yaw_rate_stddev ** 2
            msg.angular_velocity_covariance = [
                self.angular_velocity_xy_covariance, 0.0, 0.0,
                0.0, self.angular_velocity_xy_covariance, 0.0,
                0.0, 0.0, yaw_rate_var
            ]
        else:
            msg.angular_velocity_covariance = [
                -1.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0
            ]

        # Do not provide linear acceleration to downstream filters
        msg.linear_acceleration_covariance = [
            -1.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        ]

        self.pub.publish(msg)

    def _debug_log(self):
        now_s = time.time()
        with self._lock:
            age = now_s - self._last_sample_time if self._last_sample_time > 0.0 else 1e9
            q_raw = self._q_raw_wxyz
            q_ros = self._q_ros
            gyro = self._gyro
            calib = self._calib

        if q_ros is not None:
            yaw_deg = _yaw_deg_from_ros_quat(q_ros)
        else:
            yaw_deg = float('nan')

        gyro_z = gyro[2] if gyro is not None else float('nan')

        self.get_logger().info(
            "IMU DEBUG | "
            f"age={age:.3f}s | "
            f"raw_wxyz={q_raw} | "
            f"ros_xyzw={q_ros} | "
            f"yaw_deg={yaw_deg:.2f} | "
            f"gyro_z={gyro_z:.4f} | "
            f"calib={calib}"
        )

    def destroy_node(self):
        self._stop = True
        if hasattr(self, '_reader_thread') and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BNO055ImuNode()
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

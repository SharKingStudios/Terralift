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


def _quat_guess_and_normalize(q_raw: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    """
    Adafruit BNO055 libs have historically returned quaternion in either:
      - (w, x, y, z)
      - (x, y, z, w)

    Guess ordering, then normalize. Returns (x, y, z, w).
    """
    a, b, c, d = [float(v) for v in q_raw]

    if abs(a) >= abs(d):
        # Assume (w, x, y, z)
        w, x, y, z = a, b, c, d
    else:
        # Assume (x, y, z, w)
        x, y, z, w = a, b, c, d

    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 1e-9:
        return (0.0, 0.0, 0.0, 1.0)

    inv = 1.0 / n
    return (x * inv, y * inv, z * inv, w * inv)


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
        # This is important so robot_localization can timeout the IMU
        # instead of being fed stale data.
        self.declare_parameter('stale_stop_publish_s', 0.2)

        # Reinitialize the BNO after this many consecutive read failures
        self.declare_parameter('reset_after_consecutive_errors', 5)

        # Gyro units from the library:
        #  - 'rad' for rad/s
        #  - 'deg' for deg/s
        self.declare_parameter('gyro_units', 'rad')

        # Covariance tuning.
        # We trust yaw and yaw rate strongly.
        # We do not trust roll/pitch or accel for this planar robot.
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
        self._q: Optional[Tuple[float, float, float, float]] = None  # (x, y, z, w)
        self._gyro: Optional[Tuple[float, float, float]] = None      # rad/s

        # ---------------- Threading ----------------
        self._stop = False
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        period = 1.0 / max(1.0, self.publish_hz)
        self._warn_ctr = 0
        self._warn_every = max(1, int(round(self.publish_hz)))
        self.timer = self.create_timer(period, self._publish_cached)

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
        - Only read quaternion and gyro.
        - Do not read linear acceleration here.
        - At 5 kHz Pi I2C, keeping bus load low matters.
        """
        dt = 1.0 / max(1.0, self.read_hz)

        while not self._stop:
            if self.simulate or self.bno is None:
                if self.simulate:
                    with self._lock:
                        self._q = (0.0, 0.0, 0.0, 1.0)
                        self._gyro = (0.0, 0.0, 0.0)
                        self._last_sample_time = time.time()
                else:
                    # Try to recover the sensor periodically
                    self._init_sensor()

                time.sleep(dt)
                continue

            try:
                q_out = None
                gyro_out = None

                q_raw = self.bno.quaternion
                if q_raw and len(q_raw) == 4 and all(v is not None for v in q_raw):
                    q_out = _quat_guess_and_normalize(q_raw)

                gyro = getattr(self.bno, 'gyro', None)
                if gyro and len(gyro) == 3 and all(v is not None for v in gyro):
                    gx, gy, gz = float(gyro[0]), float(gyro[1]), float(gyro[2])
                    if self.gyro_units == 'deg':
                        gx, gy, gz = _deg2rad(gx), _deg2rad(gy), _deg2rad(gz)
                    gyro_out = (gx, gy, gz)

                if q_out is not None or gyro_out is not None:
                    with self._lock:
                        if q_out is not None:
                            self._q = q_out
                        if gyro_out is not None:
                            self._gyro = gyro_out
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
            q = self._q
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

        # Orientation:
        # We trust yaw strongly, but do not trust roll/pitch for this robot.
        if q is not None:
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
            yaw_var = math.radians(self.yaw_stddev_deg) ** 2
            msg.orientation_covariance = [
                self.roll_pitch_covariance, 0.0, 0.0,
                0.0, self.roll_pitch_covariance, 0.0,
                0.0, 0.0, yaw_var
            ]
        else:
            msg.orientation.w = 1.0
            msg.orientation_covariance = [-1.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0]

        # Angular velocity:
        # Trust z heavily. x/y are not useful for this planar platform.
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
            msg.angular_velocity_covariance = [-1.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0]

        # Do not publish linear acceleration as a usable measurement.
        msg.linear_acceleration_covariance = [-1.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0]

        self.pub.publish(msg)

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

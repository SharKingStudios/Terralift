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

    We guess ordering by comparing magnitudes (w is often the largest near identity),
    then normalize to unit length. Returns (x, y, z, w).
    """
    a, b, c, d = [float(v) for v in q_raw]

    # Guess: if |a| looks like w (often ~1 near identity) and |d| does not, treat as (w,x,y,z)
    # Otherwise treat as (x,y,z,w).
    if abs(a) >= abs(d):
        # (w, x, y, z)
        w, x, y, z = a, b, c, d
    else:
        # (x, y, z, w)
        x, y, z, w = a, b, c, d

    # Normalize
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
        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('simulate', SIM_DEFAULT)

        # How often to READ the IMU (separate from publish rate)
        self.declare_parameter('read_hz', 100.0)

        # If we haven't gotten a fresh IMU sample in this many seconds, warn
        self.declare_parameter('stale_warn_s', 0.5)

        # Gyro units from the library:
        #  - 'rad' (recommended default for adafruit_bno055.gyro in most installs)
        #  - 'deg'
        self.declare_parameter('gyro_units', 'rad')

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.read_hz = float(self.get_parameter('read_hz').value)
        self.simulate = bool(self.get_parameter('simulate').value)
        self.stale_warn_s = float(self.get_parameter('stale_warn_s').value)
        self.gyro_units = str(self.get_parameter('gyro_units').value).strip().lower()

        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        import sys
        self.get_logger().info(f"Python executable: {sys.executable}")
        self.get_logger().info(f"SIM_DEFAULT={SIM_DEFAULT} simulate_param={self.simulate}")
        if SIM_DEFAULT:
            self.get_logger().error(f"IMU imports failed: {IMPORT_ERROR}")

        # ---------------- IMU init ----------------
        self.bno = None
        if self.simulate or SIM_DEFAULT or board is None:
            self.get_logger().warn("IMU SIMULATION MODE (no I2C) - will publish identity orientation.")
        else:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.bno = adafruit_bno055.BNO055_I2C(i2c)
                self.get_logger().info("BNO055 IMU initialized over I2C")
            except Exception as e:
                self.get_logger().error(f"Failed to init BNO055 over I2C, falling back to simulate: {repr(e)}")
                self.bno = None
                self.simulate = True

        # ---------------- Cached state (published) ----------------
        self._lock = threading.Lock()
        self._last_sample_time = 0.0

        # cached values
        self._q: Optional[Tuple[float, float, float, float]] = None  # (x,y,z,w)
        self._gyro: Optional[Tuple[float, float, float]] = None     # rad/s
        self._accel: Optional[Tuple[float, float, float]] = None    # m/s^2 (GRAVITY-COMPENSATED if available)

        # Thread control
        self._stop = False
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        # Publish timer (never blocks on I2C)
        period = 1.0 / max(1.0, self.publish_hz)
        self._warn_ctr = 0
        self._warn_every = int(max(1.0, self.publish_hz))  # ~1Hz
        self.timer = self.create_timer(period, self._publish_cached)

    # ------------------------------------------------

    def _reader_loop(self):
        """Continuously read IMU in a background thread and cache results."""
        dt = 1.0 / max(1.0, self.read_hz)
        while not self._stop:
            if self.simulate or self.bno is None:
                with self._lock:
                    self._q = (0.0, 0.0, 0.0, 1.0)
                    self._gyro = (0.0, 0.0, 0.0)
                    self._accel = (0.0, 0.0, 0.0)
                    self._last_sample_time = time.time()
                time.sleep(dt)
                continue

            try:
                # Quaternion: handle either (w,x,y,z) or (x,y,z,w)
                q_raw = self.bno.quaternion  # tuple of 4 floats or None
                q_out = None
                if q_raw and len(q_raw) == 4 and all(v is not None for v in q_raw):
                    q_out = _quat_guess_and_normalize(q_raw)

                # Gyro: most adafruit_bno055 installs expose gyro in rad/s.
                # Some forks/documentation mention deg/s. Make it configurable.
                gyro = getattr(self.bno, "gyro", None)
                gyro_out = None
                if gyro and len(gyro) == 3 and all(v is not None for v in gyro):
                    gx, gy, gz = float(gyro[0]), float(gyro[1]), float(gyro[2])
                    if self.gyro_units == 'deg':
                        gx, gy, gz = _deg2rad(gx), _deg2rad(gy), _deg2rad(gz)
                    gyro_out = (gx, gy, gz)

                # Linear acceleration:
                # Prefer gravity-compensated (BNO055 fused output)
                lin = getattr(self.bno, "linear_acceleration", None)  # gravity removed (best)
                grav = getattr(self.bno, "gravity", None)             # gravity vector
                accel = getattr(self.bno, "acceleration", None)       # raw accel (includes gravity)

                accel_out = None
                if lin and len(lin) == 3 and all(v is not None for v in lin):
                    accel_out = (float(lin[0]), float(lin[1]), float(lin[2]))
                elif accel and grav and len(accel) == 3 and len(grav) == 3 and all(v is not None for v in accel) and all(v is not None for v in grav):
                    accel_out = (float(accel[0] - grav[0]), float(accel[1] - grav[1]), float(accel[2] - grav[2]))
                elif accel and len(accel) == 3 and all(v is not None for v in accel):
                    # Last resort: raw accel (includes gravity) - not good for integrating planar velocity.
                    accel_out = (float(accel[0]), float(accel[1]), float(accel[2]))

                with self._lock:
                    if q_out is not None:
                        self._q = q_out
                    if gyro_out is not None:
                        self._gyro = gyro_out
                    if accel_out is not None:
                        self._accel = accel_out
                    self._last_sample_time = time.time()

            except Exception as e:
                self.get_logger().warn(f"IMU read error (publishing last-good): {repr(e)}")

            time.sleep(dt)

    def _publish_cached(self):
        """Publish cached IMU values at publish_hz (never blocks on I2C)."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        now_s = time.time()
        with self._lock:
            age = now_s - self._last_sample_time if self._last_sample_time > 0.0 else 1e9
            q = self._q
            gyro = self._gyro
            accel = self._accel

        # Always publish something
        if q is None:
            msg.orientation.w = 1.0
        else:
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]

        if gyro is not None:
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]

        if accel is not None:
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]

        self.pub.publish(msg)

        # Stale warning
        self._warn_ctr += 1
        if self._warn_ctr % self._warn_every == 0 and age > self.stale_warn_s:
            self.get_logger().warn(
                f"/imu/data is STALE (no fresh IMU sample for {age:.2f}s). "
                f"I2C read thread may be stuck; still publishing last-known/identity."
            )

    def destroy_node(self):
        self._stop = True
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()

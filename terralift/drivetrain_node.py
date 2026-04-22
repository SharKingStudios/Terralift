#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# GPIO handling
SIM_DEFAULT = False
try:
    import RPi.GPIO as GPIO
except Exception:
    SIM_DEFAULT = True
    GPIO = None


class MecanumDriveNode(Node):
    def __init__(self):
        super().__init__('mecanum_drive')

        # ---------------- Parameters ----------------
        self.declare_parameter('pwm_hz', 2000)
        self.declare_parameter('max_duty', 100.0)
        self.declare_parameter('simulate', SIM_DEFAULT)
        self.declare_parameter('cmd_topic', 'cmd_mecanum')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('update_hz', 50.0)
        self.declare_parameter('cmd_timeout_sec', 0.35)
        self.declare_parameter('odom_timeout_sec', 0.50)
        self.declare_parameter('assist_enabled', True)
        self.declare_parameter('assist_gain_per_sec', 0.8)
        self.declare_parameter('assist_decay_per_sec', 1.6)
        self.declare_parameter('assist_max', 0.35)
        self.declare_parameter('assist_deadband', 0.04)
        self.declare_parameter('cmd_deadband', 0.03)
        self.declare_parameter('max_vx_mps', 1.22)
        self.declare_parameter('max_vy_mps', 1.22)
        self.declare_parameter('max_wz_rps', 3.14)

        self.simulate = self.get_parameter('simulate').value
        self.pwm_hz = self.get_parameter('pwm_hz').value
        self.max_duty = self.get_parameter('max_duty').value
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.update_hz = float(self.get_parameter('update_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout_sec').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout_sec').value)
        self.assist_enabled = bool(self.get_parameter('assist_enabled').value)
        self.assist_gain = float(self.get_parameter('assist_gain_per_sec').value)
        self.assist_decay = float(self.get_parameter('assist_decay_per_sec').value)
        self.assist_max = float(self.get_parameter('assist_max').value)
        self.assist_deadband = float(self.get_parameter('assist_deadband').value)
        self.cmd_deadband = float(self.get_parameter('cmd_deadband').value)
        self.max_vx = float(self.get_parameter('max_vx_mps').value)
        self.max_vy = float(self.get_parameter('max_vy_mps').value)
        self.max_wz = float(self.get_parameter('max_wz_rps').value)

        # Wheel GPIO map: FL, FR, RL, RR
        self.wheels = {
            'fl': {'dir': 17, 'pwm': 27},
            'fr': {'dir': 22, 'pwm': 10},
            'rl': {'dir': 5, 'pwm': 13},
            'rr': {'dir': 9, 'pwm': 11},
        }

        self.declare_parameter('invert_fl', False)
        self.declare_parameter('invert_fr', True)
        self.declare_parameter('invert_rl', True)
        self.declare_parameter('invert_rr', False)

        self.invert = {
            'fl': -1.0 if self.get_parameter('invert_fl').value else 1.0,
            'fr': -1.0 if self.get_parameter('invert_fr').value else 1.0,
            'rl': -1.0 if self.get_parameter('invert_rl').value else 1.0,
            'rr': -1.0 if self.get_parameter('invert_rr').value else 1.0,
        }

        self.pwms = {}

        if self.simulate:
            self.get_logger().warn("SIMULATION MODE ENABLED (no GPIO)")
        else:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)

            for w in self.wheels.values():
                GPIO.setup(w['dir'], GPIO.OUT, initial=GPIO.LOW)
                GPIO.setup(w['pwm'], GPIO.OUT, initial=GPIO.LOW)
                pwm = GPIO.PWM(w['pwm'], self.pwm_hz)
                pwm.start(0.0)
                self.pwms[w['pwm']] = pwm

        self.target_cmd = Twist()
        self.last_cmd_time = None
        self.last_odom_time = None
        self.odom_feedback = {'x': 0.0, 'y': 0.0, 'wz': 0.0}
        self.assist = {'x': 0.0, 'y': 0.0, 'wz': 0.0}
        self.last_update_time = self.get_clock().now()

        self.sub = self.create_subscription(Twist, self.cmd_topic, self.cmd_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.timer = self.create_timer(1.0 / max(1.0, self.update_hz), self.control_loop)

        self.get_logger().info(
            f"Mecanum drive ready on {self.cmd_topic} with odom assist from {self.odom_topic} "
            f"(enabled={self.assist_enabled})"
        )

    # --------------------------------------------------

    def cmd_callback(self, msg: Twist):
        self.target_cmd.linear.x = self.clamp(float(msg.linear.x), -1.0, 1.0)
        self.target_cmd.linear.y = self.clamp(float(msg.linear.y), -1.0, 1.0)
        self.target_cmd.angular.z = self.clamp(float(msg.angular.z), -1.0, 1.0)
        self.last_cmd_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        self.odom_feedback['x'] = self._normalize(msg.twist.twist.linear.x, self.max_vx)
        self.odom_feedback['y'] = self._normalize(msg.twist.twist.linear.y, self.max_vy)
        self.odom_feedback['wz'] = self._normalize(msg.twist.twist.angular.z, self.max_wz)
        self.last_odom_time = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_update_time = now

        desired = self._active_command(now)
        if self.assist_enabled and self._is_fresh(self.last_odom_time, self.odom_timeout):
            vx = self._assist_axis('x', desired.linear.x, self.odom_feedback['x'], dt)
            vy = self._assist_axis('y', desired.linear.y, self.odom_feedback['y'], dt)
            wz = self._assist_axis('wz', desired.angular.z, self.odom_feedback['wz'], dt)
        else:
            self._decay_assist(dt)
            vx = float(desired.linear.x)
            vy = float(desired.linear.y)
            wz = float(desired.angular.z)

        self.apply_drive_command(vx, vy, wz)

    def apply_drive_command(self, vx: float, vy: float, wz: float):
        speeds = {
            'fl': vx - vy - wz,
            'fr': vx + vy + wz,
            'rl': vx + vy - wz,
            'rr': vx - vy + wz,
        }

        for k in speeds:
            speeds[k] *= self.invert[k]

        # Normalize wheel speeds
        max_mag = max(abs(v) for v in speeds.values())
        if max_mag > 1.0:
            for k in speeds:
                speeds[k] /= max_mag

        for name, speed in speeds.items():
            self.set_wheel(name, speed)

    def _active_command(self, now):
        cmd = Twist()
        if self._is_fresh(self.last_cmd_time, self.cmd_timeout, now):
            cmd.linear.x = float(self.target_cmd.linear.x)
            cmd.linear.y = float(self.target_cmd.linear.y)
            cmd.angular.z = float(self.target_cmd.angular.z)
        return cmd

    def _assist_axis(self, axis: str, desired: float, actual: float, dt: float) -> float:
        assist = float(self.assist[axis])
        desired = 0.0 if abs(desired) < self.cmd_deadband else float(desired)

        if desired == 0.0 or desired * assist < 0.0:
            assist = self._decay_toward_zero(assist, self.assist_decay * dt)
        else:
            error = desired - actual
            if abs(error) <= self.assist_deadband:
                assist = self._decay_toward_zero(assist, 0.5 * self.assist_decay * dt)
            else:
                assist += self.assist_gain * error * dt
                assist = self.clamp(assist, -self.assist_max, self.assist_max)

        self.assist[axis] = assist
        return self.clamp(desired + assist, -1.0, 1.0)

    def _decay_assist(self, dt: float):
        for axis in self.assist:
            self.assist[axis] = self._decay_toward_zero(self.assist[axis], self.assist_decay * dt)

    def _decay_toward_zero(self, value: float, amount: float) -> float:
        if value > 0.0:
            return max(0.0, value - amount)
        return min(0.0, value + amount)

    def _normalize(self, value: float, scale: float) -> float:
        if abs(scale) < 1e-6:
            return 0.0
        return self.clamp(float(value) / float(scale), -2.0, 2.0)

    def _is_fresh(self, stamp, timeout_sec: float, now=None) -> bool:
        if stamp is None:
            return False
        ref = now if now is not None else self.get_clock().now()
        age = (ref - stamp).nanoseconds * 1e-9
        return age <= timeout_sec

    @staticmethod
    def clamp(value: float, lo: float, hi: float) -> float:
        return lo if value < lo else hi if value > hi else value

    # --------------------------------------------------

    def set_wheel(self, wheel, value):
        direction = value >= 0.0
        duty = min(abs(value) * self.max_duty, self.max_duty)

        if self.simulate:
            self.get_logger().info(
                f"[SIM] {wheel.upper()} DIR={'FWD' if direction else 'REV'} DUTY={duty:.1f}%"
            )
            return

        w = self.wheels[wheel]
        GPIO.output(w['dir'], GPIO.HIGH if direction else GPIO.LOW)
        time.sleep(0.002)
        self.pwms[w['pwm']].ChangeDutyCycle(duty)

    # --------------------------------------------------

    def destroy_node(self):
        if not self.simulate:
            for pwm in self.pwms.values():
                pwm.ChangeDutyCycle(0.0)
                pwm.stop()
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MecanumDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

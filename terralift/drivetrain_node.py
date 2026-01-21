#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

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
        self.declare_parameter('max_duty', 70.0)
        self.declare_parameter('simulate', SIM_DEFAULT)

        self.simulate = self.get_parameter('simulate').value
        self.pwm_hz   = self.get_parameter('pwm_hz').value
        self.max_duty = self.get_parameter('max_duty').value

        # Wheel GPIO map: FL, FR, RL, RR
        self.wheels = {
            'fl': {'dir': 17, 'pwm': 18},
            'fr': {'dir': 27, 'pwm': 22},
            'rl': {'dir': 23, 'pwm': 24},
            'rr': {'dir': 25, 'pwm': 5},
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

        # Subscribe to mecanum command
        self.sub = self.create_subscription(
            Twist,
            'cmd_mecanum',
            self.cmd_callback,
            10
        )

        self.get_logger().info("Mecanum drive ready on /cmd_mecanum")

    # --------------------------------------------------

    def cmd_callback(self, msg: Twist):
        vx = msg.linear.x    # forward/back
        vy = msg.linear.y    # left/right
        wz = msg.angular.z   # rotation

        # Mecanum inverse kinematics
        speeds = {
            'fl': vx - vy - wz,
            'fr': vx + vy + wz,
            'rl': vx + vy - wz,
            'rr': vx - vy + wz,
        }

        # Normalize wheel speeds
        max_mag = max(abs(v) for v in speeds.values())
        if max_mag > 1.0:
            for k in speeds:
                speeds[k] /= max_mag

        for name, speed in speeds.items():
            self.set_wheel(name, speed)

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

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# GPIO handling
SIM_DEFAULT = False
try:
    import RPi.GPIO as GPIO
except Exception:
    SIM_DEFAULT = True
    GPIO = None


class LiftArmNode(Node):
    def __init__(self):
        super().__init__('lift_arm')

        # ---------------- Parameters ----------------
        self.declare_parameter('pwm_pin', 12)            # GPIO12 (Pin 32)
        self.declare_parameter('pwm_hz', 50)             # Servo PWM
        self.declare_parameter('min_us', 1200)           # HARD LIMIT
        self.declare_parameter('max_us', 1800)           # HARD LIMIT
        self.declare_parameter('max_deg', 180.0)         # Editable
        self.declare_parameter('speed_us_per_tick', 20)  # ESP32-equivalent
        self.declare_parameter('update_hz', 100)
        self.declare_parameter('simulate', SIM_DEFAULT)

        self.pwm_pin = self.get_parameter('pwm_pin').value
        self.pwm_hz  = self.get_parameter('pwm_hz').value
        self.min_us  = self.get_parameter('min_us').value
        self.max_us  = self.get_parameter('max_us').value
        self.max_deg = self.get_parameter('max_deg').value
        self.speed   = self.get_parameter('speed_us_per_tick').value
        self.simulate= self.get_parameter('simulate').value

        self.current_us = (self.min_us + self.max_us) // 2
        self.target_us  = self.current_us

        if self.simulate:
            self.get_logger().warn("SIMULATION MODE (no GPIO)")
        else:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.pwm_pin, GPIO.OUT)

            self.pwm = GPIO.PWM(self.pwm_pin, self.pwm_hz)
            self.pwm.start(self._us_to_duty(self.current_us))

        # Subscriber
        self.sub = self.create_subscription(
            Float32,
            'lift_arm/command',
            self.command_cb,
            10
        )

        # Timer (speed-limited motion)
        period = 1.0 / self.get_parameter('update_hz').value
        self.timer = self.create_timer(period, self.update_servo)

        self.get_logger().info(
            f"Lift arm ready (limits {self.min_us}–{self.max_us} µs)"
        )

    # -------------------------------------------------------------

    def command_cb(self, msg: Float32):
        # Convert degrees → microseconds
        deg = max(0.0, min(self.max_deg, msg.data))

        span = self.max_us - self.min_us
        self.target_us = int(self.min_us + (deg / self.max_deg) * span)

        self.get_logger().info(
            f"Target: {deg:.1f}° → {self.target_us} µs"
        )

    # -------------------------------------------------------------

    def update_servo(self):
        delta = self.target_us - self.current_us

        if abs(delta) < self.speed:
            self.current_us = self.target_us
        else:
            self.current_us += self.speed if delta > 0 else -self.speed

        # HARD CLAMP (absolute safety)
        self.current_us = max(self.min_us, min(self.max_us, self.current_us))

        duty = self._us_to_duty(self.current_us)

        if self.simulate:
            self.get_logger().debug(f"[SIM] Servo {self.current_us} µs")
        else:
            self.pwm.ChangeDutyCycle(duty)

    # -------------------------------------------------------------

    def _us_to_duty(self, us):
        # Convert pulse width → duty cycle %
        period_us = 1_000_000 / self.pwm_hz
        return (us / period_us) * 100.0

    # -------------------------------------------------------------

    def destroy_node(self):
        if not self.simulate:
            self.pwm.ChangeDutyCycle(0.0)
            self.pwm.stop()
            GPIO.cleanup(self.pwm_pin)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiftArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

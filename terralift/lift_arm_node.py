#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

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
        # pwm_pin uses BCM numbering
        # BCM 19 = physical pin 35
        # BCM 12 = physical pin 32
        self.declare_parameter('pwm_pin', 19)
        self.declare_parameter('pwm_hz', 50)

        # Safety limits (microseconds)
        self.declare_parameter('min_us', 1200)
        self.declare_parameter('max_us', 1800)

        # Mapping degrees->pulse
        self.declare_parameter('max_deg', 180.0)

        # Motion limiting
        self.declare_parameter('speed_us_per_tick', 5)   # slower = safer
        self.declare_parameter('update_hz', 100)

        # Safety / behavior
        self.declare_parameter('simulate', SIM_DEFAULT)
        self.declare_parameter('enable_gpio', True)      # hard disable output
        self.declare_parameter('armed_on_start', False)  # require explicit arm
        self.declare_parameter('hold_us', 1500)          # safe idle/neutral pulse
        self.declare_parameter('startup_hold_s', 1.0)    # keep hold_us for a bit before allowing motion
        self.declare_parameter('stop_pwm_on_disarm', True)

        self.pwm_pin = int(self.get_parameter('pwm_pin').value)
        self.pwm_hz = float(self.get_parameter('pwm_hz').value)
        self.min_us = int(self.get_parameter('min_us').value)
        self.max_us = int(self.get_parameter('max_us').value)
        self.max_deg = float(self.get_parameter('max_deg').value)
        self.speed = int(self.get_parameter('speed_us_per_tick').value)
        self.update_hz = float(self.get_parameter('update_hz').value)

        self.simulate = bool(self.get_parameter('simulate').value) or (GPIO is None)
        self.enable_gpio = bool(self.get_parameter('enable_gpio').value)
        self.armed = bool(self.get_parameter('armed_on_start').value)

        self.hold_us = int(self.get_parameter('hold_us').value)
        self.hold_us = max(self.min_us, min(self.max_us, self.hold_us))

        self.startup_hold_s = float(self.get_parameter('startup_hold_s').value)
        self.stop_pwm_on_disarm = bool(self.get_parameter('stop_pwm_on_disarm').value)

        # State
        self.current_us = self.hold_us
        self.target_us = self.hold_us
        self._allow_motion_after = time.time() + max(0.0, self.startup_hold_s)

        # GPIO setup
        self.pwm = None
        if self.simulate or (not self.enable_gpio):
            self.get_logger().warn("Lift arm SIMULATION MODE (no PWM output)")
        else:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.pwm_pin, GPIO.OUT)
            self.pwm = GPIO.PWM(self.pwm_pin, self.pwm_hz)
            self.pwm.start(self._us_to_duty(self.hold_us))
            self.get_logger().info(f"Lift PWM started on BCM {self.pwm_pin} at hold_us={self.hold_us}")

        # Subscribers
        self.sub_cmd = self.create_subscription(Float32, 'lift_arm/command', self.command_cb, 10)
        self.sub_arm = self.create_subscription(Bool, 'lift_arm/arm', self.arm_cb, 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.update_hz, self.update_servo)

        self.get_logger().info(
            f"Lift arm ready | limits {self.min_us}-{self.max_us}us | hold_us={self.hold_us} | "
            f"armed={self.armed} | enable_gpio={self.enable_gpio} | simulate={self.simulate}"
        )

    # -------------------------------------------------------------

    def arm_cb(self, msg: Bool):
        if msg.data and (not self.armed):
            self.armed = True
            # Re-hold briefly to prevent instant jump on arming
            self.target_us = self.hold_us
            self._allow_motion_after = time.time() + 0.5
            self.get_logger().warn("Lift arm ARMED")
        elif (not msg.data) and self.armed:
            self.get_logger().warn("Lift arm DISARMED -> returning to hold")
            self.armed = False
            self.target_us = self.hold_us
            self._allow_motion_after = time.time() + 999999  # block motion until re-armed
            # Optionally stop PWM after we return to hold
            # (handled in update_servo once we reach hold)

    def command_cb(self, msg: Float32):
        # Ignore commands until armed and after startup hold
        if not self.armed:
            return
        if time.time() < self._allow_motion_after:
            return

        deg = float(msg.data)
        deg = max(0.0, min(self.max_deg, deg))

        span = self.max_us - self.min_us
        self.target_us = int(self.min_us + (deg / self.max_deg) * span)
        self.target_us = max(self.min_us, min(self.max_us, self.target_us))

    # -------------------------------------------------------------

    def update_servo(self):
        # Always ramp toward target_us safely
        delta = self.target_us - self.current_us
        if abs(delta) <= self.speed:
            self.current_us = self.target_us
        else:
            self.current_us += self.speed if delta > 0 else -self.speed

        self.current_us = max(self.min_us, min(self.max_us, self.current_us))

        if self.simulate or (not self.enable_gpio) or (self.pwm is None):
            return

        self.pwm.ChangeDutyCycle(self._us_to_duty(self.current_us))

        # If disarmed and we’ve returned to hold_us, optionally stop PWM cleanly
        if (not self.armed) and self.stop_pwm_on_disarm and (self.current_us == self.hold_us):
            # Hold briefly then stop
            time.sleep(0.1)
            try:
                self.pwm.ChangeDutyCycle(self._us_to_duty(self.hold_us))
                time.sleep(0.1)
                self.pwm.stop()
                GPIO.cleanup(self.pwm_pin)
                self.pwm = None
                self.get_logger().warn("PWM stopped (disarmed at hold)")
            except Exception:
                pass

    def _us_to_duty(self, us: int) -> float:
        period_us = 1_000_000.0 / self.pwm_hz
        return (float(us) / period_us) * 100.0

    # -------------------------------------------------------------

    def destroy_node(self):
        try:
            if self.pwm is not None:
                # Return to hold briefly before stopping
                self.pwm.ChangeDutyCycle(self._us_to_duty(self.hold_us))
                time.sleep(0.2)
                self.pwm.stop()
                GPIO.cleanup(self.pwm_pin)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiftArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

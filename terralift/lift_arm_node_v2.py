#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:  # pragma: no cover - optional hardware dependency
    import pigpio
except Exception:  # pragma: no cover
    pigpio = None

try:  # pragma: no cover - optional hardware dependency
    from gpiozero import Servo as GpioZeroServo
except Exception:  # pragma: no cover
    GpioZeroServo = None

try:  # pragma: no cover - optional hardware dependency
    import RPi.GPIO as GPIO
except Exception:  # pragma: no cover
    GPIO = None


def clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


class ServoBackendBase:
    backend_name = 'simulate'

    def set_pulse_us(self, pulse_us: float) -> None:
        return

    def close(self) -> None:
        return


class SimBackend(ServoBackendBase):
    backend_name = 'simulate'


class PigpioBackend(ServoBackendBase):
    backend_name = 'pigpio'

    def __init__(self, pin: int) -> None:
        if pigpio is None:
            raise RuntimeError('pigpio module unavailable')
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError('pigpio daemon not available')
        self.pin = pin
        self.pi.set_mode(self.pin, pigpio.OUTPUT)

    def set_pulse_us(self, pulse_us: float) -> None:
        self.pi.set_servo_pulsewidth(self.pin, pulse_us)

    def close(self) -> None:
        self.pi.set_servo_pulsewidth(self.pin, 0.0)
        self.pi.stop()


class GpioZeroBackend(ServoBackendBase):
    backend_name = 'gpiozero'

    def __init__(self, pin: int, min_us: float, max_us: float, pwm_hz: float) -> None:
        if GpioZeroServo is None:
            raise RuntimeError('gpiozero unavailable')
        self.min_us = float(min_us)
        self.max_us = float(max_us)
        self.servo = GpioZeroServo(
            pin,
            min_pulse_width=self.min_us / 1_000_000.0,
            max_pulse_width=self.max_us / 1_000_000.0,
            frame_width=1.0 / pwm_hz,
            initial_value=0.0,
        )

    def set_pulse_us(self, pulse_us: float) -> None:
        span = self.max_us - self.min_us
        if span <= 1e-6:
            value = 0.0
        else:
            value = ((pulse_us - self.min_us) / span) * 2.0 - 1.0
        self.servo.value = clamp(value, -1.0, 1.0)

    def close(self) -> None:
        self.servo.detach()
        self.servo.close()


class RpiGpioBackend(ServoBackendBase):
    backend_name = 'RPi.GPIO'

    def __init__(self, pin: int, pwm_hz: float) -> None:
        if GPIO is None:
            raise RuntimeError('RPi.GPIO unavailable')
        self.pin = pin
        self.pwm_hz = pwm_hz
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, self.pwm_hz)
        self.pwm.start(0.0)

    def set_pulse_us(self, pulse_us: float) -> None:
        period_us = 1_000_000.0 / self.pwm_hz
        duty = (pulse_us / period_us) * 100.0
        self.pwm.ChangeDutyCycle(duty)

    def close(self) -> None:
        self.pwm.ChangeDutyCycle(0.0)
        self.pwm.stop()
        GPIO.cleanup(self.pin)


class LiftArmNode(Node):
    def __init__(self) -> None:
        super().__init__('lift_arm')

        self.pwm_pin = int(self.declare_parameter('pwm_pin', 19).value)
        self.pwm_hz = float(self.declare_parameter('pwm_hz', 50.0).value)
        self.min_us = float(self.declare_parameter('min_us', 1200).value)
        self.max_us = float(self.declare_parameter('max_us', 1800).value)
        self.max_deg = float(self.declare_parameter('max_deg', 180.0).value)
        self.max_slew_us_per_sec = float(
            self.declare_parameter('max_slew_us_per_sec', 3000.0).value
        )
        self.command_deadband_us = float(
            self.declare_parameter('command_deadband_us', 2.0).value
        )
        self.output_deadband_us = float(
            self.declare_parameter('output_deadband_us', 1.0).value
        )
        self.update_hz = float(self.declare_parameter('update_hz', 120.0).value)
        self.simulate = bool(self.declare_parameter('simulate', False).value)
        self.preferred_backend = str(self.declare_parameter('backend', 'auto').value).lower()

        self.current_us = 0.5 * (self.min_us + self.max_us)
        self.target_us = self.current_us
        self.last_update = self.get_clock().now()
        self.last_sent_us = float('nan')

        self.backend = self._build_backend()
        if self.backend.backend_name == 'RPi.GPIO':
            self.get_logger().warn(
                'Lift arm is using RPi.GPIO software PWM; pigpio is recommended for less servo jitter'
            )
        self._write_output(self.current_us, force=True)

        self.create_subscription(Float32, 'lift_arm/command', self.command_cb, 10)
        self.timer = self.create_timer(1.0 / max(1.0, self.update_hz), self.update_servo)

        self.get_logger().info(
            f'Lift arm ready with {self.backend.backend_name} backend '
            f'(limits {self.min_us:.0f}-{self.max_us:.0f} us)'
        )

    def _build_backend(self) -> ServoBackendBase:
        if self.simulate:
            self.get_logger().warn('Lift arm simulation mode enabled')
            return SimBackend()

        if self.preferred_backend == 'auto':
            backend_order = ['pigpio', 'gpiozero', 'rpi.gpio']
        else:
            backend_order = [self.preferred_backend]

        for name in backend_order:
            try:
                if name == 'pigpio':
                    return PigpioBackend(self.pwm_pin)
                if name == 'gpiozero':
                    return GpioZeroBackend(self.pwm_pin, self.min_us, self.max_us, self.pwm_hz)
                if name in ('rpi.gpio', 'rpigpio'):
                    return RpiGpioBackend(self.pwm_pin, self.pwm_hz)
            except Exception as exc:
                self.get_logger().warn(f'Lift backend {name} unavailable: {exc}')

        self.get_logger().warn('No hardware servo backend available; using simulation mode')
        return SimBackend()

    def command_cb(self, msg: Float32) -> None:
        deg = clamp(float(msg.data), 0.0, self.max_deg)
        span = self.max_us - self.min_us
        self.target_us = self.min_us + (deg / max(1e-6, self.max_deg)) * span

    def _write_output(self, pulse_us: float, force: bool = False) -> None:
        if (
            not force and
            self.output_deadband_us > 0.0 and
            math.isfinite(self.last_sent_us) and
            abs(pulse_us - self.last_sent_us) <= self.output_deadband_us
        ):
            return
        self.backend.set_pulse_us(pulse_us)
        self.last_sent_us = pulse_us

    def update_servo(self) -> None:
        now = self.get_clock().now()
        dt = max(0.0, (now - self.last_update).nanoseconds * 1e-9)
        self.last_update = now
        if dt <= 0.0:
            return

        delta = self.target_us - self.current_us
        max_step = max(1.0, self.max_slew_us_per_sec * dt)

        if abs(delta) <= max(self.command_deadband_us, 1e-6):
            self.current_us = self.target_us
        else:
            self.current_us += math.copysign(min(abs(delta), max_step), delta)

        self.current_us = clamp(self.current_us, self.min_us, self.max_us)
        self._write_output(self.current_us)

    def destroy_node(self):
        self.backend.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LiftArmNode()
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

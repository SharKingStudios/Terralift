#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

SIM_DEFAULT = False
try:
    from rpi_ws281x import PixelStrip, Color
except Exception:
    SIM_DEFAULT = True
    PixelStrip = None
    Color = None


class StatusLedNode(Node):
    def __init__(self):
        super().__init__('status_leds')

        # ---------------- Parameters ----------------
        self.declare_parameter('led_count', 144)
        self.declare_parameter('gpio_pin', 21)
        self.declare_parameter('brightness', 64)
        self.declare_parameter('base_pulse_hz', 0.5)
        self.declare_parameter('tag_pulse_multiplier', 2.5)
        self.declare_parameter('frame_hz', 50.0)
        self.declare_parameter('simulate', SIM_DEFAULT)

        self.led_count   = self.get_parameter('led_count').value
        self.gpio_pin    = self.get_parameter('gpio_pin').value
        self.brightness  = self.get_parameter('brightness').value
        self.base_hz     = self.get_parameter('base_pulse_hz').value
        self.tag_mult    = self.get_parameter('tag_pulse_multiplier').value
        self.frame_hz    = self.get_parameter('frame_hz').value
        self.simulate    = self.get_parameter('simulate').value

        # ---------------- State ----------------
        self.apriltag_locked = False
        self.phase = 0.0

        # ---------------- LED Init ----------------
        if self.simulate:
            self.get_logger().warn("LED SIMULATION MODE (no WS2812 output)")
            self.strip = None
        else:
            self.strip = PixelStrip(
                self.led_count,
                self.gpio_pin,
                brightness=self.brightness,
                auto_write=False
            )
            self.strip.begin()
            self.clear_strip()

        # ---------------- Timer ----------------
        period = 1.0 / self.frame_hz
        self.timer = self.create_timer(period, self.update_animation)

        self.get_logger().info("Status LED node ready")

    # --------------------------------------------------

    def update_animation(self):
        # Determine pulse speed
        freq = self.base_hz
        if self.apriltag_locked:
            freq *= self.tag_mult

        # Advance phase
        self.phase += 2.0 * math.pi * freq / self.frame_hz
        intensity = (math.sin(self.phase) + 1.0) / 2.0  # 0..1

        green = int(255 * intensity)

        if self.simulate:
            self.get_logger().debug(f"[SIM] Green intensity: {green}")
            return

        color = Color(0, green, 0)
        for i in range(self.led_count):
            self.strip.setPixelColor(i, color)

        self.strip.show()

    # --------------------------------------------------

    def clear_strip(self):
        if not self.strip:
            return
        for i in range(self.led_count):
            self.strip.setPixelColor(i, Color(0, 0, 0))
        self.strip.show()

    # --------------------------------------------------

    def destroy_node(self):
        self.clear_strip()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StatusLedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

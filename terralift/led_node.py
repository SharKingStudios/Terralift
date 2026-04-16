#!/usr/bin/env python3
from __future__ import annotations

import colorsys
import random
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from rpi_ws281x import Color, PixelStrip, ws
except Exception:  # pragma: no cover - optional hardware dependency
    Color = None
    PixelStrip = None
    ws = None


LED_OFF = 'OFF'
LED_IDLE = 'IDLE_CONFETTI'
LED_PLANNING = 'PLANNING_YELLOW'
LED_FOLLOWING = 'FOLLOWING_GREEN_FLASH'
LED_RED = 'RED'
LED_GREEN = 'GREEN'
LED_GREEN_PULSE = 'GREEN_PULSE'
LED_GREEN_FAST = 'GREEN_FAST'
LED_YELLOW = 'YELLOW'

LEGACY_MODE_MAP = {
    'OFF': LED_OFF,
    'IDLE': LED_IDLE,
    'GREEN': LED_GREEN,
    'GREEN_PULSE': LED_GREEN_PULSE,
    'GREEN_FAST': LED_GREEN_FAST,
    'YELLOW': LED_YELLOW,
    'RED': LED_RED,
    'PLANNING_YELLOW': LED_PLANNING,
    'FOLLOWING_GREEN_FLASH': LED_FOLLOWING,
    'IDLE_CONFETTI': LED_IDLE,
}


def clamp_u8(value: float) -> int:
    return int(max(0, min(255, round(value))))


class SimStrip:
    def __init__(self, count: int) -> None:
        self.count = count
        self.pixels = [(0, 0, 0)] * count

    def set_pixel(self, index: int, rgb: Tuple[int, int, int]) -> None:
        self.pixels[index] = rgb

    def show(self) -> None:
        return

    def clear(self) -> None:
        self.pixels = [(0, 0, 0)] * self.count


class Ws281xStrip:
    def __init__(
        self,
        count: int,
        gpio_pin: int,
        brightness: int,
        dma: int,
        freq_hz: int,
        invert: bool,
        channel: int,
        strip_type_name: str,
    ) -> None:
        strip_type = getattr(ws, strip_type_name, ws.WS2811_STRIP_GRB)
        self.strip = PixelStrip(
            count,
            gpio_pin,
            freq_hz=freq_hz,
            dma=dma,
            invert=invert,
            brightness=brightness,
            channel=channel,
            strip_type=strip_type,
        )
        self.strip.begin()
        self.count = count

    def set_pixel(self, index: int, rgb: Tuple[int, int, int]) -> None:
        self.strip.setPixelColor(index, Color(*rgb))

    def show(self) -> None:
        self.strip.show()

    def clear(self) -> None:
        for i in range(self.count):
            self.strip.setPixelColor(i, Color(0, 0, 0))
        self.strip.show()


class StatusLedNode(Node):
    def __init__(self) -> None:
        super().__init__('status_leds')

        self.led_count = int(self.declare_parameter('led_count', 144).value)
        self.gpio_pin = int(self.declare_parameter('gpio_pin', 4).value)
        self.brightness = int(self.declare_parameter('brightness', 64).value)
        self.frame_hz = float(self.declare_parameter('frame_hz', 50.0).value)
        self.simulate = bool(self.declare_parameter('simulate', PixelStrip is None).value)
        self.led_topic = str(self.declare_parameter('led_topic', '/led/state').value)
        self.idle_fade_by = int(self.declare_parameter('idle_fade_by', 10).value)
        self.follow_flash_hz = float(self.declare_parameter('follow_flash_hz', 8.0).value)
        self.dma = int(self.declare_parameter('dma', 10).value)
        self.freq_hz = int(self.declare_parameter('freq_hz', 800000).value)
        self.channel = int(self.declare_parameter('channel', 0).value)
        self.invert = bool(self.declare_parameter('invert', False).value)
        self.strip_type = str(self.declare_parameter('strip_type', 'WS2811_STRIP_GRB').value)

        self.mode = LED_IDLE
        self.last_logged_mode = ''
        self.frame_index = 0
        self.idle_pixels: List[Tuple[int, int, int]] = [(0, 0, 0)] * self.led_count
        self.strip = self._build_strip()

        self.create_subscription(String, self.led_topic, self._mode_cb, 10)
        self.timer = self.create_timer(1.0 / max(1.0, self.frame_hz), self._update_animation)

        self.get_logger().info(
            f'Status LED node ready on {self.led_topic} (simulate={self.simulate})'
        )

    def _build_strip(self):
        if self.simulate:
            self.get_logger().warn('LED simulation mode enabled')
            return SimStrip(self.led_count)

        if PixelStrip is None or Color is None or ws is None:
            self.simulate = True
            self.get_logger().warn('rpi_ws281x unavailable; falling back to LED simulation mode')
            return SimStrip(self.led_count)

        try:
            strip = Ws281xStrip(
                count=self.led_count,
                gpio_pin=self.gpio_pin,
                brightness=self.brightness,
                dma=self.dma,
                freq_hz=self.freq_hz,
                invert=self.invert,
                channel=self.channel,
                strip_type_name=self.strip_type,
            )
        except Exception as exc:
            self.simulate = True
            self.get_logger().warn(f'LED init failed ({exc}); falling back to simulation mode')
            return SimStrip(self.led_count)

        strip.clear()
        return strip

    def _mode_cb(self, msg: String) -> None:
        incoming = LEGACY_MODE_MAP.get(str(msg.data).strip().upper(), LED_IDLE)
        if incoming == self.mode:
            return
        self.mode = incoming
        self.frame_index = 0
        if self.mode != LED_IDLE:
            self.idle_pixels = [(0, 0, 0)] * self.led_count
        self._log_mode_change()

    def _log_mode_change(self) -> None:
        if self.mode == self.last_logged_mode:
            return
        self.last_logged_mode = self.mode
        self.get_logger().info(f'LED mode -> {self.mode}')

    def _update_animation(self) -> None:
        self.frame_index += 1

        if self.mode == LED_OFF:
            self._set_all((0, 0, 0))
        elif self.mode == LED_IDLE:
            self._render_idle_confetti()
        elif self.mode in (LED_PLANNING, LED_YELLOW):
            self._set_all((255, 190, 0))
        elif self.mode in (LED_FOLLOWING, LED_GREEN_FAST):
            on = int((self.frame_index * self.follow_flash_hz) / max(1.0, self.frame_hz)) % 2 == 0
            self._set_all((0, 255, 0) if on else (0, 0, 0))
        elif self.mode in (LED_GREEN, LED_GREEN_PULSE):
            self._set_all((0, 180, 0))
        elif self.mode == LED_RED:
            self._set_all((255, 0, 0))
        else:
            self._render_idle_confetti()

    def _render_idle_confetti(self) -> None:
        fade = clamp_u8(255 - self.idle_fade_by)
        next_pixels: List[Tuple[int, int, int]] = []
        for r, g, b in self.idle_pixels:
            next_pixels.append((
                (r * fade) // 255,
                (g * fade) // 255,
                (b * fade) // 255,
            ))

        hue = random.random()
        sat = 200.0 / 255.0
        add_r, add_g, add_b = colorsys.hsv_to_rgb(hue, sat, 1.0)
        add_rgb = (
            clamp_u8(add_r * 255.0),
            clamp_u8(add_g * 255.0),
            clamp_u8(add_b * 255.0),
        )

        index = random.randrange(self.led_count)
        old_r, old_g, old_b = next_pixels[index]
        next_pixels[index] = (
            clamp_u8(old_r + add_rgb[0]),
            clamp_u8(old_g + add_rgb[1]),
            clamp_u8(old_b + add_rgb[2]),
        )

        self.idle_pixels = next_pixels
        for i, rgb in enumerate(self.idle_pixels):
            self.strip.set_pixel(i, rgb)
        self.strip.show()

    def _set_all(self, rgb: Tuple[int, int, int]) -> None:
        for i in range(self.led_count):
            self.strip.set_pixel(i, rgb)
        self.strip.show()

    def destroy_node(self):
        self.strip.clear()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StatusLedNode()
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

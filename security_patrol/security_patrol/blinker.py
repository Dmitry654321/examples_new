# blinker.py
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA
from rclpy.node import Node


class LedMode(str, Enum):
    OFF = "off"
    DIM_WHITE = "dim_white"
    BLUE_BLINK = "blue_blink"
    RED_BLINK = "red_blink"
    GREEN_BLIP = "green_blip"


@dataclass
class LedParams:
    led_count: int = 5  # common default; keep configurable
    dim_intensity: float = 0.10
    blink_hz: float = 3.0
    green_blip_hz: float = 6.0


class LedController:
    """
    Publishes duckietown_msgs/LEDPattern:
      Header
      string[] color_list
      ColorRGBA[] rgb_vals
      int8[] color_mask
      float32 frequency
      int8[] frequency_mask
    :contentReference[oaicite:3]{index=3}
    """

    def __init__(self, node: Node, topic: str, p: LedParams):
        self._node = node
        self._p = p
        self._pub = node.create_publisher(LEDPattern, topic, 1)
        self._mode: LedMode = LedMode.OFF
        self._last_publish_stamp = node.get_clock().now()

    def set_mode(self, mode: LedMode) -> None:
        if mode == self._mode:
            return
        self._mode = mode
        self.publish(force=True)

    def publish(self, force: bool = False) -> None:
        # Re-publish occasionally so the robot keeps the pattern even if another node restarts.
        now = self._node.get_clock().now()
        if not force and (now - self._last_publish_stamp).nanoseconds < int(1e9):
            return
        self._last_publish_stamp = now

        msg = LEDPattern()
        msg.header.stamp = now.to_msg()

        # one-color pattern; each LED points to color index 0
        msg.color_list = ["custom"]
        msg.rgb_vals = [self._color_for_mode()]
        msg.color_mask = [0 for _ in range(self._p.led_count)]

        freq, fmask = self._freq_for_mode()
        msg.frequency = float(freq)
        msg.frequency_mask = fmask

        self._pub.publish(msg)

    def _color_for_mode(self) -> ColorRGBA:
        c = ColorRGBA()
        c.a = 1.0
        if self._mode == LedMode.OFF:
            c.r, c.g, c.b = 0.0, 0.0, 0.0
        elif self._mode == LedMode.DIM_WHITE:
            x = self._p.dim_intensity
            c.r, c.g, c.b = x, x, x
        elif self._mode == LedMode.BLUE_BLINK:
            c.r, c.g, c.b = 0.0, 0.0, 1.0
        elif self._mode == LedMode.RED_BLINK:
            c.r, c.g, c.b = 1.0, 0.0, 0.0
        elif self._mode == LedMode.GREEN_BLIP:
            c.r, c.g, c.b = 0.0, 1.0, 0.0
        else:
            c.r, c.g, c.b = 0.0, 0.0, 0.0
        return c

    def _freq_for_mode(self) -> tuple[float, List[int]]:
        if self._mode in (LedMode.BLUE_BLINK, LedMode.RED_BLINK):
            return self._p.blink_hz, [1 for _ in range(self._p.led_count)]
        if self._mode == LedMode.GREEN_BLIP:
            return self._p.green_blip_hz, [1 for _ in range(self._p.led_count)]
        return 0.0, [0 for _ in range(self._p.led_count)]

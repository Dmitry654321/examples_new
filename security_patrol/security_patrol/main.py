# main.py
from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import CompressedImage, Range

from .motion import WallFollower, WallFollowParams, ApproachController, ApproachParams
from .blinker import LedController, LedParams, LedMode


# ---------------- Detection types ----------------

@dataclass
class Detection:
    cls: str                 # "standing" | "laying" | "thief"
    conf: float              # 0..1
    bbox: Tuple[int, int, int, int]  # x,y,w,h


class HeuristicDetector:
    """
    Baseline LEGO-person detector using HSV thresholds:
      - Finds "yellow" blob (LEGO head/skin) to localize a person-like object.
      - Classifies standing vs laying by aspect ratio.
      - Flags thief if bbox contains a lot of dark pixels (e.g., black clothing).
    """

    def __init__(self):
        # tune these in your lighting
        self.yellow_h_lo, self.yellow_h_hi = 15, 40
        self.yellow_s_lo = 80
        self.yellow_v_lo = 80

        self.dark_v_hi = 55  # dark pixels: V <= this

        self.min_area_px = 500
        self.thief_dark_frac = 0.22

        self.standing_ratio = 1.25  # h/w above this => standing
        self.laying_ratio = 0.95    # h/w below this => laying

    def detect(self, bgr: np.ndarray) -> List[Detection]:
        h, w = bgr.shape[:2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # Yellow mask
        lower = np.array([self.yellow_h_lo, self.yellow_s_lo, self.yellow_v_lo], dtype=np.uint8)
        upper = np.array([self.yellow_h_hi, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        # Clean
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []

        c = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(c))
        if area < self.min_area_px:
            return []

        x, y, bw, bh = cv2.boundingRect(c)

        # Classify pose by aspect ratio
        ratio = (bh / max(1.0, float(bw)))
        dets: List[Detection] = []

        # "person found" confidence from area fraction (simple heuristic)
        conf = float(min(1.0, (area / (w * h)) * 12.0))

        if ratio >= self.standing_ratio:
            dets.append(Detection("standing", conf, (x, y, bw, bh)))
        elif ratio <= self.laying_ratio:
            dets.append(Detection("laying", conf, (x, y, bw, bh)))
        else:
            # ambiguous: treat as standing by default (safe: no approach unless clearly laying)
            dets.append(Detection("standing", conf * 0.7, (x, y, bw, bh)))

        # Thief heuristic: fraction of dark pixels inside bbox
        roi = hsv[y:y+bh, x:x+bw]
        if roi.size > 0:
            v = roi[:, :, 2]
            dark_frac = float(np.mean(v <= self.dark_v_hi))
            if dark_frac >= self.thief_dark_frac:
                dets.append(Detection("thief", min(1.0, conf * 0.9 + 0.1), (x, y, bw, bh)))

        return dets


# ---------------- Smoothing ----------------

class SmoothTrigger:
    def __init__(self, n_frames: int, disappear_timeout_s: float):
        self.n_frames = max(1, int(n_frames))
        self.disappear_timeout_s = float(disappear_timeout_s)

        self.count = 0
        self.last_seen_t = 0.0
        self.active = False

    def update(self, seen: bool, now: float) -> bool:
        if seen:
            self.last_seen_t = now
            self.count += 1
        else:
            self.count = 0

        if not self.active and self.count >= self.n_frames:
            self.active = True

        if self.active and (now - self.last_seen_t) > self.disappear_timeout_s:
            self.active = False
            self.count = 0

        return self.active


# ---------------- FSM ----------------

class State(str, Enum):
    PATROL_WALLFOLLOW = "patrol"
    THIEF_ALERT = "thief_alert"
    EMERGENCY_APPROACH = "emergency_approach"
    EMERGENCY_WAIT = "emergency_wait"
    STANDING_SEEN = "standing_seen"


class SecurityPatrolNode(Node):
    def __init__(self):
        super().__init__("security_patrol_node")

        # -------- Parameters --------
        self.declare_parameter("vehicle", "duckie02")

        # Topics (override if your names differ)
        self.declare_parameter("image_topic", "")
        self.declare_parameter("range_topic", "")
        self.declare_parameter("wheels_topic", "")
        self.declare_parameter("led_topic", "")

        # Safety + timing
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("frame_timeout_s", 0.8)
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("det_fps", 8.0)

        # Smoothing
        self.declare_parameter("n_frames_trigger", 3)
        self.declare_parameter("lost_target_s", 1.2)

        # Emergency approach
        self.declare_parameter("close_area_frac", 0.16)
        self.declare_parameter("close_h_frac", 0.60)
        self.declare_parameter("wait_s", 20.0)

        # Standing optional
        self.declare_parameter("standing_green_s", 0.6)

        vehicle = self.get_parameter("vehicle").value

        image_topic = self.get_parameter("image_topic").value or f"/{vehicle}/camera_node/image/compressed"
        range_topic = self.get_parameter("range_topic").value or f"/{vehicle}/range"
        wheels_topic = self.get_parameter("wheels_topic").value or f"/{vehicle}/wheels_driver_node/wheels_cmd"
        led_topic = self.get_parameter("led_topic").value or f"/{vehicle}/led_pattern"

        # -------- Pub/Sub --------
        self.wheels_pub = self.create_publisher(WheelsCmdStamped, wheels_topic, 1)
        self.range_sub = self.create_subscription(Range, range_topic, self.on_range, 10)
        self.image_sub = self.create_subscription(CompressedImage, image_topic, self.on_image, 10)

        # -------- Controllers --------
        self.wall = WallFollower(WallFollowParams())
        self.approach = ApproachController(ApproachParams())

        self.led = LedController(self, led_topic, LedParams())
        self.led.set_mode(LedMode.DIM_WHITE)

        # -------- Detector + runtime --------
        self.detector = HeuristicDetector()
        self.last_det_t = 0.0
        self.det_period = 1.0 / max(1.0, float(self.get_parameter("det_fps").value))

        self.last_frame_t = 0.0
        self.frame_timeout_s = float(self.get_parameter("frame_timeout_s").value)

        self.img_wh: Optional[Tuple[int, int]] = None
        self.last_dets: List[Detection] = []

        self.range_m: Optional[float] = None

        # -------- Smoothing triggers --------
        n = int(self.get_parameter("n_frames_trigger").value)
        lost_s = float(self.get_parameter("lost_target_s").value)
        self.trig_laying = SmoothTrigger(n, lost_s)
        self.trig_thief = SmoothTrigger(n, lost_s)
        self.trig_standing = SmoothTrigger(n, lost_s)

        # FSM
        self.state: State = State.PATROL_WALLFOLLOW
        self.state_t0 = time.time()

        # For EMERGENCY_WAIT
        self.wait_s = float(self.get_parameter("wait_s").value)

        # Control loop
        control_hz = float(self.get_parameter("control_hz").value)
        self.timer = self.create_timer(1.0 / max(2.0, control_hz), self.tick)

        self.get_logger().info(f"SecurityPatrol running. image={image_topic} range={range_topic}")
        self.get_logger().info(f"wheels={wheels_topic} led={led_topic}")

    # -------- ROS callbacks --------

    def on_range(self, msg: Range):
        self.range_m = float(msg.range)

    def on_image(self, msg: CompressedImage):
        self.last_frame_t = time.time()

        now = self.last_frame_t
        if (now - self.last_det_t) < self.det_period:
            return
        self.last_det_t = now

        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        h, w = frame.shape[:2]
        self.img_wh = (w, h)
        self.last_dets = self.detector.detect(frame)

    # -------- Helpers --------

    def _publish_wheels(self, vl: float, vr: float):
        msg = WheelsCmdStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vel_left = float(vl)
        msg.vel_right = float(vr)
        self.wheels_pub.publish(msg)

    def _stop(self):
        self._publish_wheels(0.0, 0.0)

    def _enable_motion(self) -> bool:
        return bool(self.get_parameter("enable_motion").value)

    def _camera_ok(self) -> bool:
        return (time.time() - self.last_frame_t) <= self.frame_timeout_s

    def _best_det(self, cls: str) -> Optional[Detection]:
        cands = [d for d in self.last_dets if d.cls == cls]
        if not cands:
            return None
        return max(cands, key=lambda d: d.conf)

    # -------- Control tick / FSM --------

    def tick(self):
        now = time.time()

        # Safety: if camera stopped -> stop wheels immediately
        if not self._camera_ok():
            self.led.set_mode(LedMode.RED_BLINK)
            self._stop()
            return

        # Update triggers
        laying_det = self._best_det("laying")
        thief_det = self._best_det("thief")
        standing_det = self._best_det("standing")

        laying_active = self.trig_laying.update(laying_det is not None, now)
        thief_active = self.trig_thief.update(thief_det is not None, now)
        standing_active = self.trig_standing.update(standing_det is not None, now)

        # Priority: EMERGENCY > THIEF > STANDING
        if self.state not in (State.EMERGENCY_APPROACH, State.EMERGENCY_WAIT) and laying_active:
            self.state = State.EMERGENCY_APPROACH
            self.state_t0 = now

        elif self.state == State.PATROL_WALLFOLLOW and thief_active:
            self.state = State.THIEF_ALERT
            self.state_t0 = now

        elif self.state == State.PATROL_WALLFOLLOW and standing_active:
            self.state = State.STANDING_SEEN
            self.state_t0 = now

        # Execute current state
        if self.state == State.PATROL_WALLFOLLOW:
            self.led.set_mode(LedMode.DIM_WHITE)
            vl, vr = self.wall.compute(self.range_m)
            self._actuate(vl, vr)

        elif self.state == State.THIEF_ALERT:
            self.led.set_mode(LedMode.BLUE_BLINK)
            # continue patrol (optionally slowed)
            # quick simple slow-down:
            old_base = self.wall.p.base
            self.wall.p.base = max(0.12, old_base * 0.75)
            vl, vr = self.wall.compute(self.range_m)
            self.wall.p.base = old_base

            self._actuate(vl, vr)

            # exit when thief not seen for X seconds (trigger handles it)
            if not thief_active:
                self.state = State.PATROL_WALLFOLLOW
                self.state_t0 = now

        elif self.state == State.EMERGENCY_APPROACH:
            self.led.set_mode(LedMode.RED_BLINK)

            # If we lost the bbox -> stop (safe) and fall back to patrol
            if laying_det is None or self.img_wh is None:
                self._stop()
                self.state = State.PATROL_WALLFOLLOW
                self.state_t0 = now
                return

            w, h = self.img_wh
            x, y, bw, bh = laying_det.bbox
            cx = x + bw / 2.0

            # Close criterion
            close_area_frac = float(self.get_parameter("close_area_frac").value)
            close_h_frac = float(self.get_parameter("close_h_frac").value)
            area_frac = (bw * bh) / max(1.0, float(w * h))
            h_frac = bh / max(1.0, float(h))

            if area_frac >= close_area_frac or h_frac >= close_h_frac:
                self._stop()
                self.state = State.EMERGENCY_WAIT
                self.state_t0 = now
                return

            vl, vr = self.approach.compute(w, cx)
            self._actuate(vl, vr)

        elif self.state == State.EMERGENCY_WAIT:
            self.led.set_mode(LedMode.RED_BLINK)
            self._stop()
            if (now - self.state_t0) >= self.wait_s:
                self.state = State.PATROL_WALLFOLLOW
                self.state_t0 = now

        elif self.state == State.STANDING_SEEN:
            # Optional: brief green blink then back to patrol
            self.led.set_mode(LedMode.GREEN_BLIP)
            self._stop()  # do nothing special
            green_s = float(self.get_parameter("standing_green_s").value)
            if (now - self.state_t0) >= green_s:
                self.state = State.PATROL_WALLFOLLOW
                self.state_t0 = now

        # Maintain LED pattern publication
        self.led.publish()

    def _actuate(self, vl: float, vr: float):
        # Manual override parameter
        if not self._enable_motion():
            self._stop()
            return
        self._publish_wheels(vl, vr)


def main():
    rclpy.init()
    node = SecurityPatrolNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

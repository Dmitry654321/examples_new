# motion.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class WallFollowParams:
    follow_left_wall: bool = True
    target_m: float = 0.25
    kp: float = 1.8

    # Base speed in "wheel command units" (Duckietown uses WheelsCmdStamped floats;
    # exact units vary by stack, but this works as a practical controller gain space)
    base: float = 0.28

    # When the wall is "lost" (range too large), turn to find it
    no_wall_dist_m: float = 0.80
    search_turn: float = 0.18  # differential term

    # Hard safety stop
    stop_dist_m: float = 0.12

    # clamp wheel commands
    max_cmd: float = 0.60


class WallFollower:
    """
    Range-based wall follower producing (vel_left, vel_right).

    Convention used:
      vel_left  = base - turn
      vel_right = base + turn

    With this convention:
      turn > 0  -> robot turns LEFT  (right wheel faster)
      turn < 0  -> robot turns RIGHT (left wheel faster)
    """

    def __init__(self, p: WallFollowParams):
        self.p = p

    def compute(self, rng_m: float | None) -> Tuple[float, float]:
        # If no reading yet -> stop (safe)
        if rng_m is None:
            return 0.0, 0.0

        # Safety stop
        if rng_m < self.p.stop_dist_m:
            return 0.0, 0.0

        # Lost wall -> search
        if rng_m > self.p.no_wall_dist_m:
            turn = self.p.search_turn if self.p.follow_left_wall else -self.p.search_turn
            vl = clamp(self.p.base - turn, -self.p.max_cmd, self.p.max_cmd)
            vr = clamp(self.p.base + turn, -self.p.max_cmd, self.p.max_cmd)
            return vl, vr

        # Normal wall-follow
        error = rng_m - self.p.target_m  # + if too far, - if too close
        turn = self.p.kp * error
        if not self.p.follow_left_wall:
            turn = -turn  # mirror for right-wall following

        vl = clamp(self.p.base - turn, -self.p.max_cmd, self.p.max_cmd)
        vr = clamp(self.p.base + turn, -self.p.max_cmd, self.p.max_cmd)
        return vl, vr


@dataclass
class ApproachParams:
    v: float = 0.18
    k_turn: float = 0.25
    max_cmd: float = 0.45


class ApproachController:
    """
    Use bbox center to turn toward target and drive forward slowly.
    """

    def __init__(self, p: ApproachParams):
        self.p = p

    def compute(self, img_w: int, bbox_cx: float) -> Tuple[float, float]:
        # error_norm in [-1, +1]
        if img_w <= 1:
            return 0.0, 0.0
        error_norm = (bbox_cx - (img_w / 2.0)) / (img_w / 2.0)

        # If target is right (error_norm > 0), we want turn RIGHT => left wheel faster
        turn = self.p.k_turn * error_norm

        vl = clamp(self.p.v + turn, -self.p.max_cmd, self.p.max_cmd)
        vr = clamp(self.p.v - turn, -self.p.max_cmd, self.p.max_cmd)
        return vl, vr

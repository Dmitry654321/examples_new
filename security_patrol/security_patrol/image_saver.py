# image_saver.py
from __future__ import annotations

import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class ImageSaver(Node):
    def __init__(self):
        super().__init__("image_saver")

        self.declare_parameter("vehicle", "duckie02")
        self.declare_parameter("image_topic", "")  # if empty -> auto
        self.declare_parameter("save_dir", "dataset")
        self.declare_parameter("label", "unlabeled")
        self.declare_parameter("fps", 2.0)

        vehicle = self.get_parameter("vehicle").value
        image_topic = self.get_parameter("image_topic").value
        if not image_topic:
            image_topic = f"/{vehicle}/camera_node/image/compressed"

        self.save_dir = Path(self.get_parameter("save_dir").value) / str(self.get_parameter("label").value)
        self.save_dir.mkdir(parents=True, exist_ok=True)

        self.min_dt = 1.0 / max(0.1, float(self.get_parameter("fps").value))
        self.last_save_t = 0.0

        self.sub = self.create_subscription(CompressedImage, image_topic, self.cb, 10)
        self.get_logger().info(f"Saving from {image_topic} -> {self.save_dir} at ~{1/self.min_dt:.1f} FPS")

    def cb(self, msg: CompressedImage):
        now = time.time()
        if (now - self.last_save_t) < self.min_dt:
            return
        self.last_save_t = now

        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            return

        ts = int(now * 1000)
        path = self.save_dir / f"{ts}.jpg"
        cv2.imwrite(str(path), img)


def main():
    rclpy.init()
    node = ImageSaver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

#!/usr/bin/env python3
"""Validate the three camera streams of the bartender scene.

Subscribes to /camera/{overhead,side,wrist}/image_raw (and camera_info),
prints resolution / encoding / timestamp / frame_id for one frame of each,
saves those frames as PNGs, and measures the observed rate over a window.

    python3 scripts/validate_cameras.py
    python3 scripts/validate_cameras.py --window 8

Saves to artifacts/camera_validation/{overhead,side,wrist}.png. Exits 0
only if all three streams produced images.
"""
import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

import numpy as np

try:
    import cv2
except ImportError:                                       # pragma: no cover
    cv2 = None

REPO = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir))
OUT_DIR = os.path.join(REPO, 'artifacts', 'camera_validation')
CAMERAS = ['overhead', 'side', 'wrist']
QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

# The gz bridge maps R8G8B8 to 'rgb8' (or 'bgr8' in some versions); accept
# both rather than guessing.
ENCODINGS = ('rgb8', 'bgr8')


class CameraCollector(Node):

    def __init__(self):
        super().__init__('validate_cameras')
        self.images = {}          # camera -> (Image, arrival time)
        self.infos = {}            # camera -> CameraInfo
        self.counts = {name: 0 for name in CAMERAS}
        self.first = {}
        self.last = {}
        for name in CAMERAS:
            self.create_subscription(
                Image, f'/camera/{name}/image_raw',
                (lambda n: (lambda m: self._on_image(n, m)))(name), QOS)
            self.create_subscription(
                CameraInfo, f'/camera/{name}/camera_info',
                (lambda n: (lambda m: self._on_info(n, m)))(name), QOS)

    def _on_image(self, name, msg):
        self.counts[name] += 1
        now = time.monotonic()
        if name not in self.first:
            self.first[name] = now
        self.last[name] = now
        if name not in self.images:
            self.images[name] = (msg, now)

    def _on_info(self, name, msg):
        if name not in self.infos:
            self.infos[name] = msg

    def rates(self, window):
        """Observed Hz over the collection window, per camera."""
        rates = {}
        for name in CAMERAS:
            span = self.last.get(name, 0) - self.first.get(name, 0)
            rates[name] = (self.counts[name] - 1) / span if span > 0 else 0.0
        return rates


def to_bgr(msg):
    """sensor_msgs/Image -> BGR ndarray; enough for these RGB streams."""
    if msg.encoding not in ENCODINGS:
        raise ValueError(f'unexpected encoding {msg.encoding}')
    data = np.frombuffer(msg.data, dtype=np.uint8)
    img = data.reshape(msg.height, msg.width, 3)
    return img[:, :, ::-1] if msg.encoding == 'rgb8' else img.copy()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--window', type=float, default=5.0,
                        help='Seconds to measure the rate over.')
    args = parser.parse_args()

    rclpy.init()
    node = CameraCollector()
    deadline = time.monotonic() + args.window
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    ok = True
    os.makedirs(OUT_DIR, exist_ok=True)
    rates = node.rates(args.window)
    for name in CAMERAS:
        entry = node.images.get(name)
        info = node.infos.get(name)
        if entry is None:
            print(f'{name:9s} NO IMAGES on /camera/{name}/image_raw')
            ok = False
            continue
        msg, arrived = entry
        stamp = msg.header.stamp
        print(f'{name:9s} {msg.width}x{msg.height} {msg.encoding} '
              f'stamp={stamp.sec}.{stamp.nanosec:09d} '
              f'frame_id={msg.header.frame_id or "(empty)"} '
              f'~{rates[name]:.1f} Hz')
        if info is None:
            print(f'          NO camera_info on /camera/{name}/camera_info')
            ok = False
        if cv2 is not None:
            out = os.path.join(OUT_DIR, f'{name}.png')
            cv2.imwrite(out, to_bgr(msg))
            print(f'          saved {os.path.relpath(out, REPO)}')
        else:
            print('          (cv2 not available; frame not saved)')
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

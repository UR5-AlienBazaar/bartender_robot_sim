#!/usr/bin/env python3
"""Three-camera debug view for the bartender scene.

Shows the three RGB streams side by side:

    +----------------+----------------+
    |   OVERHEAD     |     SIDE       |
    +----------------+----------------+
    |              WRIST              |
    +---------------------------------+

    python3 scripts/view_cameras.py

A debugging tool only: the combined image is NOT a policy input, and the
individual streams stay separately available. Needs a display (X11); quit
with q or Ctrl-C.
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit('view_cameras needs cv2 (python3-opencv)')

CAMERAS = ['overhead', 'side', 'wrist']
QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
PANE_W, PANE_H = 640, 480


def to_bgr(msg):
    img = np.frombuffer(msg.data, dtype=np.uint8)
    img = img.reshape(msg.height, msg.width, 3)
    return img[:, :, ::-1] if msg.encoding == 'rgb8' else img.copy()


class Panes(Node):

    def __init__(self):
        super().__init__('view_cameras')
        self.latest = {}
        for name in CAMERAS:
            self.create_subscription(
                Image, f'/camera/{name}/image_raw',
                (lambda n: (lambda m: self._on(n, m)))(name), QOS)

    def _on(self, name, msg):
        self.latest[name] = msg


def labelled(img, text):
    out = img.copy()
    cv2.rectangle(out, (0, 0), (260, 34), (0, 0, 0), -1)
    cv2.putText(out, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (255, 255, 255), 2, cv2.LINE_AA)
    return out


def blank(label):
    img = np.zeros((PANE_H, PANE_W, 3), dtype=np.uint8)
    cv2.putText(img, f'no image: {label}', (40, PANE_H // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (128, 128, 128), 2)
    return img


def main():
    rclpy.init()
    node = Panes()
    win = 'bartender cameras (q to quit)'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            panes = {}
            for name in CAMERAS:
                if name in node.latest:
                    panes[name] = labelled(to_bgr(node.latest[name]),
                                            name.upper())
                else:
                    panes[name] = blank(name)
            top = np.hstack([panes['overhead'], panes['side']])
            pad = (2 * PANE_W - PANE_W) // 2
            bottom = cv2.copyMakeBorder(
                panes['wrist'], top=0, bottom=0, left=pad, right=pad,
                borderType=cv2.BORDER_CONSTANT, value=(0, 0, 0))
            cv2.imshow(win, np.vstack([top, bottom]))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

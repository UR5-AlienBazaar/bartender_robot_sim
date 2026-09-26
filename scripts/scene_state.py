#!/usr/bin/env python3
"""Read the current poses of the bartender scene's objects.

This is the environment's ground-truth query: it reads SceneBroadcaster's
report of where every non-static model actually is (the same stream the
bar's skills use), and prints the world-frame pose of each named entity.

    python3 scripts/scene_state.py                 # once, then exit
    python3 scripts/scene_state.py --watch         # stream to the terminal

The names are the contract from bartender_scene.yaml: bottle_1..bottle_5,
glass. Entities that do not exist are reported as missing rather than
silently skipped, because a reset that failed to create an entity must be
loud about it.

The node never spins itself; the caller does (a background thread here,
the reset node's thread there) -- nested spin calls are the deadlock this
project already paid for once, see PLAN.md.
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

POSE_TOPIC = '/world/workcell_world/dynamic_pose/info'
ENTITIES = ['bottle_1', 'bottle_2', 'bottle_3', 'bottle_4', 'bottle_5',
            'glass']

# SceneBroadcaster publishes best-effort at 60 Hz.
QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class SceneState(Node):

    def __init__(self, timeout=5.0, node_name='scene_state'):
        super().__init__(node_name)
        self._poses = {}
        self._timeout = timeout
        self._lock = threading.Lock()
        self._spin_thread = None
        self._sub = self.create_subscription(
            TFMessage, POSE_TOPIC, self._on_pose, QOS)

    def _on_pose(self, msg):
        for tf in msg.transforms:
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with self._lock:
                self._poses[tf.child_frame_id] = (
                    (t.x, t.y, t.z), (q.x, q.y, q.z, q.w), yaw)

    def spin_in_background(self):
        """Serve this node's subscriptions from a daemon thread."""
        self._spin_thread = threading.Thread(
            target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        except Exception:                                 # noqa: BLE001
            # shutdown() mid-spin is normal at exit, not an error.
            pass

    def join_background_spin(self, timeout=5.0):
        """Stop the background spin thread; call after rclpy.shutdown()."""
        if self._spin_thread is not None:
            self._spin_thread.join(timeout)

    def wait_for(self, names):
        """Block until every named entity has been seen, or timeout.

        Needs someone spinning the node; does not spin it itself.
        """
        deadline = time.monotonic() + self._timeout
        while rclpy.ok():
            with self._lock:
                missing = [n for n in names if n not in self._poses]
            if not missing:
                return []
            if time.monotonic() > deadline:
                return missing
            time.sleep(0.05)

    def clear(self):
        with self._lock:
            self._poses.clear()

    def get_entity_pose(self, name):
        """(x, y, z), (qx, qy, qz, qw), yaw -- or None if never seen."""
        with self._lock:
            return self._poses.get(name)


def fmt_pose(entry):
    (x, y, z), quat, yaw = entry
    return (f'x={x:+.4f} y={y:+.4f} z={z:+.4f} '
            f'qx={quat[0]:+.4f} qy={quat[1]:+.4f} '
            f'qz={quat[2]:+.4f} qw={quat[3]:+.4f} yaw={yaw:+.4f}')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--watch', action='store_true',
                        help='Keep printing poses as they arrive.')
    parser.add_argument('--timeout', type=float, default=5.0,
                        help='Seconds to wait for the pose stream.')
    args = parser.parse_args()

    rclpy.init()
    state = SceneState(timeout=args.timeout)
    state.spin_in_background()
    missing = state.wait_for(ENTITIES)
    if missing:
        print(f'missing entities: {", ".join(missing)}', file=sys.stderr)
    for name in ENTITIES:
        entry = state.get_entity_pose(name)
        if entry is None:
            print(f'{name:10s} MISSING')
        else:
            print(f'{name:10s} {fmt_pose(entry)}')
    if not args.watch:
        rclpy.shutdown()
        state.join_background_spin()
        return 1 if missing else 0

    try:
        while rclpy.ok():
            time.sleep(0.5)
            print('\033[2J\033[H', end='')
            for name in ENTITIES:
                entry = state.get_entity_pose(name)
                print(f'{name:10s} '
                      f'{fmt_pose(entry) if entry else "MISSING"}')
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    state.join_background_spin()
    return 0


if __name__ == '__main__':
    sys.exit(main())

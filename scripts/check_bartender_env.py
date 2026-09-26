#!/usr/bin/env python3
"""Readiness check for the bartender VLA environment.

Checks, in order, and exits nonzero naming the first missing thing:

  - /joint_states is publishing (the arm is alive)
  - ur_arm_controller is active on /controller_manager
  - all three camera streams publish images and camera_info
  - the six scene entities (bottle_1..bottle_5, glass) report poses

    python3 scripts/check_bartender_env.py

Intended for the top of automated jobs: episode recording should not start
against a half-up stack.
"""
import os
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scene_state import ENTITIES, SceneState          # noqa: E402

CAMERAS = ['overhead', 'side', 'wrist']
QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)


def wait_for(topic_type, topic, node, timeout):
    """True if at least one message arrives on `topic` within `timeout`."""
    got = []

    def cb(msg):
        got.append(msg)

    sub = node.create_subscription(topic_type, topic, cb, QOS)
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not got and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return bool(got)


def check_topics(node):
    checks = [('joint states publishing',
               wait_for(JointState, '/joint_states', node, 3.0))]
    for name in CAMERAS:
        checks.append((f'{name} image',
                       wait_for(Image, f'/camera/{name}/image_raw', node, 5.0)))
        checks.append((f'{name} camera_info',
                       wait_for(CameraInfo, f'/camera/{name}/camera_info',
                                node, 5.0)))
    return checks


def check_controller(node):
    cm = node.create_client(
        ListControllers, '/controller_manager/list_controllers')
    if not cm.wait_for_service(timeout_sec=5.0):
        return False
    future = cm.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    result = future.result()
    for c in (result.controller if result else []):
        if c.name == 'ur_arm_controller' and c.state == 'active':
            return True
    return False


def main():
    rclpy.init()
    node = Node('check_bartender_env')

    checks = check_topics(node)
    checks.append(('ur_arm_controller active', check_controller(node)))

    state = SceneState(timeout=5.0)
    state.spin_in_background()
    missing = state.wait_for(ENTITIES)

    rclpy.shutdown()
    state.join_background_spin()

    failures = []
    for label, ok in checks:
        print(f'{"ok  " if ok else "FAIL"} {label}')
        if not ok:
            failures.append(label)
    for entity in ENTITIES:
        ok = entity not in missing
        print(f'{"ok  " if ok else "FAIL"} entity {entity}')
        if not ok:
            failures.append(f'entity {entity}')

    if failures:
        print('\nNOT READY: ' + '; '.join(failures), file=sys.stderr)
        return 1
    print('\nREADY')
    return 0


if __name__ == '__main__':
    sys.exit(main())

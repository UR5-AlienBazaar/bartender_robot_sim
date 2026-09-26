#!/usr/bin/env python3
"""Reset the bartender scene to its deterministic start state.

What a reset does, in order:

  1. drives the arm to its home configuration (the spawn pose, all joints
     zero) through the real trajectory controller, so the arm's motion is
     honest -- the controller's tolerances apply and the move is checked;
  2. recreates bottle_1..bottle_5 and glass at their nominal poses from
     bartender_scene.yaml, via Gazebo's remove + create services. Remove
     and re-create rather than teleport (set_pose): a teleported object
     keeps its velocity component, so a bottle knocked flying mid-episode
     would keep creeping after the reset, and the read-back would have to
     fail it. A freshly created entity has exactly the nominal pose and
     exactly zero velocity -- the deterministic start the episode runner
     needs. Entity names are the contract, so the world looks identical
     after a reset (only the numeric entity ids churn, and nothing queries
     by id);
  3. reads the poses back from SceneBroadcaster and reports the worst
     residual, so "the reset worked" is a measurement and not a boolean.

Run it as a service the environment can call:

    python3 scripts/reset_bartender_scene.py        # serves /reset_bartender_scene

or once from a shell:

    python3 scripts/reset_bartender_scene.py --once

All waits are done on threading.Events set by done-callbacks while a
background thread spins the node -- never nested spins, which deadlock;
this project already paid for that lesson (see PLAN.md).
"""
import argparse
import os
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from scene_state import SceneState

WORLD = 'workcell_world'
SERVICE = '/reset_bartender_scene'
ARM_CONTROLLER = '/ur_arm_controller'
JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

REPO = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir))
SCENE_YAML = os.path.join(
    REPO, 'ros2_ws', 'src', 'bartender_gazebo', 'config',
    'bartender_scene.yaml')

# How far a read-back pose may sit from nominal for the reset to count.
# set_pose is exact; the slack is for the one physics step between the
# teleport and the read, and for a bottle settling a hair on its base.
POSE_TOLERANCE = 0.005


def load_scene_objects():
    """Entity -> (model URI, pose), straight from the scene YAML."""
    import yaml
    with open(SCENE_YAML) as f:
        scene = yaml.safe_load(f)
    objects = {entity: (spec['model'], spec['pose'])
               for entity, spec in scene['objects'].items()}
    return objects, scene['arm_home']


def _run_service(argv):
    """Run one `ign service` call, retrying once; returns (ok, output).

    `ign service` exits 0 even when the call times out, and prints
    nothing at all in that case, so success has to be judged by the
    response text (`data: true`), not the return code.
    """
    for _attempt in (1, 2):
        proc = subprocess.run(argv, capture_output=True, text=True)
        if 'data: true' in proc.stdout.lower():
            return True, proc.stdout
        time.sleep(1.0)
    return False, proc.stdout or proc.stderr


def remove_cmd(entity):
    return ['ign', 'service', '-s', f'/world/{WORLD}/remove',
            '--reqtype', 'ignition.msgs.Entity',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '10000',
            '--req', f'name: "{entity}" type: MODEL']


def create_cmd(model, entity, pose):
    """The `ign service` argv that (re)creates `entity` at `pose`.

    Pose is the YAML's x y z rpy. Every nominal pose in the scene is
    upright (rpy 0 0 0), so the orientation goes in as the identity
    quaternion; the assert keeps an editor from quietly adding a tilted
    reset pose this code would then mis-convert.
    """
    x, y, z, roll, pitch, yaw = pose
    assert (roll, pitch, yaw) == (0.0, 0.0, 0.0), \
        'non-upright reset poses need a real rpy->quat conversion here'
    req = (f'sdf_filename: "model://{model}" name: "{entity}" '
           f'pose {{ position {{ x: {x} y: {y} z: {z} }} '
           f'orientation {{ w: 1 }} }}')
    return ['ign', 'service', '-s', f'/world/{WORLD}/create',
            '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '10000', '--req', req]


def block_on(future, timeout_sec):
    """Wait for a future via a done-callback Event; None on timeout."""
    done = threading.Event()
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(timeout_sec):
        return None
    try:
        return future.result()
    except Exception:                                    # noqa: BLE001
        return None


class ResetScene(SceneState):

    def __init__(self):
        super().__init__(node_name='reset_bartender_scene')
        self._objects, self._arm_home = load_scene_objects()
        self._nominal = {name: (pose[0], pose[1], pose[2])
                         for name, (_m, pose) in self._objects.items()}
        self._service = self.create_service(
            Trigger, SERVICE, self._on_reset)
        self._arm = ActionClient(
            self, FollowJointTrajectory,
            f'{ARM_CONTROLLER}/follow_joint_trajectory')
        self.get_logger().info(
            f'serving {SERVICE}; entities: {", ".join(sorted(self._objects))}')

    def _on_reset(self, request, response):
        ok, message = self.reset()
        response.success = ok
        response.message = message
        return response

    def _home_the_arm(self, timeout=90.0):
        """Drive the arm to its home joints; returns (ok, why).

        The timeout is generous because the controller runs on sim time:
        at a real-time factor of ~0.3 (three CPU-rendered cameras) a 2 s
        trajectory takes ~7 s of wall clock, with variance.
        """
        if not self._arm.wait_for_server(timeout_sec=timeout):
            return False, f'{ARM_CONTROLLER} action server never appeared'
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in self._arm_home]
        pt.time_from_start.sec = 2
        goal.trajectory.points = [pt]
        handle = block_on(self._arm.send_goal_async(goal), timeout_sec=30.0)
        if handle is None:
            return False, 'arm goal was not answered in time'
        if not handle.accepted:
            return False, 'arm goal refused by the controller'
        result = block_on(handle.get_result_async(), timeout_sec=timeout)
        if result is None:
            return False, 'arm did not reach home in time'
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            return False, (f'arm home move failed, error code '
                          f'{result.result.error_code}')
        return True, 'arm at home'

    def _recreate_objects(self):
        """remove + create every object; returns list of failures."""
        failures = []
        for entity, (model, pose) in sorted(self._objects.items()):
            ok, out = _run_service(remove_cmd(entity))
            if not ok:
                failures.append(f'{entity}: remove failed ({out.strip()!r})')
                continue
            # Removal is processed by the server on its own iteration; give
            # it a moment or the create collides with the dying name.
            time.sleep(0.5)
            ok, out = _run_service(create_cmd(model, entity, pose))
            if not ok:
                failures.append(f'{entity}: create failed ({out.strip()!r})')
        return failures

    def _read_back(self):
        """Worst residual between read-back pose and nominal, in metres."""
        self.clear()
        missing = self.wait_for(list(self._nominal))
        if missing:
            return None, f'missing after reset: {", ".join(missing)}'
        worst, worst_entity = 0.0, None
        for entity, (x, y, z, *_rest) in self._nominal.items():
            entry = self.get_entity_pose(entity)
            (px, py, pz), _quat, _yaw = entry
            residual = ((px - x) ** 2 + (py - y) ** 2 + (pz - z) ** 2) ** 0.5
            if residual > worst:
                worst, worst_entity = residual, entity
        return (worst, worst_entity), None

    def reset(self):
        """Full reset; returns (ok, human-readable report)."""
        arm_ok, arm_why = self._home_the_arm()
        if not arm_ok:
            return False, f'reset failed: {arm_why}'
        failures = self._recreate_objects()
        if failures:
            return False, 'reset failed: ' + '; '.join(failures)
        (worst, worst_entity), problem = self._read_back()
        if problem:
            return False, f'reset failed: {problem}'
        if worst > POSE_TOLERANCE:
            return False, (f'reset failed: {worst_entity} is {worst * 1000:.1f}'
                           f' mm from nominal')
        return True, (f'reset ok: arm home, 6 objects within '
                      f'{worst * 1000:.1f} mm (worst: {worst_entity})')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--once', action='store_true',
                        help='Reset once and exit instead of serving.')
    args = parser.parse_args()

    rclpy.init()
    node = ResetScene()
    node.spin_in_background()
    if args.once:
        ok, message = node.reset()
        print(('OK: ' if ok else 'FAILED: ') + message)
        rclpy.shutdown()
        node.join_background_spin()
        return 0 if ok else 1
    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    node.join_background_spin()
    return 0


if __name__ == '__main__':
    sys.exit(main())

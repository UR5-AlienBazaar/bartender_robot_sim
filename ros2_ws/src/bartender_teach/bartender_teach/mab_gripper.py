"""The workcell's custom gripper: one MAB MD drive, CAN ID 779, on the Pi.

The Pi runs scripts/mab_arm_node.py (started by run_arm.sh with
`-p "can_ids:=[779]"`). It takes ABSOLUTE motor positions in radians on
/roboarm/joint_command and publishes the motor on /joint_states as
`joint779`.

Everything that already talks about the gripper -- the pendant's `open` and
`close`, a pipeline's `grip` step, the API's /gripper -- speaks in Robotiq
knuckle radians: 0.02 open, 0.8 closed. This gripper only opens and closes:
0.02 opens it, anything above that closes it. So a script taught for the
Robotiq drives it unchanged.

Nothing here is a fixed motor position, because there are none to trust:
the drive's position shifts when it loses power, and the fingers have no
end stop on the OPEN side, so opening to a remembered number could drive
them past their range. Everything is measured from the CLOSED stop instead,
which is physical: `gripper home` closes the empty gripper until it stalls
and stores that position (the reference, in REF_FILE, shared by the pendant
and the API). Then

    open   -> reference + OPEN_TRAVEL   (short of the ~2.4 rad full stroke)
    close  -> reference - CLOSE_PAST    (past the stop, so it always pushes
                                         until the fingers stall: on the
                                         bottle, or on the stop)

Re-home after the Pi or the drive has been off. The drive's maxTorque (3 Nm,
set on the drive) is the grip force. After the stall candletool switches the
drive off, so the fingers stay put but are no longer pushed.

Used automatically when the Pi's node is running (something subscribes to
TOPIC); `--gripper robotiq` or `--gripper mab` forces one or the other.
No ROS in here, so the mapping can be tested without a robot.
"""
import os

import yaml

TOPIC = '/roboarm/joint_command'
JOINT = 'joint779'

# Measured on the real gripper on 2026-09-27: closed stop to fully open is
# about 2.4 rad (0.40 -> 2.8). Higher is more open.
OPEN_TRAVEL = 2.3
FULL_STROKE = 2.4
CLOSE_PAST = 0.2
# Homing aims this far below wherever the gripper is: more than the whole
# stroke, so it reaches the closed stop from anywhere.
HOME_SWEEP = 3.0
# A reading this far below the stored closed stop cannot be real: the
# drive's position has shifted, and the reference is stale.
STALE_MARGIN = 0.15

REF_FILE = os.path.expanduser('~/.ros/bartender_mab_gripper.yaml')

# The Robotiq range the rest of the code uses (see GRIPPER_OPEN_POS and
# GRIPPER_UPPER_LIMIT in teach_points).
KNUCKLE_OPEN = 0.02
KNUCKLE_CLOSED = 0.8

# The motor does not report its position while it is moving, so a command
# simply waits this long for the fingers to get there.
MOVE_WAIT_S = 2.0


def is_open_command(knuckle):
    """Tell the Robotiq open position (True) from any grip (False)."""
    return float(knuckle) <= KNUCKLE_OPEN + 0.01


def motor_from_knuckle(knuckle, ref):
    """Motor radians for a Robotiq knuckle value, from the closed stop `ref`."""
    if is_open_command(knuckle):
        return ref + OPEN_TRAVEL
    return ref - CLOSE_PAST


def knuckle_from_motor(motor, ref):
    """Robotiq-equivalent opening for display, clamped to the knuckle range."""
    fraction = 1.0 - (float(motor) - ref) / OPEN_TRAVEL
    fraction = min(max(fraction, 0.0), 1.0)
    return KNUCKLE_OPEN + fraction * (KNUCKLE_CLOSED - KNUCKLE_OPEN)


def is_stale(motor, ref):
    """Tell whether `motor` reads outside the stroke `ref` allows: re-home."""
    motor = float(motor)
    return (motor < ref - STALE_MARGIN
            or motor > ref + FULL_STROKE + STALE_MARGIN)


def load_ref(path=None):
    """Return the stored closed-stop position, or None if never homed."""
    try:
        with open(path or REF_FILE) as f:
            data = yaml.safe_load(f) or {}
        return float(data['closed_stop'])
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return None


def save_ref(ref, path=None):
    """Store the closed-stop position for the pendant and the API."""
    path = path or REF_FILE
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        yaml.safe_dump({'closed_stop': float(ref)}, f)

"""Tests for the MAB gripper's mapping and reference file, with no ROS.

mab_gripper.py is deliberately ROS-free so it can be tested without a robot;
this file is what makes that claim true. The ROS-side command path around it
(_command_mab, homing, the fresh-reading wait) is exercised in
test_teach_points.py, which needs rclpy -- these need nothing.
"""
import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bartender_teach import mab_gripper as mab   # noqa: E402

# A plausible closed-stop reading, from the module's own measured note that
# the stroke runs 0.40 -> 2.8 rad.
REF = 0.4


# -- the constants the mapping is built on -----------------------------------

def test_open_stays_inside_the_stroke_because_there_is_no_open_stop():
    """OPEN_TRAVEL must land short of FULL_STROKE, not at it.

    The fingers have no end stop on the open side, so a command that reaches
    the whole measured stroke is already a command that can run them off.
    """
    assert 0 < mab.OPEN_TRAVEL < mab.FULL_STROKE


def test_homing_sweeps_further_than_the_whole_stroke():
    """HOME_SWEEP aims below wherever the gripper sits; less could stop short."""
    assert mab.HOME_SWEEP > mab.FULL_STROKE


def test_close_pushes_past_the_stop():
    """A close that aimed at the stop exactly might land a hair short of it."""
    assert mab.CLOSE_PAST > 0


def test_the_knuckle_range_is_the_robotiq_one_the_callers_use():
    """0.02/0.8 must match GRIPPER_OPEN_POS/GRIPPER_UPPER_LIMIT in teach_points.

    The point of the mapping is that a script written for the Robotiq drives
    this gripper unchanged; drift in either copy silently re-aims every grip.
    teach_points itself needs rclpy, so this asserts the numbers outright.
    """
    assert mab.KNUCKLE_OPEN == 0.02
    assert mab.KNUCKLE_CLOSED == 0.8


# -- open or close ------------------------------------------------------------

def test_the_open_position_is_an_open_command():
    assert mab.is_open_command(mab.KNUCKLE_OPEN)


@pytest.mark.parametrize('knuckle', [0.05, 0.5, 0.79, 0.8])
def test_any_real_grip_is_a_close_command(knuckle):
    """Scripts grip at 0.5 or 0.8; none of those may read as 'open'."""
    assert not mab.is_open_command(knuckle)


def test_is_open_command_takes_strings():
    """Command parsing hands it floats, but float() is the contract."""
    assert mab.is_open_command('0.02')
    assert not mab.is_open_command('0.5')


def test_open_parks_short_of_the_stroke():
    assert mab.motor_from_knuckle(mab.KNUCKLE_OPEN, REF) == pytest.approx(
        REF + mab.OPEN_TRAVEL)


def test_close_aims_below_the_stop():
    """Past the stop so the fingers always end up pushing: bottle or stall."""
    assert mab.motor_from_knuckle(0.5, REF) == pytest.approx(REF - mab.CLOSE_PAST)


# -- reading the motor back ----------------------------------------------------

def test_fully_open_reads_back_as_open():
    assert mab.knuckle_from_motor(REF + mab.OPEN_TRAVEL, REF) == pytest.approx(
        mab.KNUCKLE_OPEN)


def test_the_closed_stop_reads_back_as_closed():
    assert mab.knuckle_from_motor(REF, REF) == pytest.approx(mab.KNUCKLE_CLOSED)


def test_mid_stroke_reads_back_as_mid_opening():
    mid = REF + mab.OPEN_TRAVEL / 2
    expected = mab.KNUCKLE_OPEN + (mab.KNUCKLE_CLOSED - mab.KNUCKLE_OPEN) / 2
    assert mab.knuckle_from_motor(mid, REF) == pytest.approx(expected)


def test_a_grip_and_the_stop_read_back_as_the_same_display_value():
    """A close command goes CLOSE_PAST below the stop, which clamps to 0.8 --
    so 'held at a grip' and 'parked on the stop' are indistinguishable on
    purpose: this gripper only ever opens or closes.
    """
    gripping = mab.motor_from_knuckle(0.5, REF)
    assert mab.knuckle_from_motor(gripping, REF) == pytest.approx(
        mab.KNUCKLE_CLOSED)


@pytest.mark.parametrize('motor,expected', [
    (REF - 5.0, mab.KNUCKLE_CLOSED),      # driven below the stop
    (REF + 99.0, mab.KNUCKLE_OPEN),       # driven past full stroke
])
def test_readings_outside_the_stroke_clamp_into_the_knuckle_range(motor, expected):
    assert mab.knuckle_from_motor(motor, REF) == pytest.approx(expected)


# -- the stale check ------------------------------------------------------------

@pytest.mark.parametrize('motor', [
    REF,                                  # parked on the stop
    REF + mab.FULL_STROKE,                # fully open
    REF + mab.FULL_STROKE + mab.STALE_MARGIN,   # at the upper edge
    REF - mab.STALE_MARGIN,               # at the lower edge
])
def test_inside_the_stroke_and_its_margins_is_not_stale(motor):
    assert not mab.is_stale(motor, REF)


@pytest.mark.parametrize('motor', [
    REF - mab.STALE_MARGIN - 1e-6,        # a hair below the stop
    REF + mab.FULL_STROKE + mab.STALE_MARGIN + 1e-6,
    REF - 3.0,                            # power loss moved the zero entirely
])
def test_outside_the_stroke_is_stale(motor):
    """A reading the stop cannot produce means the stored reference is wrong."""
    assert mab.is_stale(motor, REF)


def test_nan_is_not_stale_because_the_caller_checks_it_first():
    """is_stale answers 'is this reading impossible', not 'is there a reading'.

    _command_mab and gripper_position guard `motor == motor` before calling,
    so a NaN arriving here must not masquerade as a stale reference -- the
    failure that produces it is a missing reading, not a shifted zero.
    """
    assert not mab.is_stale(float('nan'), REF)


# -- the reference file ---------------------------------------------------------

def test_the_stop_round_trips_through_the_file(tmp_path):
    path = str(tmp_path / 'ref.yaml')
    mab.save_ref(0.413, path)
    assert mab.load_ref(path) == pytest.approx(0.413)


def test_save_creates_missing_directories(tmp_path):
    """First home on a fresh Pi must not fail for want of ~/.ros."""
    path = str(tmp_path / 'deep' / 'nested' / 'ref.yaml')
    mab.save_ref(1.0, path)
    assert mab.load_ref(path) == pytest.approx(1.0)


def test_no_file_means_never_homed(tmp_path):
    assert mab.load_ref(str(tmp_path / 'absent.yaml')) is None


def test_a_corrupt_file_means_never_homed(tmp_path):
    path = tmp_path / 'ref.yaml'
    path.write_text('closed_stop: [unclosed\n')
    assert mab.load_ref(str(path)) is None


@pytest.mark.parametrize('text', [
    'closed_stop: soon\n',      # not a number -> ValueError
    'closed_stop: [1, 2]\n',    # not a scalar -> TypeError
    'other_key: 1.0\n',         # right shape, wrong key -> KeyError
    'just a string\n',          # not a mapping at all
])
def test_a_wrong_shaped_file_means_never_homed(tmp_path, text):
    """Anything unparseable is a missing reference, never a guessed one.

    A bad file reaching the caller as a number would put the open command
    somewhere it cannot measure from; None is what makes the pendant answer
    'run gripper home' instead.
    """
    path = tmp_path / 'ref.yaml'
    path.write_text(text)
    assert mab.load_ref(str(path)) is None


def test_the_file_is_yaml_a_human_can_fix(tmp_path):
    """It is edited by hand on the Pi when debugging; keep it legible."""
    path = tmp_path / 'ref.yaml'
    mab.save_ref(0.413, str(path))
    assert yaml.safe_load(path.read_text()) == {'closed_stop': 0.413}

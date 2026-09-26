"""Tests for the generated bartender scene.

The scene config (config/bartender_scene.yaml) is the source of truth for
where everything stands and where the cameras look; the world SDF and the
debug bottle models are its rendered output. These tests re-render both
and fail on any difference, so the generated files can never drift from
the config the rest of the system reads (the reset script, the launch's
static TF publishers, later randomization).

They also hold the cross-file contracts that would otherwise fail
silently: the entity names in the world, the camera topics, the wrist
camera's placement in the robot description, and the glass dimensions the
pour reward will eventually use.
"""
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.normpath(os.path.join(HERE, os.pardir))
REPO = os.path.normpath(os.path.join(PACKAGE, *([os.pardir] * 3)))
sys.path.insert(0, os.path.join(PACKAGE, 'scripts'))

from make_bartender_scene import (                    # noqa: E402
    DEBUG_BOTTLES, MODELS, load_scene, world_sdf, debug_bottle_sdf)

WORLD = os.path.join(PACKAGE, 'worlds', 'workcell_world.sdf')
DESCRIPTION = os.path.join(
    REPO, 'ros2_ws', 'src', 'bartender_description', 'urdf',
    'workcell.urdf.xacro')
RENDER = os.path.join(
    REPO, 'ros2_ws', 'src', 'bartender_description', 'scripts',
    'render_bartender_urdf.py')
GLASS = os.path.join(REPO, 'models', 'serving_glass', 'model.sdf')


def scene():
    return load_scene()


def test_the_world_matches_the_scene_config():
    with open(WORLD) as f:
        on_disk = f.read()
    assert on_disk == world_sdf(scene()), \
        'worlds/workcell_world.sdf differs from bartender_scene.yaml; ' \
        're-run scripts/make_bartender_scene.py'


def test_the_debug_bottles_match_the_generator():
    for name, profile in DEBUG_BOTTLES.items():
        with open(os.path.join(MODELS, name, 'model.sdf')) as f:
            on_disk = f.read()
        assert on_disk == debug_bottle_sdf(name, profile), \
            f'models/{name}/model.sdf differs from the generator; ' \
            're-run scripts/make_bottle_scene.py'


def test_entities_are_named_by_contract():
    """bottle_1..bottle_5, glass -- the names the environment queries."""
    world = ET.parse(WORLD).getroot()
    names = {include.findtext('name') for include in world.iter('include')}
    assert names == {'bottle_1', 'bottle_2', 'bottle_3', 'bottle_4',
                     'bottle_5', 'glass'}


def test_every_object_pose_is_the_config_pose():
    world = ET.parse(WORLD).getroot()
    sdf_poses = {include.findtext('name'):
                 [float(v) for v in include.findtext('pose').split()]
                 for include in world.iter('include')}
    for entity, spec in scene()['objects'].items():
        assert sdf_poses[entity] == [float(v) for v in spec['pose']]


def test_the_camera_topics_are_the_contract_ones():
    world = ET.parse(WORLD).getroot()
    topics = {sensor.findtext('topic') for sensor in world.iter('sensor')}
    assert topics == {'/camera/overhead/image_raw',
                      '/camera/side/image_raw'}
    fps = scene()['cameras']['overhead']['fps']
    for sensor in world.iter('sensor'):
        assert float(sensor.findtext('update_rate')) == fps
        image = sensor.find('camera/image')
        assert (int(image.findtext('width')),
                int(image.findtext('height'))) == (640, 480)


def test_the_glass_dimensions_in_config_are_the_models():
    """The reward will use config.glass; it must match the physics."""
    glass = ET.parse(GLASS).getroot()
    cylinder = glass.find('.//collision/geometry/cylinder')
    radius = float(cylinder.findtext('radius'))
    length = float(cylinder.findtext('length'))
    assert scene()['glass']['radius'] == radius
    assert scene()['glass']['height'] == length


def test_arm_home_is_the_srdf_home():
    """The episode start must stay the pose MoveIt itself calls home."""
    srdf = ET.parse(os.path.join(
        REPO, 'ros2_ws', 'src', 'bartender_moveit_config', 'srdf',
        'workcell.srdf')).getroot()
    state = next(s for s in srdf.iter('group_state')
                 if s.get('name') == 'home')
    order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    by_name = {j.get('name'): float(j.get('value')) for j in state}
    assert [by_name[n] for n in order] == \
        [float(v) for v in scene()['arm_home']]


def test_the_wrist_camera_in_the_description_matches_the_config():
    """The URDF is where the wrist sensor actually lives; it must agree."""
    import subprocess
    result = subprocess.run(
        [sys.executable, RENDER, DESCRIPTION, 'sim_ignition:=true'],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    robot = ET.fromstring(result.stdout)

    links = {link.get('name') for link in robot.findall('link')}
    assert 'wrist_camera_link' in links
    assert 'wrist_camera_optical_frame' in links

    sensor = None
    for block in robot.findall('gazebo'):
        found = block.find("sensor[@name='wrist_camera']")
        if found is not None:
            sensor, reference = found, block.get('reference')
            break
    assert sensor is not None, 'no wrist_camera sensor in the description'
    assert reference == 'wrist_camera_link'
    assert sensor.findtext('topic') == '/camera/wrist/image_raw'
    assert float(sensor.findtext('update_rate')) == \
        scene()['cameras']['wrist']['fps']
    assert sensor.findtext('camera/image/width') == '640'
    assert sensor.findtext('camera/image/height') == '480'
    assert sensor.findtext('camera/image/format') == 'R8G8B8'

    # The mount: tool0 -> wrist_camera_link at the configured offset.
    joint = next(j for j in robot.findall('joint')
                 if j.find('child').get('link') == 'wrist_camera_link')
    assert joint.find('parent').get('link') == 'tool0'
    xyz = [float(v) for v in joint.find('origin').get('xyz').split()]
    rpy = [float(v) for v in joint.find('origin').get('rpy').split()]
    config = scene()['cameras']['wrist']
    assert xyz == [float(v) for v in config['relative_pose'][:3]]
    assert rpy == [float(v) for v in config['relative_pose'][3:]]

    # The optical frame: standard ROS convention via rpy(-pi/2, 0, -pi/2).
    optical = next(j for j in robot.findall('joint')
                   if j.find('child').get('link') ==
                   'wrist_camera_optical_frame')
    rpy = [float(v) for v in optical.find('origin').get('rpy').split()]
    assert rpy == [-1.5708, 0.0, -1.5708]

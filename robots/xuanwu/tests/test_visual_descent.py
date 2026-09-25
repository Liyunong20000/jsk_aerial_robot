"""Opt-in descent, unique-frame recovery and one-shot standard landing handoff."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav
from test_visual_landing import env, started, advance, visual, odom, reference, command
from xuanwu import visual_landing_controller as c


def frame(env, node, **kwargs):
    advance(env)
    node._odom_callback(odom(env, x=node.xy_ref[0], y=node.xy_ref[1],
                             z=node.z_ref, yaw=node.yaw_ref))
    node._visual_callback(visual(env, **kwargs))


def descending(env):
    env.params.update({'~descend': True, '~required_frames': 3})
    node = started(env, z=2)
    for _ in range(3):
        frame(env, node, z=1)
    assert node.state == c.DESCENDING
    return node


def test_public_enum_and_bool_default(env):
    node = started(env)
    assert not node.descend
    assert node.land_trigger_height == pytest.approx(0.30)
    assert (c.IDLE, c.ALIGNING, c.ALIGNED, c.VISION_LOST, c.ABORTED, c.DESCENDING) == tuple(range(6))
    assert not hasattr(node, 'visual_abort_timeout')


@pytest.mark.parametrize('value,expected', [(False, False), (True, True), ('false', False), ('true', True)])
def test_bool_loading(env, value, expected):
    env.params['~descend'] = value
    assert c.VisualLandingController().descend is expected


@pytest.mark.parametrize('params', [
    {'~descend': 'align_then_land'}, {'~descend': 1}, {'~descent_rate': 0},
    {'~descent_rate': float('nan')}, {'~max_z_ref_lead': -1},
    {'~land_trigger_height': 0}, {'~landing_required_frames': 0},
    {'~landing_required_frames': 1.5},
])
def test_invalid_descent_config(env, params):
    env.params.update(params)
    with pytest.raises(ValueError):
        c.VisualLandingController()
    assert not env.publishers


def test_alignment_only_at_low_visual_height_never_lands(env):
    node = started(env, z=2)
    for _ in range(30):
        frame(env, node, z=0.01)
        node._control()
        assert node.z_ref == 2 and command(node).target_vel_z == 0
        assert command(node).pos_z_nav_mode == FlightNav.POS_MODE
    assert node.state == c.ALIGNED
    node.land_publisher.publish.assert_not_called()


def test_no_descent_before_confirmation_then_same_xy_yaw_servo(env):
    env.params.update({'~descend': True, '~required_frames': 3})
    node = started(env, z=2)
    for _ in range(2):
        frame(env, node, x=0.01, y=-0.01, yaw=0.01, z=1)
        node._control()
        assert node.state == c.ALIGNING and node.z_ref == 2
        assert command(node).target_vel_z == 0
    frame(env, node, x=0.01, y=-0.01, yaw=0.01, z=1)
    assert node.state == c.DESCENDING
    node._control()
    assert node.z_ref == pytest.approx(2 - node.descent_rate * .02)
    assert command(node).target_vel_z == pytest.approx(-node.descent_rate)
    assert command(node).pos_z_nav_mode == FlightNav.POS_VEL_MODE
    assert node.velocity[0] < 0 and node.velocity[1] > 0 and node.yaw_rate < 0
    assert c.ALIGNED not in [args[0][0].data for args in node.state_publisher.publish.call_args_list]


def test_z_antiwindup_when_uav_does_not_follow(env):
    node = descending(env)
    node._control()
    for _ in range(300):
        advance(env)
        node._odom_callback(odom(env, z=2))
        node._visual_callback(visual(env, z=1))
        before = node.z_ref
        node._control()
        assert 0 <= 2-node.z_ref <= node.max_z_ref_lead + 1e-12
        assert node.z_ref <= before
        assert command(node).target_vel_z == pytest.approx((node.z_ref-before)/.02)
    assert node.z_ref == pytest.approx(1.95)
    assert node.vertical_velocity == pytest.approx(0)
    assert node.state == c.DESCENDING


@pytest.mark.parametrize('bad', [{'x': .051}, {'yaw': .081}])
def test_alignment_loss_freezes_z_and_requires_full_reconfirmation(env, bad):
    node = descending(env)
    node._control()
    assert node.vertical_velocity < 0
    held = node.z_ref
    frame(env, node, z=.1, **bad)
    assert node.state == c.ALIGNING and node.vertical_velocity == 0
    assert node.alignment_count == node.landing_count == 0
    node._control()
    assert node.z_ref == held and command(node).target_vel_z == 0
    node.land_publisher.publish.assert_not_called()
    for _ in range(2):
        frame(env, node, z=1)
        node._control()
        assert node.state == c.ALIGNING and node.z_ref == held
    frame(env, node, z=1)
    assert node.state == c.DESCENDING
    node._control()
    assert node.z_ref == pytest.approx(held - node.descent_rate * .02)


@pytest.mark.parametrize('before_timer', [False, True])
def test_visual_loss_freezes_all_refs_indefinitely_then_realigns(env, before_timer):
    node = descending(env)
    frame(env, node, x=.02, yaw=.02, z=.1)
    node._control()
    assert node.velocity.any() and node.yaw_rate != 0 and node.vertical_velocity < 0
    frozen = reference(node).copy()
    duplicate = visual(env, x=.02, z=.1)
    for gap in (.26, 1, 5, 60):
        advance(env, gap)
        node._odom_callback(odom(env, x=node.xy_ref[0], y=node.xy_ref[1], z=node.z_ref))
        node._visual_callback(duplicate)
        node._control()
        assert node.state == c.VISION_LOST
        np.testing.assert_array_equal(reference(node), frozen)
        assert not node.velocity.any() and node.yaw_rate == node.vertical_velocity == 0
        assert command(node).target_vel_z == 0
    assert node.landing_count == node.alignment_count == 0
    advance(env)
    if before_timer:
        node._control()
    node._visual_callback(visual(env, z=1))
    assert node.state == c.ALIGNING and node.alignment_count == 1
    node._control()
    assert node.z_ref == frozen[2]
    frame(env, node, z=1)
    assert node.state == c.ALIGNING
    frame(env, node, z=1)
    assert node.state == c.DESCENDING
    node._control()
    assert node.z_ref < frozen[2]


def test_first_recovery_frame_never_directly_resumes_descent(env):
    node = descending(env)
    node.required_frames = 1
    advance(env, .3)
    node._odom_callback(odom(env, z=node.z_ref))
    node._visual_callback(visual(env, z=1))  # Arrives before timer sees the outage.
    assert node.state == c.ALIGNING
    frame(env, node, z=1)
    assert node.state == c.DESCENDING


def test_land_requires_consecutive_unique_frames_and_relinquishes_nav(env):
    node = descending(env)
    node._control()
    frame(env, node, z=.29)
    assert node.landing_count == 1
    duplicate = visual(env, z=.29)
    for _ in range(20):
        node._visual_callback(duplicate)
    assert node.landing_count == 1
    node.land_publisher.publish.assert_not_called()
    frame(env, node, z=.31)
    assert node.landing_count == 0
    # World altitude remains ~2m: only FINAL visual relative_z decides handoff.
    frame(env, node, z=.30)
    frame(env, node, z=.30)
    assert not node.land_command_sent
    frozen = reference(node).copy()
    def assert_stopped(_):
        assert not node.active and node.land_command_sent
        assert not node.velocity.any() and node.vertical_velocity == node.yaw_rate == 0
        np.testing.assert_array_equal(reference(node), frozen)
    node.land_publisher.publish.side_effect = assert_stopped
    frame(env, node, z=.30)
    node.land_publisher.publish.assert_called_once()
    nav_count = node.nav_publisher.publish.call_count
    for _ in range(10):
        frame(env, node, z=.1)
        node._trigger_callback(Empty())
        node._control()
    node.land_publisher.publish.assert_called_once()
    assert node.nav_publisher.publish.call_count == nav_count
    np.testing.assert_array_equal(reference(node), frozen)


def test_world_z_below_threshold_does_not_land(env):
    node = descending(env)
    for _ in range(4):
        advance(env)
        node._odom_callback(odom(env, z=.1))
        node._visual_callback(visual(env, z=node.land_trigger_height + .1))
    node.land_publisher.publish.assert_not_called()


@pytest.mark.parametrize('fault', ['stale', 'xy_jump', 'z_jump'])
def test_hard_local_failure_still_latches_aborted(env, fault):
    node = descending(env)
    node._control()
    frozen = reference(node).copy()
    advance(env, .6 if fault == 'stale' else .02)
    if fault != 'stale':
        node._odom_callback(odom(env, x=1 if fault == 'xy_jump' else 0,
                                 z=3 if fault == 'z_jump' else node.z_ref))
    node._control()
    assert node.state == c.ABORTED
    assert node.vertical_velocity == 0
    np.testing.assert_array_equal(reference(node), frozen)
    frame(env, node, z=1)
    node._control()
    assert node.state == c.ABORTED
    node.land_publisher.publish.assert_not_called()
    node._trigger_callback(Empty())
    assert node.state == c.ALIGNING


def test_launch_bool_argument_chain():
    root = Path(__file__).resolve().parents[1]
    for filename in ('bringup.launch','visual_landing_gazebo.launch','visual_landing_controller.launch'):
        launch = ET.parse(root/'launch'/filename).getroot()
        assert launch.find("arg[@name='descend']").get('default') == 'false'
        if filename == 'visual_landing_controller.launch':
            param = launch.find("node/param[@name='descend']")
            assert param.get('type')=='bool' and param.get('value')=='$(arg descend)'
        else:
            target = 'bringup.launch' if filename.startswith('visual_landing_gazebo') else 'visual_landing_controller.launch'
            include = next(e for e in launch.findall('include') if e.get('file').endswith('/'+target))
            assert include.find("arg[@name='descend']").get('value')=='$(arg descend)'

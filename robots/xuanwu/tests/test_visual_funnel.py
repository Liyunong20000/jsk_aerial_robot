"""Two-stage descent envelope and independent final landing evidence."""
import pytest
import yaml
from pathlib import Path
from test_visual_landing import env, started, advance, visual, odom, command
from test_visual_descent import frame
from xuanwu import visual_landing_controller as c


def funnel(env):
    env.params.update({'~descend': True, '~descent_funnel': True,
                       '~land_trigger_height': .15})
    return started(env, z=2)


@pytest.mark.parametrize('xy,descending', [(.099, True), (.1, False), (.101, False)])
def test_high_descent_uses_loose_strict_xy_radius(env, xy, descending):
    node = funnel(env)
    frame(env, node, x=xy, z=.5)
    node._control()
    assert (node.state == c.DESCENDING) == descending
    assert (node.vertical_velocity < 0) == descending
    assert node.velocity[0] < 0
    assert node.landing_count == 0


def test_radius_is_xy_norm_not_per_axis(env):
    node = funnel(env)
    frame(env, node, x=.08, y=.08, z=.5)
    node._control()
    assert node.state == c.ALIGNING and node.z_ref == 2


def test_leaving_envelope_stops_z_and_reentry_resumes(env):
    node = funnel(env)
    frame(env, node, x=.09, z=.5)
    node._control()
    held = node.z_ref
    frame(env, node, x=.11, z=.5)
    node._control()
    assert node.z_ref == held and command(node).target_vel_z == 0
    frame(env, node, x=.09, z=.5)
    node._control()
    assert node.z_ref < held


@pytest.mark.parametrize('z', [.15, .14])
@pytest.mark.parametrize('xy', [.04, .08])
def test_low_height_holds_z_until_final_alignment(env, z, xy):
    node = funnel(env)
    for _ in range(5):
        frame(env, node, x=xy, z=z)
        node._control()
        assert node.z_ref == 2 and command(node).target_vel_z == 0
    assert node.landing_count == 0
    node.land_publisher.publish.assert_not_called()


def test_three_unique_consecutive_final_frames_and_one_shot_handoff(env):
    node = funnel(env)
    frame(env, node, x=.039, z=.15)
    duplicate = visual(env, x=.039, z=.15)
    for _ in range(20):
        node._visual_callback(duplicate)
        node._control()
    assert node.landing_count == 1
    frame(env, node, x=.04, z=.15)
    assert node.landing_count == 0
    for _ in range(2):
        frame(env, node, x=.039, z=.15)
        node._control()
        assert node.active and not node.land_command_sent
        assert node.vertical_velocity == 0
    frame(env, node, x=.039, z=.15)
    assert not node.active and node.land_command_sent
    node.land_publisher.publish.assert_called_once()
    count = node.nav_publisher.publish.call_count
    frame(env, node, x=0, z=.1)
    node._control()
    assert node.nav_publisher.publish.call_count == count
    node.land_publisher.publish.assert_called_once()


def test_high_frame_resets_final_evidence(env):
    node = funnel(env)
    frame(env, node, z=.15)
    frame(env, node, z=.151)
    assert node.landing_count == 0


def test_yaw_gate_blocks_descent_and_resets_final_evidence(env):
    node = funnel(env)
    frame(env, node, z=.5, yaw=.101)
    node._control()
    assert node.state == c.ALIGNING and node.z_ref == 2
    frame(env, node, z=.15)
    assert node.landing_count == 1
    frame(env, node, z=.15, yaw=.081)
    assert node.landing_count == 0


def test_vision_outage_holds_then_first_recovery_frame_does_not_descend(env):
    node = funnel(env)
    frame(env, node, z=.5)
    node._control()
    held = node.z_ref
    advance(env, .3)
    node._odom_callback(odom(env, z=held))
    node._visual_callback(visual(env, z=.5))
    node._control()
    assert node.state == c.ALIGNING and node.z_ref == held
    frame(env, node, z=.5)
    node._control()
    assert node.state == c.DESCENDING and node.z_ref < held


def test_vision_outage_resets_final_count(env):
    node = funnel(env)
    frame(env, node, z=.15)
    frame(env, node, z=.15)
    advance(env, .3)
    node._odom_callback(odom(env, z=2))
    node._visual_callback(visual(env, z=.15))
    assert node.landing_count == 1 and not node.land_command_sent


def test_timer_cannot_integrate_below_observed_handoff_plane(env):
    node = funnel(env)
    frame(env, node, z=.1501)
    floor = 2 - .0001
    for _ in range(5):
        advance(env)
        node._odom_callback(odom(env, z=node.z_ref))
        node._control()
        assert node.z_ref >= floor - 1e-12
    assert node.z_ref == pytest.approx(floor)
    assert node.vertical_velocity == pytest.approx(0)


@pytest.mark.parametrize('params', [
    {'~descent_funnel': 1}, {'~descent_xy_distance': 0},
    {'~landing_xy_distance': float('nan')},
    {'~landing_xy_distance': .11},
    {'~descent_yaw_threshold': 0}, {'~landing_yaw_threshold': float('nan')},
    {'~landing_yaw_threshold': .11}, {'~descent_yaw_threshold': 3.2},
])
def test_invalid_envelope_config(env, params):
    env.params.update(params)
    with pytest.raises(ValueError):
        c.VisualLandingController()


def test_hardware_config_enables_requested_policy(env):
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root/'config/visual_landing.yaml').read_text())['visual_landing']
    env.params.update({'~visual_landing/' + k: v for k, v in config.items()})
    node = c.VisualLandingController()
    assert node.descent_funnel
    assert (node.descent_yaw_threshold, node.landing_yaw_threshold) == (.1, .05)
    assert (node.descent_xy_distance, node.landing_xy_distance,
            node.land_trigger_height, node.landing_required_frames) == (.1, .04, .15, 3)


@pytest.mark.parametrize('yaw,allowed', [(.099, True), (-.099, True), (.101, False), (-.101, False)])
def test_descent_yaw_has_its_own_looser_gate(env, yaw, allowed):
    node = funnel(env)
    frame(env, node, x=.09, z=.5, yaw=yaw)
    node._control()
    assert (node.state == c.DESCENDING) == allowed
    assert (node.vertical_velocity < 0) == allowed


def test_yaw_tightens_at_handoff_height_without_hysteresis_carryover(env):
    node = funnel(env)
    frame(env, node, z=.5, yaw=.07)
    assert node.state == c.DESCENDING
    frame(env, node, z=.15, yaw=.07)
    node._control()
    assert node.landing_count == 0 and node.vertical_velocity == 0
    frame(env, node, z=.15, yaw=.049)
    assert node.landing_count == 1
    frame(env, node, z=.15, yaw=.051)
    assert node.landing_count == 0
    for _ in range(3):
        frame(env, node, z=.15, yaw=-.049)
    node.land_publisher.publish.assert_called_once()


@pytest.mark.parametrize('z,threshold', [(.5, .1), (.15, .05)])
def test_exact_yaw_threshold_is_excluded(env, z, threshold):
    node = funnel(env)
    # Test the gate directly: quaternion conversions may round a boundary angle.
    import numpy as np
    node._funnel_frame(np.array([0., 0., z]), 0., threshold, False)
    assert not node.yaw_aligned and node.state == c.ALIGNING
    assert node.landing_count == 0


def test_yaw_wrap_uses_shortest_relative_error(env):
    import math
    env.params['~desired_relative_yaw'] = math.pi - .01
    node = funnel(env)
    frame(env, node, z=.15, yaw=-math.pi + .01)
    assert node.landing_count == 1


def test_independent_yaw_configuration(env):
    env.params.update({'~descent_yaw_threshold': .2, '~landing_yaw_threshold': .03})
    node = funnel(env)
    frame(env, node, z=.5, yaw=.15)
    assert node.state == c.DESCENDING
    frame(env, node, z=.15, yaw=.04)
    assert node.landing_count == 0

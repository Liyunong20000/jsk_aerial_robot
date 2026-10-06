"""Explicit visual-session cancellation using real ROS messages and mocked ROS I/O."""

import numpy as np
import pytest
import rospy
from aerial_robot_msgs.msg import FlightNav
from std_msgs.msg import Empty

from test_visual_landing import advance, command, env, moving, odom, reference, started, visual
from test_visual_descent import descending, frame
from xuanwu import visual_landing_controller as c


@pytest.mark.parametrize('robot_ns', ['xuanwu', 'test_quad'])
def test_cancel_has_its_own_empty_subscriber(env, robot_ns):
    env.params['~robot_ns'] = robot_ns
    node = c.VisualLandingController()
    subscribers = {args[0]: (args[1], args[2])
                   for args, _ in rospy.Subscriber.call_args_list}
    assert subscribers[f'/{robot_ns}/visual_landing/trigger'] == (Empty, node._trigger_callback)
    assert subscribers[f'/{robot_ns}/visual_landing/cancel'] == (Empty, node._cancel_callback)


def test_active_cancel_reanchors_and_publishes_one_non_descending_hold(env):
    node = moving(env)
    advance(env)
    node._visual_callback(visual(env, x=.02, y=.01, yaw=.01))
    advance(env)
    node._odom_callback(odom(env, x=1.0, y=-.7, z=2.6, yaw=-.4))
    assert not np.allclose(node.xy_ref, [1.0, -.7])
    old_stamp = node.last_visual_stamp
    node.nav_publisher.publish.reset_mock()

    def check_at_publish(msg):
        assert node.state == c.IDLE
        assert not node.velocity.any()
        assert node.vertical_velocity == node.yaw_rate == 0
        assert (msg.target_vel_x, msg.target_vel_y, msg.target_vel_z,
                msg.target_omega_z) == (0, 0, 0, 0)

    node.nav_publisher.publish.side_effect = check_at_publish
    node._cancel_callback(Empty())
    assert node.nav_publisher.publish.call_count == 1
    assert node.state == c.IDLE and not node.active
    np.testing.assert_allclose(reference(node), [1.0, -.7, 2.6, -.4])
    assert node.alignment_count == node.landing_count == 0
    assert not node.xy_aligned and not node.yaw_aligned
    assert node.visual is None and node.visual_received is None
    assert not node.tracking_started and node.trigger_time is None
    assert node.last_visual_stamp == old_stamp and not node.land_command_sent
    msg = command(node)
    assert msg.control_frame == FlightNav.WORLD_FRAME and msg.target == FlightNav.COG
    assert msg.pos_xy_nav_mode == msg.yaw_nav_mode == FlightNav.POS_VEL_MODE
    assert msg.pos_z_nav_mode == FlightNav.POS_MODE
    assert (msg.target_pos_x, msg.target_pos_y, msg.target_pos_z,
            msg.target_yaw) == pytest.approx((1.0, -.7, 2.6, -.4))
    assert msg.target_acc_x == msg.target_acc_y == msg.target_pos_diff_z == 0
    for _ in range(20):
        advance(env)
        node._control()
    node._cancel_callback(Empty())
    assert node.nav_publisher.publish.call_count == 1


@pytest.mark.parametrize('case', ['receipt', 'stamp', 'invalid'])
def test_stale_or_invalid_odom_cancel_freezes_existing_refs(env, case):
    node = moving(env)
    frozen = reference(node).copy()
    if case == 'receipt':
        env.clock.wall += node.odom_timeout + .1
    elif case == 'stamp':
        env.clock.ros += node.odom_timeout + .1
    else:
        advance(env, node.odom_timeout + .1)
        node._odom_callback(odom(env, x=float('nan')))
    node.nav_publisher.publish.reset_mock()
    node._cancel_callback(Empty())
    assert node.state == c.IDLE and not node.active
    np.testing.assert_array_equal(reference(node), frozen)
    msg = command(node)
    assert msg.pos_z_nav_mode == FlightNav.POS_MODE
    assert (msg.target_vel_x, msg.target_vel_y, msg.target_vel_z,
            msg.target_omega_z) == (0, 0, 0, 0)
    assert node.nav_publisher.publish.call_count == 1
    assert rospy.logwarn.called
    node._control()
    assert node.nav_publisher.publish.call_count == 1


@pytest.mark.parametrize('state', [c.ALIGNING, c.ALIGNED, c.VISION_LOST, c.ABORTED])
def test_cancel_returns_other_active_states_to_idle(env, state):
    if state == c.ALIGNED:
        env.params['~required_frames'] = 1
    node = started(env)
    if state == c.ALIGNED:
        node._visual_callback(visual(env))
    elif state in (c.VISION_LOST, c.ABORTED):
        advance(env, .6 if state == c.ABORTED else .3)
        if state == c.VISION_LOST:
            node._odom_callback(odom(env))
        node._control()
    assert node.state == state and node.active
    node.nav_publisher.publish.reset_mock()
    node._cancel_callback(Empty())
    assert node.state == c.IDLE and not node.active
    assert node.nav_publisher.publish.call_count == 1
    node._control()
    assert node.nav_publisher.publish.call_count == 1


def test_descending_cancel_stops_descent_without_land_handoff(env):
    node = descending(env)
    node._control()
    frame(env, node, z=.5)
    node._control()
    assert node.state == c.DESCENDING
    assert node.vertical_velocity < 0 and node.landing_count == 0
    old_z = node.z_ref
    advance(env)
    node._odom_callback(odom(env, x=.2, y=-.2, z=old_z + .03, yaw=.3))
    node.nav_publisher.publish.reset_mock()
    node._cancel_callback(Empty())
    assert node.state == c.IDLE and not node.active
    assert node.vertical_velocity == 0 and node.landing_count == 0
    assert node.z_ref == pytest.approx(old_z + .03)
    assert command(node).pos_z_nav_mode == FlightNav.POS_MODE
    assert command(node).target_vel_z == 0
    assert node.nav_publisher.publish.call_count == 1
    for _ in range(10):
        frame(env, node, z=.1)
        node._control()
    assert node.z_ref == pytest.approx(old_z + .03)
    assert node.nav_publisher.publish.call_count == 1
    node.land_publisher.publish.assert_not_called()
    assert not node.land_command_sent


def test_idle_cancel_is_idempotent_and_does_not_create_reference(env):
    node = c.VisualLandingController()
    for _ in range(3):
        node._cancel_callback(Empty())
        node._control()
    assert node.state == c.IDLE and not node.active
    assert node.xy_ref is node.z_ref is node.yaw_ref is None
    node.nav_publisher.publish.assert_not_called()


def test_cancel_after_standard_land_handoff_does_not_interfere(env):
    node = descending(env)
    for _ in range(30):
        if not node.land_command_sent:
            frame(env, node, z=.1)
    assert node.land_command_sent and not node.active
    assert node.land_publisher.publish.call_count == 1
    nav_count = node.nav_publisher.publish.call_count
    state, frozen = node.state, reference(node).copy()
    rospy.logwarn.reset_mock()
    node._cancel_callback(Empty())
    node._control()
    assert node.land_command_sent and not node.active and node.state == state
    assert node.nav_publisher.publish.call_count == nav_count
    assert node.land_publisher.publish.call_count == 1
    np.testing.assert_array_equal(reference(node), frozen)
    rospy.logwarn.assert_called()


def test_retrigger_after_cancel_requires_fresh_odom_and_rejects_visual_replay(env):
    node = started(env)
    old = visual(env, x=.4)
    node._visual_callback(old)
    assert node.last_visual_stamp == old.header.stamp
    node._cancel_callback(Empty())
    advance(env, 1.0)
    node._trigger_callback(Empty())
    assert node.state == c.IDLE and not node.active
    node._odom_callback(odom(env, x=3, y=4, z=5, yaw=.2))
    node._trigger_callback(Empty())
    assert node.state == c.ALIGNING and node.active
    np.testing.assert_allclose(reference(node), [3, 4, 5, .2])
    assert node.visual_received is None and node.alignment_count == 0
    node._visual_callback(old)
    assert node.visual_received is None and node.alignment_count == 0
    fresh = visual(env, x=.01)
    node._visual_callback(fresh)
    assert node.last_visual_stamp == fresh.header.stamp
    assert node.visual_received == env.clock.wall
    assert node.alignment_count == 1
    assert node.state == c.ALIGNING


def test_cancel_publish_error_still_relinquishes_visual_nav(env):
    node = moving(env)
    node.nav_publisher.publish.side_effect = rospy.ROSException('publisher closed')
    with pytest.raises(rospy.ROSException):
        node._cancel_callback(Empty())
    assert node.state == c.IDLE and not node.active
    node.nav_publisher.publish.reset_mock()
    node._control()
    node.nav_publisher.publish.assert_not_called()

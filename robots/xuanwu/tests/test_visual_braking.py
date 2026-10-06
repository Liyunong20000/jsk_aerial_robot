"""Braking captures once, waits for physical stopping, and reacquires on drift."""
import numpy as np
import pytest
from std_msgs.msg import Empty
from test_visual_landing import env, started, advance, visual, odom, reference, command
from test_visual_descent import frame
from xuanwu import visual_landing_controller as c


def approach(env, funnel=True):
    env.params.update({'~descend': True, '~descent_funnel': funnel,
                       '~required_frames': 1, '~land_trigger_height': .15})
    node=started(env,z=2)
    frame(env,node,z=.5)
    return node


@pytest.mark.parametrize('funnel',[True,False])
def test_capture_removes_old_reference_once_and_excludes_entry_frame(env,funnel):
    node=approach(env,funnel)
    node.xy_ref=np.array([.039,-.02]);node.velocity[:]=[.01,.01]
    advance(env)
    msg=odom(env,x=0,y=0,z=1.99,yaw=.01);msg.twist.twist.linear.x=.067
    node._odom_callback(msg)
    node._visual_callback(visual(env,x=.01,z=.14))
    assert node.state==c.BRAKING and c.BRAKING==6
    np.testing.assert_array_equal(node.xy_ref,[0,0])
    assert node.z_ref==1.99 and node.landing_count==0
    held=reference(node).copy()
    assert (command(node).target_vel_x,command(node).target_vel_y,command(node).target_vel_z)==(0,0,0)
    for _ in range(12):
        advance(env)
        msg=odom(env,x=.005,y=0,z=1.99,yaw=.01);msg.twist.twist.linear.x=.067
        node._odom_callback(msg)
        node._visual_callback(visual(env,x=.015,z=.14))
        node._control()
        np.testing.assert_array_equal(reference(node),held)
        assert node.landing_count==0
    node.land_publisher.publish.assert_not_called()
    # Subsequent stationary odometry and visual evidence permit a single handoff.
    for _ in range(30):
        if node.land_command_sent:break
        advance(env);node._odom_callback(odom(env,x=.005,z=1.99,yaw=.01))
        node._visual_callback(visual(env,x=.015,z=.14));node._control()
    node.land_publisher.publish.assert_called_once()


@pytest.mark.parametrize('bad',[{'x':.05},{'yaw':.06}])
def test_drift_releases_hold_into_slow_alignment_and_reentry_captures_again(env,bad):
    node=approach(env)
    frame(env,node,z=.14)
    assert node.state==c.BRAKING
    advance(env);node._odom_callback(odom(env,x=.03,z=2))
    node._visual_callback(visual(env,z=.14,**bad));node._control()
    assert node.state==c.ALIGNING and node.landing_count==0
    assert node.vertical_velocity==0 and np.linalg.norm(node.velocity_target)<=.02+1e-12
    # Reenter final range at a different physical position: capture once anew.
    advance(env);node._odom_callback(odom(env,x=.03,z=2))
    node._visual_callback(visual(env,x=.01,z=.14))
    assert node.state==c.BRAKING and node.landing_count==0
    np.testing.assert_array_equal(node.xy_ref,[.03,0])
    node.land_publisher.publish.assert_not_called()


@pytest.mark.parametrize('fault',['vision','odom','cancel','retrigger','xy_jump'])
def test_braking_retains_existing_safety_and_cancel_behavior(env,fault):
    node=approach(env);frame(env,node,z=.14);held=reference(node).copy()
    if fault=='vision':
        advance(env,.26);node._odom_callback(odom(env,z=2));node._control()
        assert node.state==c.VISION_LOST
    elif fault=='odom':
        advance(env,.51);node._control();assert node.state==c.ABORTED
    elif fault=='cancel':
        node._cancel_callback(Empty());assert node.state==c.IDLE and not node.active
    elif fault=='retrigger':
        node._trigger_callback(Empty());assert node.state==c.ALIGNING
    else:
        advance(env);node._odom_callback(odom(env,x=.3,z=2));node._control()
        assert node.state==c.ABORTED
        np.testing.assert_array_equal(reference(node),held)
    assert node.landing_settle_since is None and node.landing_count==0
    node.land_publisher.publish.assert_not_called()


def test_hard_xy_fault_cannot_be_hidden_by_brake_entry(env):
    node=approach(env)
    held=reference(node).copy()
    advance(env);node._odom_callback(odom(env,x=.3,z=2))
    node._visual_callback(visual(env,z=.14))
    assert node.state==c.ABORTED
    np.testing.assert_array_equal(reference(node),held)
    node.land_publisher.publish.assert_not_called()


@pytest.mark.parametrize('value',[0.,-1.,float('nan'),float('inf')])
def test_invalid_reacquisition_speed(env,value):
    env.params['~landing_realign_max_xy_vel']=value
    with pytest.raises(ValueError):c.VisualLandingController()


@pytest.mark.parametrize('funnel',[True,False])
def test_low_height_reacquisition_stays_slow_across_multiple_frames(env,funnel):
    node=approach(env,funnel)
    frame(env,node,z=.14)
    for _ in range(10):
        frame(env,node,x=.1,z=.14);node._control()
        assert node.state==c.ALIGNING
        assert np.linalg.norm(node.velocity_target)<=node.landing_realign_max_xy_vel+1e-12
        assert node.vertical_velocity==0
    node.land_publisher.publish.assert_not_called()


def test_legacy_confirmation_at_final_height_cannot_integrate_z_before_braking(env):
    env.params.update({'~descend':True,'~required_frames':1,'~land_trigger_height':.15})
    node=started(env,z=2)
    frame(env,node,z=.14);node._control()
    assert node.z_ref==2 and node.vertical_velocity==0
    frame(env,node,z=.14)
    assert node.state==c.BRAKING

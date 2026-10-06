"""Final land must wait for actual motion/reference convergence, not camera count."""
import numpy as np
import pytest
from test_visual_landing import env, started, advance, visual, odom
from test_visual_descent import frame
from xuanwu import visual_landing_controller as c


def final_node(env, funnel=True):
    env.params.update({'~descend': True, '~descent_funnel': funnel,
                       '~required_frames': 1, '~land_trigger_height': .15})
    node = started(env, z=2)
    frame(env, node, z=.5)
    assert node.state == c.DESCENDING
    frame(env, node, z=.14)
    assert node.state == c.BRAKING and node.landing_count == 0
    return node


def update(env, node, speed=0., ref_error=0., seconds=.02, z=.14):
    advance(env, seconds)
    m = odom(env, x=node.xy_ref[0]-ref_error, y=node.xy_ref[1], z=node.z_ref)
    m.twist.twist.linear.x = speed
    node._odom_callback(m)
    node._visual_callback(visual(env, z=z))
    node._control()


@pytest.mark.parametrize('funnel', [True, False])
@pytest.mark.parametrize('speed,error', [(.067,0.), (0.,.039), (.02,0.),
                                         (0.,.02), (float('nan'),0.),
                                         (float('inf'),0.)])
def test_motion_or_reference_error_blocks_handoff_despite_many_frames(env, funnel, speed, error):
    node = final_node(env, funnel)
    for _ in range(40):
        update(env, node, speed, error)
    node.land_publisher.publish.assert_not_called()
    assert node.landing_count == 0 and node.landing_settle_since is None


@pytest.mark.parametrize('funnel', [True, False])
def test_count_and_dwell_both_required_then_one_shot(env, funnel):
    node = final_node(env, funnel)
    for _ in range(3):
        update(env, node)
    assert node.landing_count == 3 and not node.land_command_sent
    # Duplicates and timer ticks cannot hand off, without a new unique visual observation.
    duplicate = visual(env, z=.14)
    for _ in range(8):
        advance(env,.02)
        node._odom_callback(odom(env,z=2))
        node._visual_callback(duplicate)
        node._control()
    assert node.landing_count == 3
    while not node.land_command_sent:
        update(env,node)
    node.land_publisher.publish.assert_called_once()
    before = node.nav_publisher.publish.call_count
    update(env,node)
    assert node.nav_publisher.publish.call_count == before
    node.land_publisher.publish.assert_called_once()


def test_velocity_excursion_between_visual_frames_restarts_full_dwell(env):
    node=final_node(env)
    for _ in range(20):update(env,node)
    assert node.landing_settle_since is not None
    advance(env,.01)
    m=odom(env,z=2);m.twist.twist.linear.y=.05
    node._odom_callback(m)
    assert node.landing_settle_since is None and node.landing_count==0
    for _ in range(20):update(env,node)
    assert not node.land_command_sent
    for _ in range(10):
        if not node.land_command_sent:update(env,node)
    node.land_publisher.publish.assert_called_once()


@pytest.mark.parametrize('fault', ['vision','height','cancel','trigger','reference'])
def test_outage_or_disqualification_clears_dwell(env,fault):
    from std_msgs.msg import Empty
    node=final_node(env)
    for _ in range(10):update(env,node)
    if fault=='vision':
        advance(env,.26);node._odom_callback(odom(env,z=2));node._control()
    elif fault=='height':update(env,node,z=.5)
    elif fault=='cancel':node._cancel_callback(Empty())
    elif fault=='trigger':node._trigger_callback(Empty())
    else:
        node.xy_ref+=np.array([.03,0.]);node._control()
    assert node.landing_settle_since is None and node.landing_count==0
    node.land_publisher.publish.assert_not_called()


@pytest.mark.parametrize('name', ['landing_max_xy_speed','landing_max_xy_ref_error','landing_settle_duration'])
@pytest.mark.parametrize('value',[0.,-1.,float('nan'),float('inf')])
def test_invalid_settling_parameters(env,name,value):
    env.params['~'+name]=value
    with pytest.raises(ValueError):c.VisualLandingController()


def test_elapsed_dwell_cannot_substitute_for_unique_frame_count(env):
    env.params['~landing_required_frames']=40
    node=final_node(env)
    for _ in range(30):update(env,node)
    assert node.landing_count==30 and not node.land_command_sent
    for _ in range(10):update(env,node)
    node.land_publisher.publish.assert_called_once()

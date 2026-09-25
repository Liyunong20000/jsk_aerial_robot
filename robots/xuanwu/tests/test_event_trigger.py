"""Local bridge startup and retained target/gripper behavior; mocked ROS I/O."""

from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseStamped
from spinal.msg import ServoControlCmd, ServoState, ServoStates
from std_msgs.msg import Empty

from xuanwu import event_trigger as bridge
from test_visual_landing import env


@pytest.fixture
def node(env, monkeypatch):
    for name in ('sleep', 'logdebug', 'logerr'):
        monkeypatch.setattr(bridge.rospy, name, Mock())
    return bridge.EventtriggerNode()


def test_startup_without_module_services_or_navigation_output(env, node):
    topics = {call.args[0] for call in bridge.rospy.Subscriber.call_args_list}
    assert topics == {
        '/xuanwu/servo/states', '/xuanwu/target_pose/info',
        '/xuanwu/target_pose/trigger', '/xuanwu/servo/target_states/info',
        '/xuanwu/servo/target_states/trigger', '/xuanwu/servo/return/trigger',
    }
    assert '/xuanwu/uav/nav' not in env.publishers
    assert '/xuanwu/visual_landing/trigger' not in env.publishers
    for publisher in env.publishers.values():
        publisher.publish.assert_not_called()


def test_target_pose_forwarding_preserves_frame_and_pose(env, node):
    target = PoseStamped()
    target.header.frame_id = 'world'
    target.pose.position.x = 0.4
    target.pose.position.z = 1.2
    target.pose.orientation.w = 1.0
    node._callback_target_pose_info(target)
    node._callback_target_pose_trigger(Empty())
    forwarded = node.pub_drone_target.publish.call_args.args[0]
    assert forwarded.header.frame_id == target.header.frame_id
    assert forwarded.pose == target.pose


def test_gripper_contact_still_stops_closing_and_requires_return(env, node):
    def feedback(angle, load):
        node._callback_servo_states(ServoStates(servos=[
            ServoState(index=0, angle=angle, load=load, error=0)]))

    feedback(900, 0)
    node._callback_servo_target_states_info(ServoControlCmd(index=[0], angles=[650]))
    node._callback_servo_target_states_trigger(Empty())
    assert node.pub_servo_target.publish.call_args.args[0].angles == [650]
    feedback(800, 310)
    feedback(790, 310)
    assert node._servo_contact_latched
    assert node.pub_servo_target.publish.call_args.args[0].angles == [800]
    count = node.pub_servo_target.publish.call_count
    node._callback_servo_target_states_trigger(Empty())
    assert node.pub_servo_target.publish.call_count == count
    node._callback_servo_return_trigger(Empty())
    assert not node._servo_contact_latched
    assert node.pub_servo_target.publish.call_args.args[0].angles == [node.servo_max_angles]

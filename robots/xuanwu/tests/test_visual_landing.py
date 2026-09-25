"""Phase 1 regressions with real messages and mocked ROS I/O; no hardware."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import rospy
import tf.transformations as tft
import yaml
from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, UInt8

from xuanwu import apriltag_relative_pose as estimator
from xuanwu import visual_landing_controller as controller


@pytest.fixture
def env(monkeypatch):
    clock = SimpleNamespace(ros=100.0, wall=100.0)
    params = {
        '/camera_drone_matrix': np.eye(4).ravel().tolist(),
        '/drone_tags_matrix': [
            {'id': i, 'matrix': np.eye(4).ravel().tolist()} for i in (0, 1)],
    }
    publishers = {}

    def publisher(topic, *args, **kwargs):
        publishers[topic] = Mock(msg_type=args[0], options=kwargs)
        return publishers[topic]

    monkeypatch.setattr(rospy, 'get_param', lambda name, default: params.get(name, default))
    monkeypatch.setattr(rospy, 'Publisher', publisher)
    monkeypatch.setattr(rospy, 'Subscriber', Mock())
    monkeypatch.setattr(rospy, 'Timer', Mock())
    monkeypatch.setattr(rospy.Time, 'now', lambda: rospy.Time.from_sec(clock.ros))
    monkeypatch.setattr(controller, 'time', SimpleNamespace(monotonic=lambda: clock.wall))
    for name in ('loginfo', 'logwarn', 'logwarn_throttle'):
        monkeypatch.setattr(rospy, name, Mock())
    return SimpleNamespace(clock=clock, params=params, publishers=publishers)


def fill_pose(pose, x=0.0, y=0.0, z=1.0, yaw=0.0):
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    q = tft.quaternion_from_euler(0, 0, yaw)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q


def tag(tag_id, **kwargs):
    message = AprilTagDetection()
    message.id = [tag_id]
    fill_pose(message.pose.pose.pose, **kwargs)
    return message


def visual(env, **kwargs):
    message = PoseStamped()
    message.header.stamp = rospy.Time.from_sec(env.clock.ros)
    message.header.frame_id = 'qilin_docking_frame'
    fill_pose(message.pose, **kwargs)
    return message


def odom(env, **kwargs):
    message = Odometry()
    message.header.stamp = rospy.Time.from_sec(env.clock.ros)
    message.header.frame_id = '/world'
    fill_pose(message.pose.pose, **kwargs)
    return message


def advance(env, seconds=0.02):
    env.clock.ros += seconds
    env.clock.wall += seconds


def command(node):
    return node.nav_publisher.publish.call_args.args[0]


@pytest.mark.parametrize('ids', [(0,), (1,), (0, 1)])
def test_calibrated_geometry_and_consistent_full_orientation(env, ids):
    # Synthetic calibration retains nonidentity camera rotation and both tag offsets.
    env.params['/camera_drone_matrix'] = [0,-1,0,-.13, 1,0,0,0, 0,0,1,0, 0,0,0,1]
    for entry, offset in zip(env.params['/drone_tags_matrix'], (.065, -.065)):
        entry['matrix'][7] = offset
    node = estimator.AprilTagRelativePose()
    camera_uav = tft.euler_matrix(0.15, -0.1, -0.3)
    camera_uav[:3, 3] = [0.1, -0.4, 1.4]
    detections = []
    for i in ids:
        camera_tag = camera_uav @ np.linalg.inv(node.tag_matrices[i])
        detection = tag(i)
        p = detection.pose.pose.pose
        p.position.x, p.position.y, p.position.z = camera_tag[:3, 3]
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = tft.quaternion_from_matrix(camera_tag)
        detections.append(detection)
    message = AprilTagDetectionArray(detections=detections)
    message.header.stamp = rospy.Time(99, 900000000)
    node._callback(message)
    output = node.publisher.publish.call_args.args[0]
    actual = controller.pose_state(output.pose)
    expected = node.camera_matrix @ camera_uav
    np.testing.assert_allclose(actual[0], expected[:3, 3], atol=1e-12)
    np.testing.assert_allclose(actual[1], expected[:3, :3], atol=1e-12)
    assert output.header.stamp == message.header.stamp
    assert output.header.frame_id == 'qilin_docking_frame'


def test_dual_tag_disagreement_history_and_orientation_average(env):
    node = estimator.AprilTagRelativePose()
    result, _ = node.estimate(AprilTagDetectionArray(detections=[tag(0, x=0), tag(1, x=1)]))
    assert result[0, 3] == 0  # No history: tag 0.
    node.estimate(AprilTagDetectionArray(detections=[tag(1, x=0.9)]))
    result, _ = node.estimate(AprilTagDetectionArray(detections=[tag(0, x=0), tag(1, x=1)]))
    assert result[0, 3] == 1
    result, _ = node.estimate(AprilTagDetectionArray(detections=[
        tag(0, x=0.01, yaw=np.pi - 0.1), tag(1, x=0.03, yaw=-np.pi + 0.1)]))
    assert result[0, 3] == pytest.approx(0.02)
    assert abs(tft.euler_from_matrix(result)[2]) == pytest.approx(np.pi)


def test_missing_invalid_and_timestamp_fallback(env):
    node = estimator.AprilTagRelativePose()
    for detections in ([], [tag(5)], [tag(0, x=float('nan'))]):
        node._callback(AprilTagDetectionArray(detections=detections))
    node.publisher.publish.assert_not_called()
    detection = tag(1)
    detection.pose.header.stamp = rospy.Time(99)
    node._callback(AprilTagDetectionArray(detections=[detection]))
    assert node.publisher.publish.call_args.args[0].header.stamp == rospy.Time(99)
    detection.pose.header.stamp = rospy.Time()
    node._callback(AprilTagDetectionArray(detections=[detection]))
    assert node.publisher.publish.call_args.args[0].header.stamp == rospy.Time(100)


def started(env, **kwargs):
    node = controller.VisualLandingController()
    node._odom_callback(odom(env, **kwargs))
    node._trigger_callback(Empty())
    return node


def reference(node):
    return np.array([*node.xy_ref, node.z_ref, node.yaw_ref])


def assert_stationary(node, expected):
    node._control()
    np.testing.assert_array_equal(reference(node), expected)
    msg = command(node)
    assert (msg.target_vel_x, msg.target_vel_y, msg.target_vel_z, msg.target_omega_z) == (0, 0, 0, 0)
    assert not node.velocity.any() and node.yaw_rate == 0


def moving(env):
    node = started(env, x=2, y=3, z=4, yaw=0.4)
    node._visual_callback(visual(env, x=0.5, y=0.5, yaw=-0.4))
    for _ in range(5):
        advance(env)
        node._control()
    assert np.linalg.norm(node.velocity) > 0 and node.yaw_rate > 0
    return node


def test_startup_trigger_and_retrigger(env):
    node = controller.VisualLandingController()
    node._visual_callback(visual(env, x=10, y=-10))
    node._control()
    assert node.state == controller.IDLE
    node._trigger_callback(Empty())
    assert not node.active
    node.nav_publisher.publish.assert_not_called()
    node._odom_callback(odom(env, x=2, y=3, z=4, yaw=0.4))
    node._trigger_callback(Empty())
    assert node.state == controller.ALIGNING
    np.testing.assert_allclose(reference(node), [2, 3, 4, 0.4])
    advance(env)
    assert_stationary(node, reference(node).copy())
    msg = command(node)
    assert (msg.target_pos_x, msg.target_pos_y, msg.target_pos_z, msg.target_yaw) == pytest.approx((2, 3, 4, 0.4))
    node._visual_callback(visual(env, x=0.5, yaw=0.2))
    advance(env)
    node._control()
    assert np.linalg.norm(node.velocity) > 0
    cached = visual(env)
    node._visual_callback(cached)
    node._odom_callback(odom(env, x=7, y=8, z=9, yaw=-0.5))
    node._trigger_callback(Empty())
    np.testing.assert_allclose(reference(node), [7, 8, 9, -0.5])
    node._visual_callback(cached)
    assert node.alignment_count == 0 and node.visual_received is None
    assert not node.velocity_target.any()


@pytest.mark.parametrize('world_yaw,relative_yaw,expected', [
    (0, 0, (-1, 0)), (np.pi/2, 0, (0, -1)), (-np.pi/2, 0, (0, 1)),
    (0.3, 0.3, (-1, 0)),
])
def test_dog_to_world_and_x_sign(env, world_yaw, relative_yaw, expected):
    node = started(env, yaw=world_yaw)
    node._visual_callback(visual(env, x=1, yaw=relative_yaw))
    assert node.dog_velocity_target[0] < 0
    np.testing.assert_allclose(node.velocity_target, np.array(expected)*node.max_xy_vel, atol=1e-12)


def test_y_sign_and_clockwise_yaw_correction(env):
    node = started(env)
    node._visual_callback(visual(env, y=0.1, yaw=-0.1))
    assert node.dog_velocity_target[1] == pytest.approx(-0.05)
    assert node.omega_target == pytest.approx(0.08)  # CW measured -> CCW command.
    node._control()
    assert env.publishers[node.robot_ns + '/visual_landing/debug/yaw_error'].publish.call_args.args[0].data > 0


def test_tilted_frame_uses_clean_planar_rotation(env):
    node = started(env)
    w_r_d = tft.euler_matrix(0.3, 0.6, np.pi / 2)[:3, :3]
    d_r_u = tft.euler_matrix(-0.2, 0.15, 0.25)[:3, :3]
    o, v = odom(env), visual(env, x=1)
    for pose, rot in ((o.pose.pose, w_r_d @ d_r_u), (v.pose, d_r_u)):
        matrix = np.eye(4)
        matrix[:3, :3] = rot
        q = tft.quaternion_from_matrix(matrix)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    advance(env)
    o.header.stamp = rospy.Time.from_sec(env.clock.ros)
    node._odom_callback(o)
    node._visual_callback(v)
    np.testing.assert_allclose(node.velocity_target, [0, -node.max_xy_vel], atol=1e-12)


def test_vector_and_yaw_saturation_and_no_minimum_speed(env):
    node = started(env)
    node._visual_callback(visual(env, x=3, y=4, yaw=-1))
    np.testing.assert_allclose(node.dog_velocity_target, [-0.048, -0.064])
    assert np.linalg.norm(node.velocity_target) == pytest.approx(node.max_xy_vel)
    assert node.omega_target == node.max_yaw_rate
    advance(env)
    node._visual_callback(visual(env, x=1e-6, y=-2e-6, yaw=1e-6))
    np.testing.assert_allclose(node.dog_velocity_target, [-0.5e-6, 1e-6])
    assert node.omega_target == pytest.approx(-0.8e-6)


def test_desired_yaw_offset_and_shortest_wrap(env):
    env.params['~desired_relative_yaw'] = -np.pi + 0.05
    node = started(env)
    node._visual_callback(visual(env, yaw=np.pi - 0.05))
    assert node.omega_target == pytest.approx(0.08)


def test_cached_velocity_and_acceleration_limited_consistent_integration(env):
    node = started(env)
    node._visual_callback(visual(env, x=1, y=1, yaw=-1))
    old_xy = node.xy_ref.copy()
    old_velocity = node.velocity.copy()
    old_yaw, old_rate = node.yaw_ref, node.yaw_rate
    for _ in range(10):
        advance(env)
        node._control()
        assert np.linalg.norm(node.velocity - old_velocity) <= node.max_xy_acceleration*0.02 + 1e-12
        assert abs(node.yaw_rate - old_rate) <= node.max_yaw_acceleration*0.02 + 1e-12
        np.testing.assert_allclose(node.xy_ref - old_xy, node.velocity*0.02, atol=1e-12)
        assert controller.wrap(node.yaw_ref-old_yaw) == pytest.approx(node.yaw_rate*0.02)
        old_xy, old_velocity = node.xy_ref.copy(), node.velocity.copy()
        old_yaw, old_rate = node.yaw_ref, node.yaw_rate
    assert node.alignment_count == 0
    assert node.visual_received == 100.0  # Ten control cycles, no new camera frame.


def test_duplicate_timestamp_never_renews_evidence_or_timeout(env):
    node = started(env)
    frame = visual(env)
    node._visual_callback(frame)
    for _ in range(12):
        advance(env)
        node._visual_callback(frame)
        node._control()
    assert node.alignment_count == 1
    assert node.visual_received == 100.0
    advance(env)
    node._visual_callback(frame)
    assert_stationary(node, reference(node).copy())
    assert node.state == controller.VISION_LOST
    assert node.alignment_count == 0


def test_first_timeout_cycle_freezes_instead_of_braking(env):
    node = moving(env)
    frozen = reference(node).copy()
    advance(env, 0.16)  # Unique frame age 0.26 s, velocities were nonzero.
    assert_stationary(node, frozen)
    assert node.state == controller.VISION_LOST
    assert not node.velocity_target.any()
    advance(env)
    assert_stationary(node, frozen)


def test_recovery_before_abort_and_evidence_reset(env):
    node = moving(env)
    advance(env, 0.16)
    node._control()
    assert node.state == controller.VISION_LOST
    advance(env, 0.1)
    node._odom_callback(odom(env, x=2, y=3, z=4, yaw=0.4))
    node._visual_callback(visual(env, x=-0.5, yaw=0.4))
    assert node.state == controller.ALIGNING
    advance(env)
    node._control()
    assert node.velocity[0] > 0 and node.yaw_rate < 0


@pytest.mark.parametrize('timer_before_recovery', [False, True])
def test_late_visual_recovers_without_abort(env, timer_before_recovery):
    node = moving(env)
    frozen = reference(node).copy()
    advance(env, 0.41)
    node._odom_callback(odom(env, x=2, y=3, z=4, yaw=0.4))
    if timer_before_recovery:
        assert_stationary(node, frozen)
        assert node.state == controller.VISION_LOST
    node._visual_callback(visual(env, x=-1, yaw=1))
    assert node.state == controller.ALIGNING
    advance(env)
    node._control()
    assert node.velocity.any()


def test_no_visual_after_trigger_holds_indefinitely(env):
    node = started(env)
    frozen = reference(node).copy()
    for delay in (0.26, 0.25, 1, 5, 60):
        advance(env, delay)
        node._odom_callback(odom(env))
        assert_stationary(node, frozen)
        assert node.state == controller.VISION_LOST


def test_aligned_immediate_stop_and_terminal_hold(env):
    env.params['~required_frames'] = 2
    node = moving(env)
    advance(env)
    node._visual_callback(visual(env, x=0.02, yaw=0.02))
    assert node.alignment_count == 1
    node._control()
    assert node.velocity.any() and node.yaw_rate != 0
    frozen = reference(node).copy()
    advance(env)
    # In the hysteresis band, preserve alignment evidence.
    node._visual_callback(visual(env, x=0.04, yaw=0.07))
    assert node.state == controller.ALIGNED
    assert not node.velocity.any() and node.yaw_rate == 0  # Callback stops immediately.
    assert_stationary(node, frozen)
    advance(env, 2)
    node._visual_callback(visual(env, x=50, yaw=-2))
    assert_stationary(node, frozen)
    assert node.state == controller.ALIGNED
    node._trigger_callback(Empty())  # Stale odom cannot erase the safe hold.
    assert node.state == controller.ALIGNED
    node._odom_callback(odom(env, x=2, y=3, z=4))
    node._trigger_callback(Empty())
    assert node.state == controller.ALIGNING and node.alignment_count == 0


def test_hysteresis_requires_enter_then_preserves_band_and_resets_at_exit(env):
    node = started(env)
    samples = [(0.04, 0.07, 0), (0.02, 0.04, 1), (0.04, 0.07, 2),
               (0.051, 0.07, 0), (0.04, 0.04, 0), (0.02, 0.04, 1),
               (0.02, 0.081, 0), (0.02, 0.07, 0)]
    for x, yaw, count in samples:
        advance(env)
        node._visual_callback(visual(env, x=x, yaw=yaw))
        assert node.alignment_count == count


@pytest.mark.parametrize('clock_offset', [-90, 86400])
def test_unsynchronized_camera_clocks_and_frozen_ros_time(env, clock_offset):
    node = started(env)
    frame = visual(env, x=1)
    frame.header.stamp = rospy.Time.from_sec(env.clock.ros + clock_offset)
    node._visual_callback(frame)
    advance(env)
    node._control()
    assert node.velocity[0] < 0
    frozen = reference(node).copy()
    env.clock.wall += 0.25  # ROS clock frozen, local monotonic timeout still expires.
    node._visual_callback(frame)
    assert_stationary(node, frozen)
    assert node.state == controller.VISION_LOST
    assert node.visual_received == 100.0


def test_reordered_frame_ignored_and_scheduler_stall_bounded(env):
    node = started(env)
    older = visual(env, x=-10)
    advance(env)
    node._visual_callback(visual(env, x=10))
    node._visual_callback(older)
    assert node.velocity_target[0] < 0
    advance(env, 0.15)
    before = node.xy_ref.copy()
    node._control()
    assert np.linalg.norm(node.xy_ref-before) <= node.max_xy_acceleration*0.04**2 + 1e-12


def test_reference_lead_with_nonfollowing_uav_and_zero_feedforward_at_boundary(env):
    node = started(env, x=2, y=-3, z=4)
    anchor = np.array([2, -3])
    for i in range(1000):
        advance(env)
        node._odom_callback(odom(env, x=2, y=-3, z=4))  # UAV never follows.
        if i % 5 == 0:
            node._visual_callback(visual(env, x=3, y=4))
        previous = node.xy_ref.copy()
        node._control()
        assert np.linalg.norm(node.xy_ref-anchor) <= node.max_xy_ref_lead + 1e-12
        np.testing.assert_allclose(node.xy_ref-previous, node.velocity*0.02, atol=1e-12)
    np.testing.assert_allclose(node.xy_ref-anchor, [-0.06, -0.08], atol=1e-12)
    np.testing.assert_allclose(node.velocity, [0, 0], atol=1e-12)
    assert node.state == controller.ALIGNING


def test_odom_discontinuity_aborts_without_teleporting_reference(env):
    node = moving(env)
    frozen = reference(node).copy()
    advance(env)
    node._odom_callback(odom(env, x=50, y=50))
    assert_stationary(node, frozen)
    assert node.state == controller.ABORTED


def test_fixed_altitude_and_flightnav_modes(env):
    node = started(env, z=2.3)
    for z in (-100, -1, 0, 1, 100):
        advance(env)
        node._visual_callback(visual(env, x=0.3, z=z))
        node._control()
        msg = command(node)
        assert node.z_ref == msg.target_pos_z == 2.3
        assert msg.target_vel_z == msg.target_pos_diff_z == 0
        assert msg.target_acc_x == msg.target_acc_y == 0
        assert msg.control_frame == FlightNav.WORLD_FRAME and msg.target == FlightNav.COG
        assert msg.pos_xy_nav_mode == msg.yaw_nav_mode == FlightNav.POS_VEL_MODE
        assert msg.pos_z_nav_mode == FlightNav.POS_MODE


def test_closed_loop_ideal_follower_converges_at_10hz_camera_50hz_control(env):
    node = started(env, x=0.5, y=-0.3, z=2, yaw=-0.4)
    for step in range(1500):
        advance(env)
        node._odom_callback(odom(env, x=node.xy_ref[0], y=node.xy_ref[1], z=2, yaw=node.yaw_ref))
        if step % 5 == 0:
            node._visual_callback(visual(env, x=node.xy_ref[0], y=node.xy_ref[1], z=100, yaw=node.yaw_ref))
        node._control()
        assert command(node).target_pos_z == 2
    assert node.state == controller.ALIGNED
    assert np.linalg.norm(node.xy_ref) < node.align_enter_distance
    assert abs(node.yaw_ref) < node.yaw_enter_threshold


@pytest.mark.parametrize('kind', ['zero', 'wrong_frame', 'nan', 'bad_quaternion'])
def test_invalid_visual_never_drives(env, kind):
    node = started(env)
    message = visual(env, x=1)
    if kind == 'zero':
        message.header.stamp = rospy.Time()
    elif kind == 'wrong_frame':
        message.header.frame_id = 'camera'
    elif kind == 'nan':
        message.pose.position.x = float('nan')
    else:
        message.pose.orientation.w = 0
    node._visual_callback(message)
    assert_stationary(node, reference(node).copy())
    assert node.visual_received is None


@pytest.mark.parametrize('kind', ['stale', 'future', 'zero', 'wrong_frame', 'nan', 'bad_quaternion'])
def test_invalid_odom_cannot_activate(env, kind):
    node = controller.VisualLandingController()
    message = odom(env)
    if kind == 'stale':
        message.header.stamp = rospy.Time(90)
    elif kind == 'future':
        message.header.stamp = rospy.Time(110)
    elif kind == 'zero':
        message.header.stamp = rospy.Time()
    elif kind == 'wrong_frame':
        message.header.frame_id = 'local'
    elif kind == 'nan':
        message.pose.pose.position.z = float('nan')
    else:
        message.pose.pose.orientation.w = 0
    node._odom_callback(message)
    node._trigger_callback(Empty())
    node._control()
    assert node.state == controller.IDLE and not node.active
    node.nav_publisher.publish.assert_not_called()


def test_odom_loss_is_immediate_abort_requiring_trigger(env):
    node = moving(env)
    frozen = reference(node).copy()
    # Unique camera frames continue, but odometry stops.
    for _ in range(5):
        advance(env, 0.1)
        node._visual_callback(visual(env, x=1))
    assert node.state == controller.ABORTED
    assert_stationary(node, frozen)
    node._odom_callback(odom(env))
    advance(env)
    node._visual_callback(visual(env))
    assert_stationary(node, frozen)
    assert node.state == controller.ABORTED


def test_state_topic_diagnostics_and_configurable_rate(env):
    env.params['~control_rate'] = 25
    node = controller.VisualLandingController()
    assert rospy.Timer.call_args.args[0].to_sec() == pytest.approx(1/25)
    assert (node.IDLE, node.ALIGNING, node.ALIGNED, node.VISION_LOST, node.ABORTED) == (0, 1, 2, 3, 4)
    assert node.state_publisher.msg_type is UInt8
    assert node.state_publisher.options['latch']
    assert node.state_publisher.publish.call_args.args[0].data == node.IDLE
    node._odom_callback(odom(env, z=2))
    node._trigger_callback(Empty())
    node._visual_callback(visual(env, x=0.1, y=-0.2, z=3, yaw=-0.1))
    advance(env)
    node._control()
    before = reference(node).copy()
    node._publish_debug(env.clock.wall)
    np.testing.assert_array_equal(reference(node), before)
    def debug(name):
        return node.debug_publishers[name].publish.call_args.args[0].data
    assert debug('relative_z') == 3 and debug('z_ref') == 2
    assert debug('error_x') == pytest.approx(-0.1)
    assert debug('error_y') == pytest.approx(0.2)
    assert debug('yaw_error') == pytest.approx(0.1)
    assert debug('vx_world') == command(node).target_vel_x
    assert debug('omega_ref') == command(node).target_omega_z
    assert debug('visual_measurement_age') == pytest.approx(0.02)


def test_parameter_loading_private_precedence_and_conservative_defaults(env):
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / 'config/visual_landing.yaml').read_text())['visual_landing']
    node = controller.VisualLandingController()
    for key in ('control_rate', 'xy_kp', 'max_xy_vel', 'max_xy_ref_lead', 'yaw_kp', 'max_yaw_rate',
                'required_frames', 'visual_message_timeout', 'descent_rate'):
        assert getattr(node, key) == config[key]
    env.params.update({'/visual_landing/xy_kp': 0.4, '~visual_landing/xy_kp': 0.3, '~xy_kp': 0.2})
    assert controller.VisualLandingController().xy_kp == 0.2
    del env.params['~xy_kp']
    assert controller.VisualLandingController().xy_kp == 0.3
    del env.params['~visual_landing/xy_kp']
    assert controller.VisualLandingController().xy_kp == 0.4


def test_diagnostic_publication_error_does_not_stop_navigation(env):
    node = moving(env)
    node.debug_publishers['relative_x'].publish.side_effect = rospy.ROSException('debug unavailable')
    count = node.nav_publisher.publish.call_count
    advance(env)
    node._control()
    assert node.nav_publisher.publish.call_count == count + 1
    assert node.state == controller.ALIGNING and node.velocity.any()
    advance(env, 0.2)
    frozen = reference(node).copy()
    assert_stationary(node, frozen)
    assert node.state == controller.VISION_LOST


@pytest.mark.parametrize('params', [
    {'~descent_rate': 0}, {'~control_rate': 0}, {'~max_xy_ref_lead': -1},
    {'~xy_kp': float('nan')}, {'~required_frames': 0}, {'~required_frames': 1.5},
    {'~align_enter_distance': 0.06}, {'~yaw_enter_threshold': 0.1},
])
def test_invalid_controller_configuration(env, params):
    env.params.update(params)
    with pytest.raises(ValueError):
        controller.VisualLandingController()
    assert not env.publishers



"""Triggered UAV-local XY/yaw alignment, with optional slow descent and land handoff.

Only measurements cross the network. An acceleration-limited reference runs at
50 Hz by default, including between camera frames. Descent is opt-in; the default
holds trigger-time altitude. Final landing is delegated to the JSK navigator.
"""

import math
import threading
import time

import numpy as np
import rospy
import tf.transformations as tft
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Float64, UInt8, UInt32

IDLE, ALIGNING, ALIGNED, VISION_LOST, ABORTED = range(5)
DESCENDING = 5
STATE_NAMES = {IDLE: 'IDLE', ALIGNING: 'ALIGNING', ALIGNED: 'ALIGNED',
               VISION_LOST: 'VISION_LOST', ABORTED: 'ABORTED', DESCENDING: 'DESCENDING'}


def wrap(rad):
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(rad), math.cos(rad))


def limit_vector(vector, maximum):
    length = np.linalg.norm(vector)
    return vector * (maximum / length) if length > maximum else vector.copy()


def pose_state(pose):
    p, q = pose.position, pose.orientation
    position = np.array([p.x, p.y, p.z], dtype=float)
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if (not np.all(np.isfinite(position)) or not np.all(np.isfinite(quaternion))
            or np.linalg.norm(quaternion) < 1e-12):
        return None
    quaternion /= np.linalg.norm(quaternion)
    return position, tft.quaternion_matrix(quaternion)[:3, :3], tft.euler_from_quaternion(quaternion)[2]


class VisualLandingController:
    IDLE, ALIGNING, ALIGNED, VISION_LOST, ABORTED = IDLE, ALIGNING, ALIGNED, VISION_LOST, ABORTED
    DESCENDING = DESCENDING

    @staticmethod
    def _param(name, default):
        """Scalar private overrides > private YAML section > root YAML section."""
        return rospy.get_param('~' + name, rospy.get_param(
            '~visual_landing/' + name, rospy.get_param('/visual_landing/' + name, default)))

    def __init__(self):
        self.robot_ns = ('/' + str(self._param('robot_ns', 'xuanwu')).strip('/')).rstrip('/')
        self.docking_frame = self._param('docking_frame', 'qilin_docking_frame')
        self.world_frame = self._param('world_frame', 'world').lstrip('/')
        descend = self._param('descend', False)
        if isinstance(descend, str) and descend.lower() in ('true', 'false'):
            descend = descend.lower() == 'true'
        if not isinstance(descend, bool):
            raise ValueError('descend must be a boolean')
        self.descend = descend
        for name, default in (
                ('control_rate', 50.0), ('xy_kp', 0.5), ('yaw_kp', 0.8),
                ('max_xy_vel', 0.08), ('max_xy_ref_lead', 0.10),
                ('max_xy_acceleration', 0.25), ('max_yaw_rate', 0.15),
                ('max_yaw_acceleration', 0.4), ('odom_timeout', 0.5),
                ('visual_message_timeout', 0.25), ('descent_rate', 0.03),
                ('max_z_ref_lead', 0.05), ('land_trigger_height', 0.30),
                ('future_tolerance', 0.1), ('align_enter_distance', 0.03),
                ('align_exit_distance', 0.05), ('yaw_enter_threshold', 0.05),
                ('yaw_exit_threshold', 0.08)):
            value = float(self._param(name, default))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be finite and positive')
            setattr(self, name, value)
        self.desired_xy = np.array([float(self._param('desired_x', 0.0)),
                                    float(self._param('desired_y', 0.0))])
        self.desired_yaw = float(self._param('desired_relative_yaw', 0.0))
        frames = self._param('required_frames', 10)
        self.required_frames = int(frames)
        landing_frames = self._param('landing_required_frames', 3)
        self.landing_required_frames = int(landing_frames)
        if (not np.all(np.isfinite(self.desired_xy)) or not math.isfinite(self.desired_yaw)
                or self.required_frames < 1 or float(frames) != self.required_frames
                or self.align_exit_distance <= self.align_enter_distance
                or self.yaw_exit_threshold <= self.yaw_enter_threshold
                or self.landing_required_frames < 1
                or float(landing_frames) != self.landing_required_frames):
            raise ValueError('Invalid visual alignment target or confirmation thresholds')
        self._lock = threading.RLock()
        self.active = False
        self.state = IDLE
        self.odom = None
        self.odom_received = None
        self.last_visual_stamp = None  # Never reset by a trigger: cached frames stay cached.
        self.visual = None
        self.visual_received = None
        self.alignment_count = 0
        self.landing_count = 0
        self.land_command_sent = False
        self.xy_aligned = False
        self.yaw_aligned = False
        self.velocity_target = np.zeros(2)
        self.dog_velocity_target = np.zeros(2)
        self.omega_target = 0.0
        self.trigger_time = None
        self.tracking_started = False
        self.xy_ref = None
        self.z_ref = None
        self.yaw_ref = None
        self.velocity = np.zeros(2)
        self.yaw_rate = 0.0
        self.vertical_velocity = 0.0
        self.previous_time = None
        self.nav_publisher = rospy.Publisher(self.robot_ns + '/uav/nav', FlightNav, queue_size=1)
        self.land_publisher = rospy.Publisher(self.robot_ns + '/teleop_command/land', Empty, queue_size=1)
        self.state_publisher = rospy.Publisher(self.robot_ns + '/visual_landing/state', UInt8,
                                               queue_size=1, latch=True)
        self.state_publisher.publish(UInt8(data=self.state))
        self.debug_publishers = {
            name: rospy.Publisher(self.robot_ns + '/visual_landing/debug/' + name,
                                  Float64, queue_size=1)
            for name in ('relative_x', 'relative_y', 'relative_z', 'relative_yaw',
                         'error_x', 'error_y', 'error_xy_norm', 'yaw_error',
                         'vx_dog', 'vy_dog', 'vx_world', 'vy_world', 'omega_ref',
                         'x_ref', 'y_ref', 'z_ref', 'yaw_ref', 'visual_measurement_age')
        }
        self.counter_publisher = rospy.Publisher(
            self.robot_ns + '/visual_landing/debug/aligned_frame_counter', UInt32, queue_size=1)
        # All state is initialized before any callback can run.
        self.odom_subscriber = rospy.Subscriber(self.robot_ns + '/uav/cog/odom', Odometry,
                                                self._odom_callback, queue_size=1)
        self.info_subscriber = rospy.Subscriber(self.robot_ns + '/visual_landing/info', PoseStamped,
                                                self._visual_callback, queue_size=1)
        self.trigger_subscriber = rospy.Subscriber(self.robot_ns + '/visual_landing/trigger', Empty,
                                                   self._trigger_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self._control)
        rospy.loginfo('Visual landing controller ready; state=IDLE')

    def _set_state(self, state):
        if state != self.state:
            previous = self.state
            self.state = state
            self.state_publisher.publish(UInt8(data=state))
            rospy.loginfo('Visual landing: %s -> %s', STATE_NAMES[previous], STATE_NAMES[state])

    def _stamp_fresh(self, stamp, timeout):
        age = (rospy.Time.now() - stamp).to_sec()
        return stamp.to_sec() > 0 and -self.future_tolerance <= age <= timeout

    def _odom_fresh(self, now):
        return (self.odom is not None and self.odom_received is not None
                and 0 <= now - self.odom_received <= self.odom_timeout
                and self._stamp_fresh(self.odom[3], self.odom_timeout))

    def _visual_age(self, now):
        # Source ROS clocks are not guaranteed synchronized across computers.
        origin = self.visual_received if self.visual_received is not None else self.trigger_time
        return float('inf') if origin is None else max(0.0, now - origin)

    def _stop_reference(self):
        """Emergency/terminal hold bypasses all normal acceleration limiting."""
        self.velocity[:] = 0.0
        self.yaw_rate = 0.0
        self.vertical_velocity = 0.0
        self.velocity_target[:] = 0.0
        self.dog_velocity_target[:] = 0.0
        self.omega_target = 0.0

    def _reset_evidence(self):
        self.alignment_count = 0
        self.landing_count = 0
        self.xy_aligned = self.yaw_aligned = False

    def _check_safety(self, now):
        """Check local faults and reset stale visual evidence before accepting a frame."""
        if not self.active or self.state in (ALIGNED, ABORTED):
            return
        if not self._odom_fresh(now):
            self._stop_reference()
            self._reset_evidence()
            self._set_state(ABORTED)
            rospy.logwarn('Visual landing aborted: UAV odometry is stale; new trigger required.')
            return
        age = self._visual_age(now)
        if age > self.visual_message_timeout:
            self._stop_reference()
            self._reset_evidence()
            self._set_state(VISION_LOST)

    def _odom_callback(self, message):
        value = pose_state(message.pose.pose)
        if value is None or message.header.frame_id.lstrip('/') != self.world_frame:
            rospy.logwarn_throttle(2.0, 'Ignoring invalid odometry or unexpected odometry frame.')
            return
        if not self._stamp_fresh(message.header.stamp, self.odom_timeout):
            return
        with self._lock:
            if self.odom is not None and message.header.stamp <= self.odom[3]:
                return
            self.odom = (*value, message.header.stamp)
            self.odom_received = time.monotonic()

    def _visual_callback(self, message):
        value = pose_state(message.pose)
        if value is None or message.header.frame_id != self.docking_frame:
            rospy.logwarn_throttle(2.0, 'Ignoring invalid visual pose or unexpected docking frame.')
            return
        # Only identity/order is inferred from the remote clock. Zero is invalid.
        if message.header.stamp.to_sec() <= 0:
            return
        with self._lock:
            # A repeated or reordered network packet cannot renew freshness or evidence.
            if self.last_visual_stamp is not None and message.header.stamp <= self.last_visual_stamp:
                return
            now = time.monotonic()
            self._check_safety(now)
            self.last_visual_stamp = message.header.stamp
            self.visual = (*value, message.header.stamp)
            self.visual_received = now
            if not self.active or self.state in (ALIGNED, ABORTED):
                return
            recovering = self.state == VISION_LOST
            position, rotation, yaw = value
            _, odom_rotation, _, _ = self.odom
            # ^A R_B maps B vectors into A. Extract heading, then use a clean R2:
            # tilted 3D rotations must not shrink/mix planar velocity components.
            world_docking = odom_rotation @ rotation.T
            yaw_wd = math.atan2(world_docking[1, 0], world_docking[0, 0])
            c, s = math.cos(yaw_wd), math.sin(yaw_wd)
            error = self.desired_xy - position[:2]
            yaw_error = wrap(self.desired_yaw - yaw)
            self.dog_velocity_target = limit_vector(self.xy_kp * error, self.max_xy_vel)
            self.velocity_target = np.array([[c, -s], [s, c]]) @ self.dog_velocity_target
            self.omega_target = float(np.clip(self.yaw_kp * yaw_error,
                                              -self.max_yaw_rate, self.max_yaw_rate))
            distance, angle = np.linalg.norm(error), abs(yaw_error)
            if self.state == DESCENDING:
                if distance > self.align_exit_distance or angle > self.yaw_exit_threshold:
                    self.vertical_velocity = 0.0
                    self._reset_evidence()
                    self._set_state(ALIGNING)
                else:
                    self.landing_count = (self.landing_count + 1
                                          if position[2] <= self.land_trigger_height else 0)
                    if self.landing_count >= self.landing_required_frames:
                        # Under the same lock as _control: no later visual nav can
                        # race the standard landing controller. Latch before send.
                        self._stop_reference()
                        self.active = False
                        self.land_command_sent = True
                        self.land_publisher.publish(Empty())
                        rospy.loginfo('Visual descent handed off to JSK land at relative Z %.3f m.',
                                      position[2])
                return
            self.xy_aligned = (distance < self.align_exit_distance if self.xy_aligned
                               else distance <= self.align_enter_distance)
            self.yaw_aligned = (angle < self.yaw_exit_threshold if self.yaw_aligned
                                else angle <= self.yaw_enter_threshold)
            if self.xy_aligned and self.yaw_aligned:
                self.alignment_count += 1
            else:
                self.alignment_count = 0
            self._set_state(ALIGNING)
            if not self.tracking_started:
                self.tracking_started = True
                rospy.loginfo('Visual tracking active: first fresh post-trigger camera frame.')
            if self.alignment_count >= self.required_frames and not recovering:
                if self.descend:
                    self._set_state(DESCENDING)
                else:
                    self._stop_reference()
                    self._set_state(ALIGNED)
                rospy.loginfo('Visual alignment complete: XY error %.4f m, yaw error %.4f rad.',
                              distance, yaw_error)

    def _trigger_callback(self, _message):
        with self._lock:
            if self.land_command_sent:
                rospy.logwarn('Visual trigger ignored after land handoff; restart node for a new flight.')
                return
            now = time.monotonic()
            if not self._odom_fresh(now):
                self._check_safety(now)
                rospy.logwarn('Visual alignment trigger ignored: no fresh valid UAV odometry.')
                return
            self._stop_reference()
            self._reset_evidence()
            self.visual = None
            self.visual_received = None
            self.trigger_time = now
            self.tracking_started = False
            position, _, yaw, _ = self.odom
            self.xy_ref = position[:2].copy()
            self.z_ref = float(position[2])
            self.yaw_ref = float(yaw)
            self.previous_time = now
            self.active = True
            self._set_state(ALIGNING)
            rospy.loginfo('Visual trigger: references initialized from odometry (%.3f, %.3f, %.3f), yaw %.3f.',
                          *position, yaw)

    def _control(self, _event=None):
        with self._lock:
            if not self.active:
                self._publish_debug(time.monotonic())
                return
            now = time.monotonic()
            # Bound integration after scheduler stalls; never jump through elapsed downtime.
            dt = max(0.0, min(now - self.previous_time, 2.0 / self.control_rate))
            self.previous_time = now
            self._check_safety(now)
            self.vertical_velocity = 0.0
            if self.state in (ALIGNING, DESCENDING) and self.visual_received is not None and dt > 0:
                self.velocity += limit_vector(self.velocity_target - self.velocity,
                                               self.max_xy_acceleration * dt)
                self.yaw_rate += float(np.clip(self.omega_target - self.yaw_rate,
                                              -self.max_yaw_acceleration * dt,
                                              self.max_yaw_acceleration * dt))
                candidate = self.xy_ref + self.velocity * dt
                odom_xy = self.odom[0][:2]
                z_lead_fault = (self.state == DESCENDING
                                and self.odom[0][2] - self.z_ref > self.max_z_ref_lead + 1e-9)
                if np.linalg.norm(self.xy_ref - odom_xy) > self.max_xy_ref_lead + 1e-9 or z_lead_fault:
                    # An odometry jump/external displacement can make a frozen ref
                    # violate the radius already. Do not teleport it to chase odom.
                    self._stop_reference()
                    self._set_state(ABORTED)
                    rospy.logwarn('Visual landing aborted: odometry moved outside XY/Z reference-lead bounds.')
                else:
                    projected = odom_xy + limit_vector(candidate - odom_xy, self.max_xy_ref_lead)
                    # Feedforward must match the bounded trajectory, including at
                    # the anti-windup boundary (which overrides acceleration limits).
                    self.velocity = (projected - self.xy_ref) / dt
                    self.xy_ref = projected
                    self.yaw_ref = wrap(self.yaw_ref + self.yaw_rate * dt)
                    if self.state == DESCENDING:
                        candidate_z = min(self.z_ref, max(
                            self.z_ref - self.descent_rate * dt,
                            self.odom[0][2] - self.max_z_ref_lead))
                        # Consistent feedforward: zero at the anti-windup boundary.
                        self.vertical_velocity = (candidate_z - self.z_ref) / dt
                        self.z_ref = candidate_z
            else:
                # Freeze XY/yaw/Z immediately on stale vision or terminal state.
                # Before the first new post-trigger frame the captured reference holds.
                self.velocity[:] = 0.0
                self.yaw_rate = 0.0
            message = FlightNav()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = self.world_frame
            message.control_frame = FlightNav.WORLD_FRAME
            message.target = FlightNav.COG
            message.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
            message.target_pos_x, message.target_pos_y = self.xy_ref
            message.target_vel_x, message.target_vel_y = self.velocity
            message.target_acc_x = message.target_acc_y = 0.0
            message.pos_z_nav_mode = (FlightNav.POS_VEL_MODE if self.state == DESCENDING
                                     else FlightNav.POS_MODE)
            message.target_pos_z = self.z_ref
            message.target_vel_z = self.vertical_velocity
            message.target_pos_diff_z = 0.0
            message.yaw_nav_mode = FlightNav.POS_VEL_MODE
            message.target_yaw = self.yaw_ref
            message.target_omega_z = self.yaw_rate
            self.nav_publisher.publish(message)
            self._publish_debug(now)

    def _publish_debug(self, now):
        """Read-only diagnostics; unavailable values are NaN, never fake zeros."""
        nan = float('nan')
        position, yaw = (np.full(3, nan), nan) if self.visual is None else (self.visual[0], self.visual[2])
        error = self.desired_xy - position[:2]
        values = dict(zip(('relative_x', 'relative_y', 'relative_z'), position))
        values.update(relative_yaw=yaw, error_x=error[0], error_y=error[1],
                      error_xy_norm=np.linalg.norm(error), yaw_error=wrap(self.desired_yaw - yaw),
                      vx_dog=self.dog_velocity_target[0], vy_dog=self.dog_velocity_target[1],
                      vx_world=self.velocity[0], vy_world=self.velocity[1], omega_ref=self.yaw_rate,
                      x_ref=nan if self.xy_ref is None else self.xy_ref[0],
                      y_ref=nan if self.xy_ref is None else self.xy_ref[1],
                      z_ref=nan if self.z_ref is None else self.z_ref,
                      yaw_ref=nan if self.yaw_ref is None else self.yaw_ref,
                      visual_measurement_age=self._visual_age(now))
        try:
            for name, value in values.items():
                self.debug_publishers[name].publish(Float64(data=float(value)))
            self.counter_publisher.publish(UInt32(data=self.alignment_count))
        except rospy.ROSException as error:
            # A failed diagnostic publisher must not terminate the control timer.
            rospy.logwarn_throttle(5.0, 'Visual landing diagnostic publication failed: %s', error)


def main():
    rospy.init_node('visual_landing_controller')
    node = VisualLandingController()
    rospy.spin()

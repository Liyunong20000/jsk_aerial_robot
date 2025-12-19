#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav

class HoverAboveTag:
    def __init__(self):
        rospy.init_node('hover_above_tag')

        # --- SETTINGS ---
        self.ns = "/xuanwu"
        self.kp_xy = 1.0        # XY Gain 
        self.kp_z = 1.0         # Z Gain
        self.max_vel = 0.5      # Max speed clamp (m/s)
        self.hover_height = 0.30 # 10cm above the tag
        
        # --- FRAMES ---
        self.world_frame = "world"
        self.tag_frame = "land_mark" # Ensure this matches your TF tree

        # --- ROS COMMUNICATION ---
        self.nav_pub = rospy.Publisher(self.ns + '/uav/nav', FlightNav, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.ns + '/uav/baselink/odom', Odometry, self.odom_callback)
        
        # --- TF BUFFER ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.current_pos = None # [x, y, z]

        rospy.loginfo(">> Tag Hover Node Initialized. Waiting for Odom...")

    def odom_callback(self, msg):
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

    def get_tag_target(self):
        """Returns [x, y, z] of the point 10cm above the tag, or None."""
        try:
            # Look for the latest transform
            trans = self.tf_buffer.lookup_transform(
                self.world_frame, 
                self.tag_frame, 
                rospy.Time(0), 
                rospy.Duration(0.1)
            )
            
            # Target = Tag Position + Offset
            target = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z + self.hover_height
            ])
            return target
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def send_velocity_command(self, target_pos):
        # 1. Calculate Error
        error = target_pos - self.current_pos

        # 2. Calculate Velocity (P-Controller)
        vel = np.zeros(3)
        vel[0] = error[0] * self.kp_xy
        vel[1] = error[1] * self.kp_xy
        vel[2] = error[2] * self.kp_z

        # 3. Clip Velocity (Safety)
        vel[0] = np.clip(vel[0], -self.max_vel, self.max_vel)
        vel[1] = np.clip(vel[1], -self.max_vel, self.max_vel)
        vel[2] = np.clip(vel[2], -self.max_vel, self.max_vel)

        # 4. Construct Message
        nav_msg = FlightNav()
        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.header.frame_id = "world"
        nav_msg.control_frame = FlightNav.WORLD_FRAME
        nav_msg.target = FlightNav.COG

        # VEL_MODE
        nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
        nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
        
        # Keep Yaw stable (0.0 rad)
        nav_msg.yaw_nav_mode = FlightNav.POS_MODE 
        nav_msg.target_yaw = 0.0

        nav_msg.target_vel_x = vel[0]
        nav_msg.target_vel_y = vel[1]
        nav_msg.target_vel_z = vel[2]

        self.nav_pub.publish(nav_msg)

    def stop_drone(self):
        """Sends zero velocity to brake/hover if tag is lost."""
        nav_msg = FlightNav()
        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.header.frame_id = "world"
        nav_msg.control_frame = FlightNav.WORLD_FRAME
        nav_msg.target = FlightNav.COG
        nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
        nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
        nav_msg.yaw_nav_mode = FlightNav.POS_MODE
        # All vels 0
        self.nav_pub.publish(nav_msg)

    def run(self):
        rate = rospy.Rate(20) # 20Hz Loop
        
        while not rospy.is_shutdown():
            if self.current_pos is not None:
                
                # 1. Try to find tag
                target = self.get_tag_target()
                
                if target is not None:
                    # 2. If found, fly to it
                    dist = np.linalg.norm(self.current_pos - target)
                    rospy.loginfo_throttle(1, f"Tracking Tag... Dist: {dist:.3f}m")
                    self.send_velocity_command(target)
                else:
                    # 3. If lost, stop/hover in place
                    rospy.logwarn_throttle(1, "Tag not visible! Stopping...")
                    self.stop_drone()
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = HoverAboveTag()
        node.run()
    except rospy.ROSInterruptException:
        pass

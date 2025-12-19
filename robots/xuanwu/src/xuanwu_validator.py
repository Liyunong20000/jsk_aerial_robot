#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
import random
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav

class XuanwuVelValidator:
    def __init__(self):
        rospy.init_node('xuanwu_vel_validator')

        # --- CONFIGURATION ---
        self.ns = "/xuanwu"
        # Control Gains (P-Controller) - Adjust if too aggressive/slow
        self.kp_xy = 0.8  
        self.kp_z = 0.8
        self.max_vel_xy = 0.3 # Limit speed for safety (m/s)
        self.max_vel_z = 0.2

        # --- PUBLISHERS ---
        self.nav_pub = rospy.Publisher(self.ns + '/uav/nav', FlightNav, queue_size=1)
        self.start_pub = rospy.Publisher(self.ns + '/teleop_command/start', Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher(self.ns + '/teleop_command/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher(self.ns + '/teleop_command/land', Empty, queue_size=1)
        
        # --- SUBSCRIBERS ---
        self.odom_sub = rospy.Subscriber(self.ns + '/uav/baselink/odom', Odometry, self.odom_callback)

        # --- TF & STATE ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.current_pos = None # [x, y, z]
        self.target_pos = None  # [x, y, z]
        self.state = "INIT"
        
        # World Frame (ensure this matches your RViz/Simulation world frame)
        self.world_frame = "world" 
        self.tag_frame = "landmark"

        rospy.loginfo(">> VELOCITY VALIDATOR INITIALIZED. Waiting for Odom...")

    def odom_callback(self, msg):
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

    def get_tag_position(self):
        """Returns the world position of the tag if visible."""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.world_frame, 
                self.tag_frame, 
                rospy.Time(0), 
                rospy.Duration(0.1)
            )
            return np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def send_velocity_cmd(self):
        """Calculates velocity based on distance to target (P-Controller)"""
        if self.current_pos is None or self.target_pos is None:
            return

        # 1. Calculate Error Vector
        error = self.target_pos - self.current_pos
        
        # 2. Calculate Desired Velocity (P-Control)
        vel_cmd = np.zeros(3)
        vel_cmd[0] = error[0] * self.kp_xy # vx
        vel_cmd[1] = error[1] * self.kp_xy # vy
        vel_cmd[2] = error[2] * self.kp_z  # vz

        # 3. Clamp Velocities (Safety)
        # XY Magnitude clamp
        xy_speed = np.linalg.norm(vel_cmd[:2])
        if xy_speed > self.max_vel_xy:
            scale = self.max_vel_xy / xy_speed
            vel_cmd[0] *= scale
            vel_cmd[1] *= scale
        
        # Z clamp
        vel_cmd[2] = np.clip(vel_cmd[2], -self.max_vel_z, self.max_vel_z)

        # 4. Create Message
        nav_msg = FlightNav()
        nav_msg.header.frame_id = self.world_frame
        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.control_frame = FlightNav.WORLD_FRAME
        nav_msg.target = FlightNav.COG

        # SET MODES TO VELOCITY (Important!)
        nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
        nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
        nav_msg.yaw_nav_mode = FlightNav.POS_MODE # Keep Yaw stable
        
        nav_msg.target_vel_x = vel_cmd[0]
        nav_msg.target_vel_y = vel_cmd[1]
        nav_msg.target_vel_z = vel_cmd[2]
        nav_msg.target_yaw = 0.0 # Face forward

        self.nav_pub.publish(nav_msg)

    def run(self):
        rate = rospy.Rate(20) # 20 Hz loop
        
        # Random Point Generator
        random_x = 1.0 + random.uniform(-0.2, 0.2)
        random_y = 1.0 + random.uniform(-0.2, 0.2)
        random_z = 2.0 

        while not rospy.is_shutdown():
            if self.state == "INIT":
                if self.current_pos is not None:
                    rospy.loginfo(">> Odom OK. Arming (3s)...")
                    self.start_pub.publish(Empty())
                    rospy.sleep(3.0)
                    
                    rospy.loginfo(">> Taking Off (Sent cmd)...")
                    self.takeoff_pub.publish(Empty())
                    self.state = "WAIT_TAKEOFF"

            elif self.state == "WAIT_TAKEOFF":
                # Wait until height > 0.5m
                if self.current_pos[2] > 0.5:
                    rospy.loginfo(f">> Airborne! Target: [{random_x:.2f}, {random_y:.2f}, {random_z:.2f}]")
                    self.target_pos = np.array([random_x, random_y, random_z])
                    self.state = "GO_TO_POINT"
                else:
                    self.takeoff_pub.publish(Empty()) # Retry takeoff if stuck

            elif self.state == "GO_TO_POINT":
                self.send_velocity_cmd() # <--- DOING THE CONTROL HERE
                
                dist = np.linalg.norm(self.current_pos - self.target_pos)
                if dist < 0.15:
                    rospy.loginfo(">> Reached Point. Searching for Tag...")
                    rospy.sleep(0.5)
                    self.state = "SEARCH_TAG"

            elif self.state == "SEARCH_TAG":
                # Hover (target_pos is still previous point)
                self.send_velocity_cmd()
                
                tag_pos = self.get_tag_position()
                if tag_pos is not None:
                    # Target = Tag Position + 0.1m Up
                    self.target_pos = np.array([tag_pos[0], tag_pos[1], tag_pos[2] + 0.1])
                    rospy.loginfo(">> TAG FOUND! Approaching hover point...")
                    self.state = "HOVER_TAG"
                else:
                    rospy.logwarn_throttle(2, ">> No Tag TF found...")

            elif self.state == "HOVER_TAG":
                # Update target continuously
                tag_pos = self.get_tag_position()
                if tag_pos is not None:
                    self.target_pos = np.array([tag_pos[0], tag_pos[1], tag_pos[2] + 0.1])
                
                self.send_velocity_cmd()

                dist = np.linalg.norm(self.current_pos - self.target_pos)
                if dist < 0.10: # Precision tolerance
                    rospy.loginfo(">> Hover Stable. Landing in 3s...")
                    rospy.sleep(3.0)
                    self.state = "LAND"

            elif self.state == "LAND":
                rospy.loginfo_throttle(1, ">> Landing...")
                self.land_pub.publish(Empty())
                if self.current_pos[2] < 0.2:
                    rospy.loginfo(">> Landed. Done.")
                    break

            rate.sleep()

if __name__ == '__main__':
    try:
        node = XuanwuVelValidator()
        node.run()
    except rospy.ROSInterruptException:
        pass

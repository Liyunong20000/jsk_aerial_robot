#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
import random
import time
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav

class FullMission:
    def __init__(self):
        rospy.init_node('full_mission_node')

        # --- CONFIGURATION ---
        self.ns = "/xuanwu"
        self.teleop_ns = self.ns + "/teleop_command"
        
        # Navigation Settings
        self.random_target = np.array([
            1.0 + random.uniform(-0.2, 0.2),
            1.0 + random.uniform(-0.2, 0.2),
            2.0
        ])
        self.hover_height = 0.4     # 30cm above tag
        self.nav_tolerance = 0.15    # 15cm tolerance for waypoints
        self.land_threshold = 0.25   # 20cm height to consider "landed"
        
        # Frames
        self.world_frame = "world"
        self.tag_frame = "land_mark"

        # --- PUBLISHERS ---
        self.start_pub = rospy.Publisher(self.teleop_ns + '/start', Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher(self.teleop_ns + '/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher(self.teleop_ns + '/land', Empty, queue_size=1)
        self.halt_pub = rospy.Publisher(self.teleop_ns + '/halt', Empty, queue_size=1)
        self.task_start_pub = rospy.Publisher('task_start', Empty, queue_size=1)
        self.nav_pub = rospy.Publisher(self.ns + '/uav/nav', FlightNav, queue_size=1)
        
        # --- SUBSCRIBERS & TF ---
        self.odom_sub = rospy.Subscriber(self.ns + '/uav/baselink/odom', Odometry, self.odom_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # --- STATE VARIABLES ---
        self.state = "INIT"
        self.current_pos = None # [x, y, z]
        self.state_timer = 0
        self.hover_start_time = None
        self.landed_start_time = None

        rospy.loginfo(f"Mission Initialized. Random Target: {self.random_target}")

    def odom_callback(self, msg):
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

    def send_pos_cmd(self, target_pos):
        nav_msg = FlightNav()
        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.header.frame_id = self.world_frame
        nav_msg.control_frame = FlightNav.WORLD_FRAME
        nav_msg.target = FlightNav.COG

        # Position Mode
        nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
        nav_msg.yaw_nav_mode = FlightNav.POS_MODE 

        nav_msg.target_pos_x = target_pos[0]
        nav_msg.target_pos_y = target_pos[1]
        nav_msg.target_pos_z = target_pos[2]
        nav_msg.target_yaw = 0.0

        self.nav_pub.publish(nav_msg)

    def get_tag_target(self):
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
                trans.transform.translation.z + self.hover_height
            ])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def run(self):
        rate = rospy.Rate(10) # 10Hz Main Loop
        
        while not rospy.is_shutdown():
            # Wait for Odom
            if self.current_pos is None:
                rate.sleep()
                continue

            # ==========================================
            # 1. ARMING
            # ==========================================
            if self.state == "INIT":
                rospy.loginfo(">> 1. ARMING MOTORS...")
                self.state = "ARMING"
                self.state_timer = 0

            elif self.state == "ARMING":
                self.start_pub.publish(Empty())
                self.state_timer += 1
                
                # Wait 3 seconds (30 ticks at 10Hz)
                if self.state_timer > 30:
                    rospy.loginfo(">> Arming Complete. Switching to TAKEOFF.")
                    self.state = "TAKEOFF"

            # ==========================================
            # 2. TAKEOFF
            # ==========================================
            elif self.state == "TAKEOFF":
                # Check if airborne
                if self.current_pos[2] > 0.5:
                    rospy.loginfo(f">> Takeoff Detected (Z: {self.current_pos[2]:.2f}m). Enabling Navigation...")
                    
                    # Send 'x' (Task Start) once to be safe
                    self.task_start_pub.publish(Empty())
                    rospy.sleep(0.5) 
                    
                    self.state = "GO_TO_POINT"
                else:
                    self.takeoff_pub.publish(Empty())
                    rospy.loginfo_throttle(1, f"   >> Sending Takeoff... (Z: {self.current_pos[2]:.2f}m)")

            # ==========================================
            # 3. GO TO RANDOM POINT
            # ==========================================
            elif self.state == "GO_TO_POINT":
                self.send_pos_cmd(self.random_target)
                
                dist = np.linalg.norm(self.current_pos - self.random_target)
                
                if dist < self.nav_tolerance:
                    rospy.loginfo(f">> Reached Random Point (Err: {dist:.2f}m). Searching for Tag...")
                    self.state = "SEARCH_AND_HOVER"
                    self.hover_start_time = None # Reset hover timer

            # ==========================================
            # 4. HOVER ABOVE TAG
            # ==========================================
            elif self.state == "SEARCH_AND_HOVER":
                tag_target = self.get_tag_target()
                
                if tag_target is not None:
                    # Fly to tag
                    self.send_pos_cmd(tag_target)
                    dist = np.linalg.norm(self.current_pos - tag_target)
                    
                    # Check if we are "At" the tag
                    if dist < self.nav_tolerance:
                        if self.hover_start_time is None:
                            self.hover_start_time = time.time()
                            rospy.loginfo(">> At Tag Position. Stabilizing (Wait 2s)...")
                        
                        elapsed = time.time() - self.hover_start_time
                        if elapsed > 2.0:
                            rospy.loginfo(">> Hover Stable! Proceeding to LAND.")
                            self.state = "LANDING"
                    else:
                        # Reset timer if we drifted away
                        self.hover_start_time = None
                        rospy.loginfo_throttle(1, f"   >> Converging on Tag... Dist: {dist:.2f}m")
                else:
                    rospy.logwarn_throttle(1, "   >> Tag not visible! Holding Position...")
                    # Hold current position (or last random point) if tag lost
                    self.send_pos_cmd(self.random_target) 

            # ==========================================
            # 5. LANDING
            # ==========================================
            elif self.state == "LANDING":
                # Robust Land Command (1Hz is enough for the command itself)
                if int(time.time()) % 2 == 0:
                    self.land_pub.publish(Empty())
                
                rospy.loginfo_throttle(1, f"   >> Descending... Z: {self.current_pos[2]:.2f}m")

                if self.current_pos[2] < self.land_threshold:
                    rospy.loginfo(">> Ground Detected. Verifying...")
                    self.state = "VERIFY_LAND"
                    self.landed_start_time = time.time()

            # ==========================================
            # 6. VERIFY & HALT
            # ==========================================
            elif self.state == "VERIFY_LAND":
                # Check for bounces
                if self.current_pos[2] > self.land_threshold + 0.1:
                    rospy.logwarn("   >> Bounced! Retrying Landing...")
                    self.state = "LANDING"
                    continue

                if time.time() - self.landed_start_time > 2.0:
                    rospy.loginfo(">> MISSION SUCCESS! Disarming...")
                    self.halt_pub.publish(Empty())
                    break
                else:
                    rospy.loginfo_throttle(0.5, "   >> Verifying Stability...")

            rate.sleep()

if __name__ == '__main__':
    try:
        node = FullMission()
        node.run()
    except rospy.ROSInterruptException:
        pass

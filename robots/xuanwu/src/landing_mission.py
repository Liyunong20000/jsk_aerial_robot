#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
import time
from std_msgs.msg import Empty, Bool
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav

class LandingMission:
    def __init__(self):
        rospy.init_node('landing_mission_node')

        # --- CONFIGURATION ---
        self.ns = "/xuanwu"
        self.teleop_ns = self.ns + "/teleop_command"
        
        # Navigation
        # Start point to ensure Tag visibility (adjust as needed)
        self.search_point = np.array([1.5, 0.0, 2.0]) 
        self.approach_height = 2.0    # Stay high for the approach
        self.nav_tolerance = 0.20     # 20cm tolerance for alignment
        
        # Frames
        self.world_frame = "world"
        self.tag_frame = "land_mark"

        # --- PUBLISHERS ---
        # Same command structure as your full_mission.py
        self.start_pub = rospy.Publisher(self.teleop_ns + '/start', Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher(self.teleop_ns + '/takeoff', Empty, queue_size=1)
        self.halt_pub = rospy.Publisher(self.teleop_ns + '/halt', Empty, queue_size=1)
        
        # TRIGGER for C++ Node
        self.mpc_trigger_pub = rospy.Publisher('/mpc_planner/start_landing', Bool, queue_size=1)
        
        # Navigation Publisher
        self.nav_pub = rospy.Publisher(self.ns + '/uav/nav', FlightNav, queue_size=1)
        
        # --- SUBSCRIBERS ---
        self.odom_sub = rospy.Subscriber(self.ns + '/uav/baselink/odom', Odometry, self.odom_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # State Variables
        self.state = "INIT"
        self.current_pos = None # [x, y, z]
        self.state_timer = 0
        self.mpc_active = False

        rospy.loginfo(">> Landing Mission Initialized.")

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
        
        # Force Position Mode (as in your example)
        nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
        nav_msg.yaw_nav_mode = FlightNav.POS_MODE 
        
        nav_msg.target_pos_x = target_pos[0]
        nav_msg.target_pos_y = target_pos[1]
        nav_msg.target_pos_z = target_pos[2]
        nav_msg.target_yaw = 0.0 # Keep yaw 0 for simplicity until MPC takes over
        
        self.nav_pub.publish(nav_msg)

    def get_tag_target_high(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.world_frame, self.tag_frame, rospy.Time(0), rospy.Duration(0.1)
            )
            # Return Tag X, Tag Y, Fixed Z=2.0 (High Approach)
            return np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                self.approach_height
            ])
        except Exception:
            return None

    def run(self):
        rate = rospy.Rate(10) # 10Hz Main Loop
        
        while not rospy.is_shutdown():
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
                
                # Wait 3 seconds
                if self.state_timer > 30:
                    rospy.loginfo(">> Arming Complete. Switching to TAKEOFF.")
                    self.state = "TAKEOFF"

            # ==========================================
            # 2. TAKEOFF
            # ==========================================
            elif self.state == "TAKEOFF":
                # Check if airborne
                if self.current_pos[2] > 0.5:
                    rospy.loginfo(f">> Takeoff Detected (Z: {self.current_pos[2]:.2f}m). Moving to Search Point...")
                    self.state = "GO_SEARCH"
                else:
                    self.takeoff_pub.publish(Empty())
                    rospy.loginfo_throttle(1, "   >> Sending Takeoff...")

            # ==========================================
            # 3. GO TO SEARCH POINT (Ensure Tag Visibility)
            # ==========================================
            elif self.state == "GO_SEARCH":
                self.send_pos_cmd(self.search_point)
                dist = np.linalg.norm(self.current_pos - self.search_point)
                
                if dist < self.nav_tolerance:
                    rospy.loginfo(f">> Reached Search Point. Aligning with Tag...")
                    self.state = "ALIGN_HIGH"

            # ==========================================
            # 4. ALIGN HIGH (Python Control)
            # ==========================================
            elif self.state == "ALIGN_HIGH":
                target = self.get_tag_target_high()
                
                if target is not None:
                    # Fly to (Tag_X, Tag_Y, 2.0m)
                    self.send_pos_cmd(target)
                    
                    dist_xy = np.linalg.norm(self.current_pos[:2] - target[:2])
                    
                    rospy.loginfo_throttle(0.5, f"   >> Aligning... XY Err: {dist_xy:.2f}m")
                    
                    # If we are aligned within tolerance
                    if dist_xy < self.nav_tolerance:
                        rospy.loginfo(">> ALIGNED! ACTIVATING C++ MPC LANDING.")
                        self.state = "MPC_HANDOVER"
                else:
                    rospy.logwarn_throttle(1, "   >> Tag lost! Holding at Search Point...")
                    self.send_pos_cmd(self.search_point) 

            # ==========================================
            # 5. HANDOVER TO C++
            # ==========================================
            elif self.state == "MPC_HANDOVER":
                if not self.mpc_active:
                    # 1. Send Trigger to C++ Node
                    self.mpc_trigger_pub.publish(True)
                    self.mpc_active = True
                    rospy.loginfo(">> HANDOVER COMPLETE. Python Stopping Navigation Commands.")
                
                # 2. DO NOT publish self.nav_pub anymore.
                # The C++ node is now publishing to /xuanwu/uav/nav
                
                # 3. Monitor for completion (optional, C++ handles the Halt)
                rospy.loginfo_throttle(1.0, f"   >> C++ Landing in Progress... Z: {self.current_pos[2]:.2f}")
                
                if self.current_pos[2] < 0.15:
                     # Just a safeguard monitor
                     rospy.loginfo(">> Drone is near ground. Waiting for C++ Halt...")
                     
            rate.sleep()

if __name__ == '__main__':
    try:
        node = LandingMission()
        node.run()
    except rospy.ROSInterruptException:
        pass

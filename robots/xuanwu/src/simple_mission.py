#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
import random
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav

class SimpleMissionPos:
    def __init__(self):
        rospy.init_node('simple_mission_pos')

        # --- SETTINGS ---
        self.ns = "/xuanwu"
        self.hover_height = 0.3 # 30cm above the tag
        self.acceptance_radius = 0.15 # 15cm tolerance to switch tasks
        
        # --- FRAMES ---
        self.world_frame = "world"
        self.tag_frame = "land_mark" # As requested

        # --- RANDOM TARGET GENERATION ---
        # Center: 1, 1, 2. Radius noise: +/- 20cm
        self.target_x = 1.0 + random.uniform(-0.2, 0.2)
        self.target_y = 1.0 + random.uniform(-0.2, 0.2)
        self.target_z = 2.0
        
        # --- STATE ---
        self.state = "GO_TO_POINT" # Start state
        self.current_pos = None

        # --- ROS COMMUNICATION ---
        self.nav_pub = rospy.Publisher(self.ns + '/uav/nav', FlightNav, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.ns + '/uav/baselink/odom', Odometry, self.odom_callback)
        
        # --- TF BUFFER ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo(f"Mission Initialized. First Target: [{self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f}]")

    def odom_callback(self, msg):
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

    def send_pos_cmd(self, x, y, z):
        """Sends a Position Control command to the internal controller."""
        nav_msg = FlightNav()
        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.header.frame_id = self.world_frame
        nav_msg.control_frame = FlightNav.WORLD_FRAME
        nav_msg.target = FlightNav.COG

        # Explicitly requesting POSITION MODE
        nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
        nav_msg.yaw_nav_mode = FlightNav.POS_MODE 

        nav_msg.target_pos_x = x
        nav_msg.target_pos_y = y
        nav_msg.target_pos_z = z
        nav_msg.target_yaw = 0.0 # Face forward

        self.nav_pub.publish(nav_msg)

    def get_tag_target(self):
        """Returns [x, y, z] of the point 0.3m above the tag, or None."""
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
        rate = rospy.Rate(10) # 10Hz
        
        while not rospy.is_shutdown():
            if self.current_pos is None:
                rate.sleep()
                continue

            # --- STATE 1: GO TO RANDOM POINT ---
            if self.state == "GO_TO_POINT":
                # Send command
                self.send_pos_cmd(self.target_x, self.target_y, self.target_z)
                
                # Check distance
                dist = np.linalg.norm(self.current_pos - np.array([self.target_x, self.target_y, self.target_z]))
                
                if dist < self.acceptance_radius:
                    rospy.loginfo(">> Reached Point! Switching to Tag Tracking...")
                    self.state = "HOVER_TAG"

            # --- STATE 2: HOVER ABOVE TAG ---
            elif self.state == "HOVER_TAG":
                tag_target = self.get_tag_target()
                
                if tag_target is not None:
                    # Found tag, go to it
                    self.send_pos_cmd(tag_target[0], tag_target[1], tag_target[2])
                    
                    # Print status
                    dist_to_tag = np.linalg.norm(self.current_pos - tag_target)
                    rospy.loginfo_throttle(1, f"Tracking Tag... Error: {dist_to_tag:.3f}m")
                else:
                    # Lost tag? Stay at last known random point (or hover in place)
                    rospy.logwarn_throttle(1, "Tag not visible! Holding last position...")
                    # Ideally, we send the last valid command again, or just wait.
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = SimpleMissionPos()
        node.run()
    except rospy.ROSInterruptException:
        pass

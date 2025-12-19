#!/usr/bin/env python3
import rospy
import numpy as np
import random
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav

class SimpleGoTo:
    def __init__(self):
        rospy.init_node('simple_goto_node')

        # --- SETTINGS ---
        self.ns = "/xuanwu"
        self.kp_xy = 1.0        # XY Gain 
        self.kp_z = 1.0         # Z Gain
        self.max_vel = 0.5      # Max speed clamp (m/s)
        
        # --- TARGET GENERATION ---
        # Center: 1, 1, 2. Radius noise: +/- 20cm
        self.target_x = 1.0 + random.uniform(-0.2, 0.2)
        self.target_y = 1.0 + random.uniform(-0.2, 0.2)
        self.target_z = 2.0     
        
        # --- ROS COMMUNICATION ---
        self.nav_pub = rospy.Publisher(self.ns + '/uav/nav', FlightNav, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.ns + '/uav/baselink/odom', Odometry, self.odom_callback)

        self.current_pos = None # [x, y, z]

        rospy.loginfo(f"Target Set: [{self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f}]")
        rospy.loginfo("Waiting for Odometry to start control loop...")

    def odom_callback(self, msg):
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

    def send_velocity_command(self):
        if self.current_pos is None:
            return

        # 1. Calculate Error
        target = np.array([self.target_x, self.target_y, self.target_z])
        error = target - self.current_pos

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

        # Set VEL_MODE for Position and Z
        nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
        nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
        
        # Keep Yaw stable (POS_MODE 0.0 means face forward)
        nav_msg.yaw_nav_mode = FlightNav.POS_MODE 
        nav_msg.target_yaw = 0.0

        nav_msg.target_vel_x = vel[0]
        nav_msg.target_vel_y = vel[1]
        nav_msg.target_vel_z = vel[2]

        self.nav_pub.publish(nav_msg)

    def run(self):
        rate = rospy.Rate(20) # 20Hz Loop
        
        while not rospy.is_shutdown():
            if self.current_pos is not None:
                self.send_velocity_command()
                
                # Print distance occasionally
                dist = np.linalg.norm(self.current_pos - np.array([self.target_x, self.target_y, self.target_z]))
                if dist < 0.1:
                    rospy.loginfo_throttle(2, f"Holding Position (Error: {dist:.3f}m)")
                else:
                    rospy.loginfo_throttle(2, f"Flying... Dist to target: {dist:.2f}m")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = SimpleGoTo()
        node.run()
    except rospy.ROSInterruptException:
        pass

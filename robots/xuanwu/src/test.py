#!/usr/bin/env python3
import rospy
import math
from aerial_robot_msgs.msg import FlightNav

rospy.init_node('trajectory_test_figure8')
pub = rospy.Publisher('/xuanwu/uav/nav', FlightNav, queue_size=1)
rate = rospy.Rate(50) # Increased to 50Hz for smoother curves

rospy.loginfo("Publishing Figure-8 Trajectory...")
rospy.sleep(1.0)

start_time = rospy.Time.now()

# CONFIGURATION
AMP = 1.0       # Amplitude (Size of the 8 in meters)
SPEED = 0.8     # Speed factor (Lower is slower)
HEIGHT = 1.0    # Altitude

while not rospy.is_shutdown():
    # 1. Calculate Time
    t_raw = (rospy.Time.now() - start_time).to_sec()
    t = t_raw * SPEED  # Scale time to control speed

    msg = FlightNav()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    
    # 2. Modes (Critical!)
    msg.pos_xy_nav_mode = 4  # POS_VEL_MODE
    msg.pos_z_nav_mode = 2   # POS_MODE (Keep Z simple)
    msg.yaw_nav_mode = 2     # POS_MODE
    msg.target = 1           # COG
    msg.control_frame = 0    # WORLD_FRAME

    # 3. Figure-8 Math
    # X oscillates at frequency f
    msg.target_pos_x = AMP * math.sin(t)
    msg.target_vel_x = AMP * SPEED * math.cos(t) # Chain rule: d/dt(sin(wt)) = w*cos(wt)
    
    # Y oscillates at frequency 2f (This creates the "8" shape)
    msg.target_pos_y = AMP * math.sin(2 * t)
    msg.target_vel_y = AMP * 2 * SPEED * math.cos(2 * t)
    
    # Z (Constant height)
    msg.target_pos_z = HEIGHT
    msg.target_vel_z = 0.0

    # Yaw (Optional: Make it look at the tangent of the path)
    # math.atan2(vy, vx) gives the direction of travel
    msg.target_yaw = math.atan2(msg.target_vel_y, msg.target_vel_x)

    pub.publish(msg)
    rate.sleep()

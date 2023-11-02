#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import time
import threading
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String

# It is for  the first drone BoomBoom
# Set the global vibration
x = 0.0
y = 0.0
z = 0.0
Px = 0.0
Py = 0.0
Pz = 0.0
Px_now = 0.0
Py_now = 0.0
Pz_now = 0.0


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')
    # Press Ctrl+F8 to toggle the breakpoint.

# This is used to type the number
# Type the position of x,y,z
def get_user_input():
    global x, y, z
    x = float(input("Enter x value: "))
    y = float(input("Enter y value: "))
    z = float(input("Enter z value: "))

# Get the distance information between the drone and the Apriltag from the topic /tag_detections
def apriltag_callback(data):
    global Px, Py, Pz
    for detection in data.detections:
        tag_id = detection.id
        position = detection.pose.pose.pose.position
        Px = detection.pose.pose.pose.position.y
        Py = detection.pose.pose.pose.position.x
        Pz = detection.pose.pose.pose.position.z
        orientation = detection.pose.pose.pose.orientation
        # Px_type = type(Px)
        #print(f"AprilTag ID: {Px}  {Px_type}")
        #print(f"AprilTag ID: {tag_id}")
        #print(f"Position (x, y, z): ({Px}, {Py}, {Pz})")
        #print(f"Orientation (x, y, z, w): ({orientation.x}, {orientation.y}, {orientation.z}, {orientation.w})")
        #print("------")


def listener():
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, apriltag_callback)
    time.sleep(0.5)
    while not rospy.is_shutdown():
        rospy.spin()

def position_callback(odom_msg):
    global Px_now, Py_now, Pz_now
    Px_now = odom_msg.pose.pose.position.x
    Py_now = odom_msg.pose.pose.position.y
    Pz_now = odom_msg.pose.pose.position.z

def listener_position():
    rospy.Subscriber('/quadrotor/uav/cog/odom', Odometry, position_callback)
    time.sleep(0.5)
    while not rospy.is_shutdown():
        rospy.spin()

def publish_flight_nav():
    # Type the number
    get_user_input()
    # Creat a publishier
    pub = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
    rate = rospy.Rate(1)
    time.sleep(0.5)

    flight_nav_msg = FlightNav()
    flight_nav_msg.header.seq = 1
    flight_nav_msg.header.stamp.secs = 0
    flight_nav_msg.header.stamp.nsecs = 0
    flight_nav_msg.header.frame_id = ''
    flight_nav_msg.control_frame = 0
    flight_nav_msg.target = 0
    flight_nav_msg.pos_xy_nav_mode = 2
    flight_nav_msg.target_pos_x = x
    flight_nav_msg.target_vel_x = 0.0
    flight_nav_msg.target_acc_x = 0.0
    flight_nav_msg.target_pos_y = y
    flight_nav_msg.target_vel_y = 0.0
    flight_nav_msg.target_acc_y = 0.0
    flight_nav_msg.yaw_nav_mode = 0
    flight_nav_msg.target_omega_z = 0.0
    flight_nav_msg.target_yaw = 0.0
    flight_nav_msg.pos_z_nav_mode = 2
    flight_nav_msg.target_pos_z = z
    flight_nav_msg.target_vel_z = 0.0
    flight_nav_msg.target_pos_diff_z = 0.0

    pub.publish(flight_nav_msg)
    rate.sleep()
def land():
    pub = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)
    time.sleep(0.5)
    rospy.loginfo("Publishing land command...")
    empty_msg = Empty()
    pub.publish(empty_msg)
def land_camera():

    global Px_now,Py_now,Pz_now,Px, Py, Pz

    get_user_input()
    pub = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
    rate = rospy.Rate(1)
    time.sleep(0.5)

    flight_nav_msg = FlightNav()
    flight_nav_msg.header.seq = 1
    flight_nav_msg.header.stamp.secs = 0
    flight_nav_msg.header.stamp.nsecs = 0
    flight_nav_msg.header.frame_id = ''
    flight_nav_msg.control_frame = 0
    flight_nav_msg.target = 0
    flight_nav_msg.pos_xy_nav_mode = 2
    flight_nav_msg.target_pos_x = x
    flight_nav_msg.target_vel_x = 0.1
    flight_nav_msg.target_acc_x = 0.0
    flight_nav_msg.target_pos_y = y
    flight_nav_msg.target_vel_y = 0.0
    flight_nav_msg.target_acc_y = 0.0
    flight_nav_msg.yaw_nav_mode = 0
    flight_nav_msg.target_omega_z = 0.0
    flight_nav_msg.target_yaw = 0.0
    flight_nav_msg.pos_z_nav_mode = 2
    flight_nav_msg.target_pos_z = z
    flight_nav_msg.target_vel_z = 0.0
    flight_nav_msg.target_pos_diff_z = 0.0

    pub.publish(flight_nav_msg)
    rate.sleep()
    while not rospy.is_shutdown():
        if Px != 0 :
            if Px_now < 0:
                Px = - Px
            #print_hi('------------------------------------')
            flight_nav_msg = FlightNav()
            flight_nav_msg.header.seq = 1
            flight_nav_msg.header.stamp.secs = 0
            flight_nav_msg.header.stamp.nsecs = 0
            flight_nav_msg.header.frame_id = ''
            flight_nav_msg.control_frame = 0
            flight_nav_msg.target = 0
            flight_nav_msg.pos_xy_nav_mode = 2
            flight_nav_msg.target_pos_x = Px_now-Px
            flight_nav_msg.target_vel_x = 0.05
            flight_nav_msg.target_acc_x = 0.0
            flight_nav_msg.target_pos_y = Py_now-Py
            flight_nav_msg.target_vel_y = 0.0
            flight_nav_msg.target_acc_y = 0.0
            flight_nav_msg.yaw_nav_mode = 0
            flight_nav_msg.target_omega_z = 0.0
            flight_nav_msg.target_yaw = 0.0
            flight_nav_msg.pos_z_nav_mode = 2
            flight_nav_msg.target_pos_z = 0.5
            flight_nav_msg.target_vel_z = 0.0
            flight_nav_msg.target_pos_diff_z = 0.0

            pub.publish(flight_nav_msg)
            print_hi('------------------------------------')
            print(f"Px: {Px_now} {Px}")
            if 0.05 > Px > -0.05:
                rate = rospy.Rate(0.5)
                while not rospy.is_shutdown():
                    print_hi('^^^^^^^^^^^^^^^')
                    time.sleep(0.5)
                    flight_nav_msg = FlightNav()
                    flight_nav_msg.header.seq = 1
                    flight_nav_msg.header.stamp.secs = 0
                    flight_nav_msg.header.stamp.nsecs = 0
                    flight_nav_msg.header.frame_id = ''
                    flight_nav_msg.control_frame = 0
                    flight_nav_msg.target = 0
                    flight_nav_msg.pos_xy_nav_mode = 2
                    ppx = Px_now - Px
                    #print({ppx})
                    flight_nav_msg.target_pos_x = ppx
                    x1 = ppx
                    x2 = x1
                    #print(f"{Px_now} ---- {Px}")

                    flight_nav_msg.target_vel_x = 0.0
                    flight_nav_msg.target_acc_x = 0.0
                    flight_nav_msg.target_pos_y = Py_now - Py
                    flight_nav_msg.target_vel_y = 0.0
                    flight_nav_msg.target_acc_y = 0.0
                    flight_nav_msg.yaw_nav_mode = 0
                    flight_nav_msg.target_omega_z = 0.0
                    flight_nav_msg.target_yaw = 0.0
                    flight_nav_msg.pos_z_nav_mode = 2
                    flight_nav_msg.target_pos_z = 0.3
                    flight_nav_msg.target_vel_z = 0.0
                    flight_nav_msg.target_pos_diff_z = 0.0
                    pub.publish(flight_nav_msg)
                    #print(f"Px: {Px_now} {Px}")
                    time.sleep(0.5)
                    if (-0.02< Px < 0.02) :
                        print("~~~~~~~~~~")
                        flight_nav_msg = FlightNav()
                        flight_nav_msg.header.seq = 1
                        flight_nav_msg.header.stamp.secs = 0
                        flight_nav_msg.header.stamp.nsecs = 0
                        flight_nav_msg.header.frame_id = ''
                        flight_nav_msg.control_frame = 0
                        flight_nav_msg.target = 0
                        flight_nav_msg.pos_xy_nav_mode = 2
                        flight_nav_msg.target_pos_x = x2
                        flight_nav_msg.target_vel_x = 0.0
                        flight_nav_msg.target_acc_x = 0.0
                        flight_nav_msg.target_pos_y = Py_now - Py
                        flight_nav_msg.target_vel_y = 0.0
                        flight_nav_msg.target_acc_y = 0.0
                        flight_nav_msg.yaw_nav_mode = 0
                        flight_nav_msg.target_omega_z = 0.0
                        flight_nav_msg.target_yaw = 0.0
                        flight_nav_msg.pos_z_nav_mode = 2
                        flight_nav_msg.target_pos_z = 0.1
                        flight_nav_msg.target_vel_z = 0.0
                        flight_nav_msg.target_pos_diff_z = 0.0
                        pub.publish(flight_nav_msg)
                        if -0.02< Py < 0.02:
                            land()
                            time.sleep(1)

                            sys.exit()


# break


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('BoomBoom')
    rospy.init_node('demo', anonymous=True)
    subscriber_thread = threading.Thread(target=listener)
    subscriber_thread_position = threading.Thread(target=listener_position)
    #publish_threasubscriber_thread.start()d = threading.Thread(target=publish_flight_nav)
    #publish_flight_nav_camera = threading.Thread(target=publish_flight_nav_camera)
    try:
        #listener()

        print_hi('Please type the start point: x, y,z')
        #publish_thread.start()
        publish_flight_nav()
        rospy.sleep(5)
        print_hi('BoomBoom reached the start point')
        subscriber_thread.start()
        subscriber_thread_position.start()
        print_hi('Please type the end point: x, y,z')
        land_camera()
        rospy.sleep(5)
        print_hi('BoomBoom reached the end point')
        # print_hi('BoomBoom reached the end point');
    except rospy.ROSInterruptException:
        pass
# See PyCharm help at https://www.jetbrains.com/help/pycharm/

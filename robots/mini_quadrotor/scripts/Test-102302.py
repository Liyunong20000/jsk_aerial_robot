#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import time
import subprocess
from std_msgs.msg import String
from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray

# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
x = 0.0
y = 0.0
z = 0.0


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')
    # Press Ctrl+F8 to toggle the breakpoint.


# Type the position of x,y,z
def get_user_input():
    global x, y, z
    x = float(input("Enter x value: "))
    y = float(input("Enter y value: "))
    z = float(input("Enter z value: "))


def apriltag_callback(data):
    for detection in data.detections:
        # 获取AprilTag的ID和位置信息
        tag_id = detection.id[0]
        position = detection.pose.pose.position
        orientation = detection.pose.pose.orientation

        # 打印位置信息
        print(f"AprilTag ID: {tag_id}")
        print(f"Position (x, y, z): ({position.x}, {position.y}, {position.z})")
        print(f"Orientation (x, y, z, w): ({orientation.x}, {orientation.y}, {orientation.z}, {orientation.w})")
        print("------")


def listener():
    rospy.init_node('apriltag_listener', anonymous=True)
    rospy.Subscriber('/apriltag/detections', AprilTagDetectionArray, callback)
    rospy.spin()


def publish_flight_nav():
    # 初始化ROS节点
    get_user_input()

    # 创建一个发布者，发布FlightNav类型的消息到指定话题
    pub = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
    rate = rospy.Rate(1)
    time.sleep(0.5)
    # while not rospy.is_shutdown():
    # 创建一个FlightNav消息对象并设置其内容

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

    # 发布消息
    pub.publish(flight_nav_msg)
    rate.sleep()

def publish_flight_nav_camera():
    # 初始化ROS节点
    get_user_input()

    # 创建一个发布者，发布FlightNav类型的消息到指定话题
    pub = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
    rate = rospy.Rate(1)
    time.sleep(0.5)
    # while not rospy.is_shutdown():
    # 创建一个FlightNav消息对象并设置其内容

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

    # 发布消息
    pub.publish(flight_nav_msg)
    rate.sleep()

# break


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('BoomBoom')
    rospy.init_node('flight_nav_publisher', anonymous=True)

    try:
        print_hi('Please type the start point: x, y,z')
        publish_flight_nav()
        #rospy.sleep(5)
        print_hi('BoomBoom reached the start point')
        print_hi('Please type the end point: x, y,z')
        publish_flight_nav_camera()
        print_hi('BoomBoom reached the end point')
        # print_hi('BoomBoom reached the end point');
    except rospy.ROSInterruptException:
        pass
# See PyCharm help at https://www.jetbrains.com/help/pycharm/

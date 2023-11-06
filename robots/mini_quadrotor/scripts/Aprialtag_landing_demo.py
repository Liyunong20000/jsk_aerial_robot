#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import time
import math
import tf2_ros
import tf
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

# It is for  the first drone BoomBoom
# use the class to create a node

class AprillandNode:

    def __init__(self): # This part will work when this node is used.
        print(f'Hi, I am BoomBoom. Today is a day._WJY')
        rospy.init_node('Aprilland', anonymous=True)

        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.Rx, self.Ry, self.Rz = 0.0, 0.0, 0.0
        self.Px, self.Py, self.Pz = 0.0, 0.0, 0.0

        # Subscribe and publish.
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.apriltag_callback)
        rospy.Subscriber('/quadrotor/uav/cog/odom', Odometry, self.position_callback)
        self.pub_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
        self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        # Initialize a TransformListener
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        #transformation

    def get_user_input(self):  # Use to input the number
        self.x = float(input("Enter x value: "))
        self.y = float(input("Enter y value: "))
        self.z = float(input("Enter z value: "))


    def land(self):  # Use to land
        time.sleep(0.5)
        rospy.loginfo("Publishing land command...")
        empty_msg = Empty()
        self.pub_land.publish(empty_msg)

    def apriltag_callback(self,data):
        # Get the information of the distance between the drone and the apriltag
        while not rospy.is_shutdown():
            try:
                transform = self.tf_buffer.lookup_transform('world', 'Target', rospy.Time(0), rospy.Duration(0.1))
                self.Rx = transform.transform.translation.x
                self.Ry = transform.transform.translation.y
                self.Rz = transform.transform.translation.z
            except :
                continue


        #print(f'{self.Rx}  {self.Ry}  {self.Rz}')


    def position_callback(self, odom_msg): # Get the position information of the drone
        self.Px = odom_msg.pose.pose.position.x
        self.Py = odom_msg.pose.pose.position.y
        self.Pz = odom_msg.pose.pose.position.z

    def publish_flight_nav(self): # Set a beginning point
        print(f'Type the begin point')
        # Type the number
        self.get_user_input()

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
        flight_nav_msg.target_pos_x = self.x
        flight_nav_msg.target_vel_x = 0.0
        flight_nav_msg.target_acc_x = 0.0
        flight_nav_msg.target_pos_y = self.y
        flight_nav_msg.target_vel_y = 0.0
        flight_nav_msg.target_acc_y = 0.0
        flight_nav_msg.yaw_nav_mode = 0
        flight_nav_msg.target_omega_z = 0.0
        flight_nav_msg.target_yaw = 0.0
        flight_nav_msg.pos_z_nav_mode = 2
        flight_nav_msg.target_pos_z = self.z
        flight_nav_msg.target_vel_z = 0.0
        flight_nav_msg.target_pos_diff_z = 0.0

        self.pub_nav.publish(flight_nav_msg)
        rate.sleep()
        while not rospy.is_shutdown():
            if -0.03 < self.Px- self.x < 0.03:
                if -0.03< self.Py - self. y < 0.03:
                    print(f'I have arrived!')
                    break
    def land_camera(self):
        print(f'Type the goal point')
        self.get_user_input()
        rate = rospy.Rate(1)
        time.sleep(0.5)

        flight_nav_msg = FlightNav()
        flight_nav_msg.header.seq = 1
        flight_nav_msg.pos_xy_nav_mode = 2
        flight_nav_msg.target_pos_x = self.x
        flight_nav_msg.target_pos_y = self.y
        flight_nav_msg.pos_z_nav_mode = 2
        flight_nav_msg.target_pos_z = self.z

        self.pub_nav.publish(flight_nav_msg)
        rate.sleep()
        while not rospy.is_shutdown():
            if (self.Rx + self.Ry) != 0:
                flight_nav_msg = FlightNav()
                flight_nav_msg.header.seq = 1
                flight_nav_msg.pos_xy_nav_mode = 2
                flight_nav_msg.target_pos_x = self.Rx
                flight_nav_msg.target_pos_y = self.Ry
                flight_nav_msg.pos_z_nav_mode = 2
                flight_nav_msg.target_pos_z = 0.5

                self.pub_nav.publish(flight_nav_msg)
                if -0.05 < math.sqrt(self.Rx **2 + self.Ry **2) < 0.05:
                    rate = rospy.Rate(0.5)
                    while not rospy.is_shutdown():
                        time.sleep(1)
                        flight_nav_msg = FlightNav()
                        flight_nav_msg.header.seq = 1
                        flight_nav_msg.pos_xy_nav_mode = 2
                        flight_nav_msg.target_pos_x = self.Rx
                        flight_nav_msg.target_pos_y = self.Ry
                        flight_nav_msg.pos_z_nav_mode = 2
                        flight_nav_msg.target_pos_z = 0.3
                        self.pub_nav.publish(flight_nav_msg)
                        time.sleep(0.5)
                        if (-0.02 < math.sqrt(self.Rx **2 + self.Ry **2) < 0.02):
                            while not rospy.is_shutdown():
                                flight_nav_msg = FlightNav()
                                flight_nav_msg.header.seq = 1
                                flight_nav_msg.pos_xy_nav_mode = 2
                                flight_nav_msg.target_pos_x = self.Rx
                                flight_nav_msg.target_pos_y = self.Ry
                                flight_nav_msg.pos_z_nav_mode = 2
                                flight_nav_msg.target_pos_z = 0.1
                                self.pub_nav.publish(flight_nav_msg)
                                time.sleep(0.5)
                                if -0.01 < math.sqrt(self.Rx **2 + self.Ry **2) < 0.01:
                                    print(f'Safe landing!')
                                    print(f'X = {self.Px}, Y = {self.Py}')
                                    time.sleep(0.2)
                                    self.land()
                                    time.sleep(1)
                                    sys.exit()


if __name__ == '__main__':
    node = AprillandNode()
    time.sleep(0.5)
    node.publish_flight_nav()
    node.land_camera()
    while not rospy.is_shutdown():
        rospy.spin()

#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

class AutoTakeoffHover:
    def __init__(self):
        rospy.init_node("auto_takeoff_hover")

        # --- CONFIGURATION (Matches your keyboard script) ---
        robot_ns = "/xuanwu" # Namespace
        ns = robot_ns + "/teleop_command"
        
        # --- PUBLISHERS ---
        # 'r' - Arming
        self.start_pub = rospy.Publisher(ns + '/start', Empty, queue_size=1)
        # 't' - Takeoff
        self.takeoff_pub = rospy.Publisher(ns + '/takeoff', Empty, queue_size=1)
        # 'x' - Task Start (Important for some state machines)
        self.motion_start_pub = rospy.Publisher('task_start', Empty, queue_size=1)
        
        # --- SUBSCRIBER ---
        # To verify we actually reached hover height
        self.odom_sub = rospy.Subscriber(robot_ns + '/uav/baselink/odom', Odometry, self.odom_callback)

        self.current_z = 0.0
        self.is_flying = False

        rospy.loginfo("Initialized. Waiting for connection...")
        rospy.sleep(1.0) 

    def odom_callback(self, msg):
        self.current_z = msg.pose.pose.position.z

    def run(self):
        # ==========================================
        # STEP 1: ARMING (Equivalent to pressing 'r')
        # ==========================================
        rospy.loginfo("1. ARMING MOTORS (Sending 'start')...")
        
        # Send a few times to ensure receipt
        for _ in range(3):
            self.start_pub.publish(Empty())
            rospy.sleep(0.2)
            
        rospy.loginfo("   >> Waiting 3s for motors to spin up...")
        rospy.sleep(3.0)

        # ==========================================
        # STEP 2: TAKEOFF (Equivalent to pressing 't')
        # ==========================================
        rospy.loginfo("2. TAKEOFF (Sending 'takeoff')...")
        
        # We loop and send 'takeoff' until the drone physically rises
        rate = rospy.Rate(5) # 5 Hz
        while not rospy.is_shutdown():
            
            if self.current_z > 0.5:
                rospy.loginfo("   >> Takeoff Detected (Altitude > 0.5m)")
                self.is_flying = True
                break
            
            self.takeoff_pub.publish(Empty())
            rospy.loginfo_throttle(1, "   >> Sending Takeoff cmd... (Z: {:.2f}m)".format(self.current_z))
            rate.sleep()

        # ==========================================
        # STEP 3: ENABLE NAV (Equivalent to pressing 'x')
        # ==========================================
        # Sometimes needed to switch state to "Hover" or "Nav"
        rospy.loginfo("3. SENDING TASK START (Sending 'x')...")
        self.motion_start_pub.publish(Empty())

        # ==========================================
        # STEP 4: MONITOR HOVER
        # ==========================================
        rospy.loginfo("4. SYSTEM IN HOVER MODE. MONITORING...")
        
        while not rospy.is_shutdown():
            rospy.loginfo_throttle(1, "   >> HOVERING. Altitude: {:.2f}m".format(self.current_z))
            
            # Safety Check: If it drops unexpectedly, boost it
            if self.current_z < 0.3:
                rospy.logwarn("   >> ALTITUDE DROP! Re-sending Takeoff command!")
                self.takeoff_pub.publish(Empty())
                
            rate.sleep()

if __name__ == "__main__":
    try:
        node = AutoTakeoffHover()
        node.run()
    except rospy.ROSInterruptException:
        pass

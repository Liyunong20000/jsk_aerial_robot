#!/usr/bin/env python
import rospy
import time
import numpy as np
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

class AutoLandVerify:
    def __init__(self):
        rospy.init_node("auto_land_verify")

        # --- CONFIGURATION ---
        self.ns = "/xuanwu"
        self.land_threshold = 0.20  # Height (m) to consider "on the ground"
        self.stable_duration = 2.0  # Seconds to wait to confirm landing
        
        # --- PUBS ---
        self.land_pub = rospy.Publisher(self.ns + '/teleop_command/land', Empty, queue_size=1)
        self.halt_pub = rospy.Publisher(self.ns + '/teleop_command/halt', Empty, queue_size=1) # Disarm safety
        
        # --- SUBS ---
        self.odom_sub = rospy.Subscriber(self.ns + '/uav/baselink/odom', Odometry, self.odom_callback)

        self.current_z = None
        self.state = "INIT"
        self.landed_time_start = None

        rospy.loginfo(">> Landing Node Initialized. Waiting for Odom...")

    def odom_callback(self, msg):
        self.current_z = msg.pose.pose.position.z

    def run(self):
        rate = rospy.Rate(5) # 5 Hz loop
        
        while not rospy.is_shutdown():
            # Wait for data
            if self.current_z is None:
                rate.sleep()
                continue

            # --- STATE 1: INIT ---
            if self.state == "INIT":
                if self.current_z > self.land_threshold:
                    rospy.loginfo(f">> Drone Detected at {self.current_z:.2f}m. Initiating Landing Sequence...")
                    self.state = "DESCENDING"
                else:
                    rospy.loginfo(f">> Drone already on ground ({self.current_z:.2f}m). Nothing to do.")
                    return

            # --- STATE 2: DESCENDING ---
            elif self.state == "DESCENDING":
                # Send Land command repeatedly (every loop is too fast, lets do every 1s)
                if int(time.time()) % 2 == 0: 
                    self.land_pub.publish(Empty())
                
                rospy.loginfo_throttle(1, f"   >> Descending... Current Z: {self.current_z:.2f}m")

                # Check if we hit the ground
                if self.current_z < self.land_threshold:
                    rospy.loginfo("   >> Ground Detected! Verifying stability...")
                    self.landed_time_start = time.time()
                    self.state = "VERIFYING"

            # --- STATE 3: VERIFYING ---
            elif self.state == "VERIFYING":
                # If it pops back up, go back to descending
                if self.current_z > self.land_threshold + 0.1:
                    rospy.logwarn("   >> Drone bounced! Resuming Land Command.")
                    self.state = "DESCENDING"
                    continue

                # Check how long we've been down
                elapsed = time.time() - self.landed_time_start
                if elapsed > self.stable_duration:
                    rospy.loginfo(">> LANDING CONFIRMED. SUCCESS.")
                    
                    # Optional: Send Halt (Disarm) to ensure motors stop
                    self.halt_pub.publish(Empty())
                    rospy.loginfo(">> Sent Halt (Disarm). Exiting.")
                    break
                else:
                    rospy.loginfo_throttle(0.5, f"   >> Verifying... {elapsed:.1f}/{self.stable_duration}s")

            rate.sleep()

if __name__ == "__main__":
    try:
        node = AutoLandVerify()
        node.run()
    except rospy.ROSInterruptException:
        pass

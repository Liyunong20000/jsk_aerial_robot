// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, DRAGON Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <spinal/ServoControlCmd.h>

class XuanwuJoyGripperNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  ros::Subscriber joy_sub_;
  ros::Publisher gripper_cmd_pub_;

  int gripper_servo_index_;
  int gripper_open_angle_;
  int gripper_close_angle_;
  std::string gripper_cmd_topic_;

  bool prev_l1_button_;
  bool prev_r1_button_;

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void publishGripperCmd(int target_angle, const std::string& command_label);

public:
  XuanwuJoyGripperNode(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~XuanwuJoyGripperNode() = default;
};

XuanwuJoyGripperNode::XuanwuJoyGripperNode(ros::NodeHandle nh, ros::NodeHandle nhp)
  : nh_(nh), nhp_(nhp),
    prev_l1_button_(false),
    prev_r1_button_(false)
{
  // Load parameters
  nhp_.param<int>("gripper_servo_index", gripper_servo_index_, 0);
  nhp_.param<int>("gripper_open_angle", gripper_open_angle_, 1400);
  nhp_.param<int>("gripper_close_angle", gripper_close_angle_, 200);
  nhp_.param<std::string>("gripper_cmd_topic", gripper_cmd_topic_, "servo/target_states");

  ROS_INFO("Xuanwu Joy Gripper Node initialized");
  ROS_INFO("  Servo index: %d", gripper_servo_index_);
  ROS_INFO("  Open angle: %d, Close angle: %d", gripper_open_angle_, gripper_close_angle_);
  ROS_INFO("  Gripper command topic: %s", gripper_cmd_topic_.c_str());

  // Create subscribers and publishers
  joy_sub_ = nh_.subscribe("joy", 1, &XuanwuJoyGripperNode::joyCallback, this);
  gripper_cmd_pub_ = nh_.advertise<spinal::ServoControlCmd>(gripper_cmd_topic_, 1);
}

void XuanwuJoyGripperNode::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if (joy_msg->buttons.size() < 6)
  {
    ROS_WARN_THROTTLE(5.0, "Joy message has insufficient buttons: %zu", joy_msg->buttons.size());
    return;
  }

  // PS4 controller button mapping (from joy node):
  // L1 (LB) = buttons[4], R1 (RB) = buttons[5]
  bool l1_pressed = (joy_msg->buttons[4] == 1);
  bool r1_pressed = (joy_msg->buttons[5] == 1);

  // Check for rising edge (button press transition from 0 to 1)
  if (l1_pressed && !prev_l1_button_)
  {
    ROS_DEBUG("L1 button pressed");
    publishGripperCmd(gripper_close_angle_, "close");
  }

  if (r1_pressed && !prev_r1_button_)
  {
    ROS_DEBUG("R1 button pressed");
    publishGripperCmd(gripper_open_angle_, "open");
  }

  prev_l1_button_ = l1_pressed;
  prev_r1_button_ = r1_pressed;
}

void XuanwuJoyGripperNode::publishGripperCmd(int target_angle, const std::string& command_label)
{
  if (gripper_cmd_pub_.getNumSubscribers() == 0)
  {
    ROS_WARN_THROTTLE(1.0, "Gripper command has no subscribers on topic: %s", gripper_cmd_topic_.c_str());
  }

  spinal::ServoControlCmd gripper_cmd;
  gripper_cmd.index.push_back(static_cast<uint8_t>(gripper_servo_index_));
  gripper_cmd.angles.push_back(static_cast<int16_t>(target_angle));
  gripper_cmd_pub_.publish(gripper_cmd);

  ROS_INFO("Joy gripper %s command: servo index %d, angle %d",
           command_label.c_str(), gripper_servo_index_, target_angle);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xuanwu_joy_gripper_node");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  XuanwuJoyGripperNode node(nh, nhp);

  ros::spin();

  return 0;
}


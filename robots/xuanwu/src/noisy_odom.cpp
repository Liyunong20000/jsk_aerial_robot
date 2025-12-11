#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_datatypes.h>

#include <random>
#include <string>
#include <algorithm>

// =======================
// Hardcoded configuration
// =======================

// Gazebo model name (from /gazebo/model_states)
static const std::string MODEL_NAME = "xuanwu";

// Noise for /xuanwu/noisy-odom (from /xuanwu/uav/baselink/odom)
static constexpr double POS_NOISE_STDDEV      = 0.2;   // [m]  strong position noise
static constexpr double ORIENT_NOISE_STDDEV   = 0.005;  // [rad] small orientation noise

// Noise for /qikin/camera-odom (from /gazebo/model_states)
static constexpr double CAMERA_POS_NOISE_STDDEV    = 0.005;   // [m]
static constexpr double CAMERA_ORIENT_NOISE_STDDEV = 0.01;  // [rad]

// Camera odom publish rate
static constexpr double CAMERA_ODOM_RATE_HZ = 10.0;  // [Hz]

// =======================
// Global state
// =======================

ros::Publisher noisy_odom_pub;
ros::Publisher camera_odom_pub;

nav_msgs::Odometry last_camera_odom;
bool has_model_state = false;

// Random engine and distributions
std::random_device rd;
std::mt19937 gen(rd());
std::normal_distribution<double> pos_noise_dist(0.0, POS_NOISE_STDDEV);
std::normal_distribution<double> orient_noise_dist(0.0, ORIENT_NOISE_STDDEV);
std::normal_distribution<double> camera_pos_noise_dist(0.0, CAMERA_POS_NOISE_STDDEV);
std::normal_distribution<double> camera_orient_noise_dist(0.0, CAMERA_ORIENT_NOISE_STDDEV);

// =======================
// Callbacks
// =======================

// /xuanwu/uav/baselink/odom  ->  /xuanwu/noisy-odom
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  nav_msgs::Odometry noisy = *msg;

  // Add noise to position
  noisy.pose.pose.position.x += pos_noise_dist(gen);
  noisy.pose.pose.position.y += pos_noise_dist(gen);
  noisy.pose.pose.position.z += pos_noise_dist(gen);

  // Add noise to orientation (convert to RPY, perturb, convert back)
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);

  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  roll  += orient_noise_dist(gen);
  pitch += orient_noise_dist(gen);
  yaw   += orient_noise_dist(gen);

  tf::Quaternion q_noisy;
  q_noisy.setRPY(roll, pitch, yaw);
  q_noisy.normalize();
  tf::quaternionTFToMsg(q_noisy, noisy.pose.pose.orientation);

  noisy.header.stamp = ros::Time::now();

  noisy_odom_pub.publish(noisy);
}

// /gazebo/model_states  -> store odom for MODEL_NAME
void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  auto it = std::find(msg->name.begin(), msg->name.end(), MODEL_NAME);
  if (it == msg->name.end())
  {
    ROS_WARN_THROTTLE(5.0, "Model '%s' not found in /gazebo/model_states", MODEL_NAME.c_str());
    return;
  }

  size_t idx = std::distance(msg->name.begin(), it);

  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "world";      // You can change if needed
  odom.child_frame_id  = MODEL_NAME;   // e.g., "xuanwu"

  odom.pose.pose  = msg->pose[idx];
  odom.twist.twist = msg->twist[idx];

  last_camera_odom = odom;
  has_model_state = true;
}

// Timer: publish /qikin/camera-odom at fixed rate with small noise
void cameraTimerCallback(const ros::TimerEvent&)
{
  if (!has_model_state)
    return;

  nav_msgs::Odometry noisy = last_camera_odom;
  noisy.header.stamp = ros::Time::now();

  // Add small noise to position
  noisy.pose.pose.position.x += camera_pos_noise_dist(gen);
  noisy.pose.pose.position.y += camera_pos_noise_dist(gen);
  noisy.pose.pose.position.z += camera_pos_noise_dist(gen);

  // Add small noise to orientation
  tf::Quaternion q;
  tf::quaternionMsgToTF(last_camera_odom.pose.pose.orientation, q);

  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  roll  += camera_orient_noise_dist(gen);
  pitch += camera_orient_noise_dist(gen);
  yaw   += camera_orient_noise_dist(gen);

  tf::Quaternion q_noisy;
  q_noisy.setRPY(roll, pitch, yaw);
  q_noisy.normalize();
  tf::quaternionTFToMsg(q_noisy, noisy.pose.pose.orientation);

  camera_odom_pub.publish(noisy);
}

// =======================
// main
// =======================

int main(int argc, char** argv)
{
  ros::init(argc, argv, "noisy_odom");
  ros::NodeHandle nh;

  // Publishers
  noisy_odom_pub  = nh.advertise<nav_msgs::Odometry>("/xuanwu/noisy_odom", 10);
  camera_odom_pub = nh.advertise<nav_msgs::Odometry>("/qikin/camera_odom", 10);

  // Subscribers
  ros::Subscriber odom_sub = nh.subscribe("/xuanwu/uav/baselink/odom", 10, odomCallback);
  ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 10, modelStatesCallback);

  // Timer for camera odom publishing
  double rate = CAMERA_ODOM_RATE_HZ;
  if (rate <= 0.0)
  {
    ROS_WARN("CAMERA_ODOM_RATE_HZ <= 0, defaulting to 10 Hz");
    rate = 10.0;
  }
  ros::Timer camera_timer = nh.createTimer(ros::Duration(1.0 / rate), cameraTimerCallback);

  ROS_INFO("noisy_odom_node started.");
  ROS_INFO("  MODEL_NAME: %s", MODEL_NAME.c_str());
  ROS_INFO("  POS_NOISE_STDDEV: %f", POS_NOISE_STDDEV);
  ROS_INFO("  ORIENT_NOISE_STDDEV: %f", ORIENT_NOISE_STDDEV);
  ROS_INFO("  CAMERA_POS_NOISE_STDDEV: %f", CAMERA_POS_NOISE_STDDEV);
  ROS_INFO("  CAMERA_ORIENT_NOISE_STDDEV: %f", CAMERA_ORIENT_NOISE_STDDEV);
  ROS_INFO("  CAMERA_ODOM_RATE_HZ: %f", rate);

  ros::spin();
  return 0;
}

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>

#include <fstream>
#include <string>
#include <algorithm>
#include <iomanip>

class OdomLogger
{
public:
  OdomLogger()
    : nh_(),
      model_name_("xuanwu")
  {
    // Open CSV files (overwrite each run)
    openFile(uav_baselink_file_,  "uav_baselink_odom.csv");
    openFile(noisy_file_,         "noisy_odom.csv");
    openFile(camera_file_,        "camera_odom.csv");
    openFile(gazebo_file_,        "gazebo_xuanwu_odom.csv");
    openFile(ekf_file_,           "ekf_odom.csv");

    const std::string header = "stamp,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz";

    writeHeader(uav_baselink_file_, header);
    writeHeader(noisy_file_,        header);
    writeHeader(camera_file_,       header);
    writeHeader(gazebo_file_,       header);
    writeHeader(ekf_file_,          header);

    // Subscribers
    sub_uav_baselink_ = nh_.subscribe("/xuanwu/uav/baselink/odom", 1000,
                                      &OdomLogger::uavBaselinkCallback, this);
    sub_noisy_ = nh_.subscribe("/xuanwu/noisy_odom", 1000,
                               &OdomLogger::noisyCallback, this);
    sub_camera_ = nh_.subscribe("/qikin/camera_odom", 1000,
                                &OdomLogger::cameraCallback, this);
    sub_model_states_ = nh_.subscribe("/gazebo/model_states", 1000,
                                      &OdomLogger::modelStatesCallback, this);
    sub_ekf_ = nh_.subscribe("/xuanwu/eskf/odom", 1000,
                             &OdomLogger::ekfCallback, this);

    ROS_INFO("OdomLogger started. Logging CSVs in current directory.");
  }

  ~OdomLogger()
  {
    if (uav_baselink_file_.is_open()) uav_baselink_file_.close();
    if (noisy_file_.is_open())        noisy_file_.close();
    if (camera_file_.is_open())       camera_file_.close();
    if (gazebo_file_.is_open())       gazebo_file_.close();
    if (ekf_file_.is_open())          ekf_file_.close();
  }

private:
  ros::NodeHandle nh_;

  ros::Subscriber sub_uav_baselink_;
  ros::Subscriber sub_noisy_;
  ros::Subscriber sub_camera_;
  ros::Subscriber sub_model_states_;
  ros::Subscriber sub_ekf_;

  std::ofstream uav_baselink_file_;
  std::ofstream noisy_file_;
  std::ofstream camera_file_;
  std::ofstream gazebo_file_;
  std::ofstream ekf_file_;

  std::string model_name_;

  // Utility: open file
  void openFile(std::ofstream& f, const std::string& filename)
  {
    f.open(filename.c_str(), std::ios::out | std::ios::trunc);
    if (!f.is_open())
    {
      ROS_ERROR("Failed to open file: %s", filename.c_str());
    }
  }

  // Utility: write CSV header
  void writeHeader(std::ofstream& f, const std::string& header)
  {
    if (f.is_open())
    {
      f << header << '\n';
      f.flush();
    }
  }

  // Utility: log odometry to CSV
  void logOdom(std::ofstream& f, const ros::Time& stamp,
               const nav_msgs::Odometry& odom_msg)
  {
    if (!f.is_open())
      return;

    const auto& p = odom_msg.pose.pose.position;
    const auto& q = odom_msg.pose.pose.orientation;
    const auto& v = odom_msg.twist.twist.linear;
    const auto& w = odom_msg.twist.twist.angular;

    f << std::fixed << std::setprecision(9)
      << stamp.toSec() << ","
      << p.x << "," << p.y << "," << p.z << ","
      << q.x << "," << q.y << "," << q.z << "," << q.w << ","
      << v.x << "," << v.y << "," << v.z << ","
      << w.x << "," << w.y << "," << w.z
      << '\n';

    f.flush();  // simple & safe for debugging; you can remove if performance is an issue
  }

  // Callbacks for each topic
  void uavBaselinkCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    logOdom(uav_baselink_file_, msg->header.stamp, *msg);
  }

  void noisyCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    logOdom(noisy_file_, msg->header.stamp, *msg);
  }

  void cameraCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    logOdom(camera_file_, msg->header.stamp, *msg);
  }

  void ekfCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    logOdom(ekf_file_, msg->header.stamp, *msg);
  }

  void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
    // Find the index of model_name_ ("xuanwu")
    auto it = std::find(msg->name.begin(), msg->name.end(), model_name_);
    if (it == msg->name.end())
    {
      ROS_WARN_THROTTLE(5.0, "Model '%s' not found in /gazebo/model_states",
                        model_name_.c_str());
      return;
    }

    size_t idx = std::distance(msg->name.begin(), it);

    nav_msgs::Odometry odom;
    odom.pose.pose  = msg->pose[idx];
    odom.twist.twist = msg->twist[idx];

    // There is no header in ModelStates; use current ROS time
    ros::Time t = ros::Time::now();
    logOdom(gazebo_file_, t, odom);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_logger_node");
  OdomLogger logger;
  ros::spin();
  return 0;
}

// eskf_node.cpp

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include <apriltag_ros/AprilTagDetectionArray.h>

#include <dynamic_reconfigure/server.h>
#include <xuanwu/EskfConfig.h>

// TF2 Includes (The new engine)
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "state_wrappers.hpp"  // for ERR_STATE_DIM, NOM_STATE_DIM
#include "eskf.hpp"

// ============================= ROS façade ==================================
class EskfNode
{
public:
  explicit EskfNode(ros::NodeHandle& nh)
    : nh_(nh)
    , tf_listener_(tf_buffer_)
    , filter_(loadQProcDiag(nh_), loadRCamDiag(nh_), loadRBoxDiag(nh_))
    , tag_pose_world_(loadTagPose(nh_))
  {
    // --- Topics ---
    nh_.param<std::string>("box_odom_topic",   box_topic_,   std::string("/xuanwu/noisy_odom"));
    nh_.param<std::string>("cam_odom_topic",   cam_topic_,   std::string("/xuanwu/tag_detections"));
    nh_.param<std::string>("fused_odom_topic", fused_topic_, std::string("/xuanwu/eskf/odom"));
    nh_.param<int>("target_tag_id", target_tag_id_, 0);
    nh_.param<float>("camera_z_bias", camera_z_bias_, 0.0);
    nh_.param<std::string>("imu_frame_name", body_frame_id_, std::string("xuanwu/lidar_imu"));
    nh_.param<std::string>("target_tag_frame_name", target_tag_frame_name_, std::string("land_mark"));

    box_sub_   = nh_.subscribe(box_topic_, 10, &EskfNode::boxCallback, this);
    cam_sub_   = nh_.subscribe(cam_topic_, 10, &EskfNode::camCallback, this);
    fused_pub_ = nh_.advertise<nav_msgs::Odometry>(fused_topic_, 10);
    debug_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/xuanwu/debug/vision_pose", 10);
    
    dr_callback_ = boost::bind(&EskfNode::reconfigureCallback, this, _1, _2);
    dr_server_.setCallback(dr_callback_);
  }

private:
  ros::NodeHandle nh_;

  // --- TF2 Members ---
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber box_sub_;
  ros::Subscriber cam_sub_;
  ros::Publisher  fused_pub_;
  ros::Publisher  debug_pub_;
  std::string box_topic_, cam_topic_, fused_topic_;
  std::string target_tag_frame_name_; 
  int target_tag_id_;

  float camera_z_bias_ = 0.0;

  std::string world_frame_id_ = "world";
  std::string body_frame_id_;

  Eskf filter_;
  ros::Time last_time_;

  Eigen::Affine3d tag_pose_world_;

  // Dynamic Reconfigure Members
  dynamic_reconfigure::Server<your_package_name::EskfConfig> dr_server_;
  dynamic_reconfigure::Server<your_package_name::EskfConfig>::CallbackType dr_callback_;

  // ------- Param loaders for noise diagonals --------
  static Eigen::Matrix<double, ERR_STATE_DIM, 1>
  loadQProcDiag(ros::NodeHandle& nh)
  {
    Eigen::Matrix<double, ERR_STATE_DIM, 1> diag;
    std::vector<double> vec;
    if (nh.getParam("Q_proc_diag", vec) && vec.size() == ERR_STATE_DIM)
    {
      for (size_t i = 0; i < ERR_STATE_DIM; ++i)
        diag(i) = vec[i];
    }
    else
    {
      ROS_WARN_STREAM("ESKF: using default Q_proc_diag");
      diag <<
        1e-4, 1e-4, 1e-4,
        1e-5, 1e-5, 1e-5,
        1e-3, 1e-3, 1e-3,
        1e-3, 1e-3, 1e-3;
    }
    return diag;
  }

  static Eigen::Matrix<double, 6, 1>
  loadRCamDiag(ros::NodeHandle& nh)
  {
    Eigen::Matrix<double, 6, 1> diag;
    std::vector<double> vec;
    if (nh.getParam("R_cam_diag", vec) && vec.size() == 6)
    {
      for (size_t i = 0; i < 6; ++i)
        diag(i) = vec[i];
    }
    else
    {
      ROS_WARN_STREAM("ESKF: using default R_cam_diag");
      diag <<
        1e-4, 1e-4, 1e-4,
        1e-4, 1e-4, 1e-4;
    }
    return diag;
  }

  static Eigen::Matrix<double, ERR_STATE_DIM, 1>
  loadRBoxDiag(ros::NodeHandle& nh)
  {
    Eigen::Matrix<double, ERR_STATE_DIM, 1> diag;
    std::vector<double> vec;
    if (nh.getParam("R_box_diag", vec) && vec.size() == ERR_STATE_DIM)
    {
      for (size_t i = 0; i < ERR_STATE_DIM; ++i)
        diag(i) = vec[i];
    }
    else
    {
      ROS_WARN_STREAM("ESKF: using default R_box_diag");
      diag <<
        1e-3, 1e-3, 1e-3,
        1e-3, 1e-3, 1e-3,
        1e-2, 1e-2, 1e-2,
        1e-2, 1e-2, 1e-2;
    }
    return diag;
  }

  void reconfigureCallback(your_package_name::EskfConfig &config, uint32_t level) 
  {
    ROS_INFO("Reconfiguring ESKF gains...");

    // Update Camera Bias
    this->camera_z_bias_ = config.camera_z_bias;

    // --- Build Q (Process Noise) ---
    Eigen::Matrix<double, ERR_STATE_DIM, 1> Q_diag;
    Q_diag.segment<3>(0).setConstant(config.Q_pos);     // dr
    Q_diag.segment<3>(3).setConstant(config.Q_att);     // phi
    Q_diag.segment<3>(6).setConstant(config.Q_vel);     // dv
    Q_diag.segment<3>(9).setConstant(config.Q_ang_vel); // dw
    
    // --- Build R_cam ---
    Eigen::Matrix<double, 6, 1> R_cam_diag;
    R_cam_diag.segment<3>(0).setConstant(config.R_cam_pos);
    R_cam_diag.segment<3>(3).setConstant(config.R_cam_att);

    // --- Build R_box ---
    Eigen::Matrix<double, ERR_STATE_DIM, 1> R_box_diag;
    R_box_diag.segment<3>(0).setConstant(config.R_box_pos);
    R_box_diag.segment<3>(3).setConstant(config.R_box_att);
    R_box_diag.segment<3>(6).setConstant(config.R_box_vel);
    R_box_diag.segment<3>(9).setConstant(config.R_box_ang_vel);

    // --- Push to Filter Core ---
    // This is where we use the setters we made in Step 1
    filter_.setProcessNoise(Q_diag.asDiagonal());
    filter_.setCamMeasurementNoise(R_cam_diag.asDiagonal());
    filter_.setBoxMeasurementNoise(R_box_diag.asDiagonal());
  }

  // Load Tag Pose from BaseAprilTag.yaml
  static Eigen::Affine3d loadTagPose(ros::NodeHandle& nh)
  {
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    std::vector<double> pos, rpy;

    // Load Position
    if (nh.getParam("apriltag_pose_world/position", pos) && pos.size() == 3) {
        T.translation() << pos[0], pos[1], pos[2];
    } else {
        ROS_WARN("ESKF: Tag position not found, assuming Origin (0,0,0)");
    }

    // Load Rotation (RPY in degrees for convenience, or radians)
    // Let's assume params are in Radians for standard consistency
    if (nh.getParam("apriltag_pose_world/orientation_rpy", rpy) && rpy.size() == 3) {
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
        T.linear() = R;
    } else {
        ROS_WARN("ESKF: Tag orientation not found, assuming Identity");
    }

    return T;
  }

  // --------------------- Callbacks ------------------------
  void boxCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    //world_frame_id_ = msg->header.frame_id; // Capture "world" from blackbox
    //body_frame_id_  = msg->child_frame_id;  // Capture "base_link" from blackbox
    
    if (!filter_.isInitialized())
    {
      filter_.initializeFromBox(*msg);
      last_time_ = msg->header.stamp;
      auto out = filter_.makeOdometry(msg->header.stamp,
                                      msg->header.frame_id,
                                      msg->child_frame_id);
      fused_pub_.publish(out);
      return;
    }

    double dt = (msg->header.stamp - last_time_).toSec();
    if (dt < 0.0) dt = 0.0;
    if (dt > 1.0) dt = 1.0;

    filter_.predict(dt);
    last_time_ = msg->header.stamp;

    filter_.updateWithBox(*msg);
    auto out = filter_.makeOdometry(msg->header.stamp,
                                    msg->header.frame_id,
                                    msg->child_frame_id);
    fused_pub_.publish(out);
  }

  void camCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
  {
    if (!filter_.isInitialized())
      return;

    // Check and iterate through messages
    if (msg->detections.empty()) {
        return; // No tags seen this frame
    }

    int found_index = -1;

    for (size_t i = 0; i < msg->detections.size(); ++i) {
        // Note: detections[i].id is a vector (bundles), for single tags it has size 1
        if (msg->detections[i].id.size() > 0 && msg->detections[i].id[0] == target_tag_id_) {
            found_index = i;
            break;
        }
    }

    if (found_index == -1) return; // Target tag not found in this image

    double dt = (msg->header.stamp - last_time_).toSec();
    if (dt > 0.0)
    {
      if (dt > 1.0) dt = 1.0;
      filter_.predict(dt);
      last_time_ = msg->header.stamp;
    }

    // 3. TF MAGIC: Get Transform from Body -> Tag
    // We rely on apriltag_ros publishing the TF frame (e.g. "tag_0")
    // TF automatically handles: Base -> Camera -> Optical -> Tag
    Eigen::Affine3d T_Body_Tag; 
    
    try {
        // Ask TF: "What is the Tag's pose relative to the Body Frame?"
        // We use a small timeout in case TF is slightly behind the message
        geometry_msgs::TransformStamped tf_body_tag = tf_buffer_.lookupTransform(
            body_frame_id_,      // Target: Base Link
            target_tag_frame_name_,      // Source: Tag Frame
            msg->header.stamp,   // Time: Sync with image
            ros::Duration(0.1)); // Timeout

        T_Body_Tag = tf2::transformToEigen(tf_body_tag);

    } catch (tf2::TransformException &ex) {
        ROS_WARN("ESKF: Could not lookup tag transform: %s", ex.what());
        return;
    }
    
    // 4. Compute Robot Pose in World
    // Logic: 
    //   We know T_World_Tag (from map/yaml)
    //   We measured T_Body_Tag (from TF)
    //   We want T_World_Body
    //   
    //   T_World_Body * T_Body_Tag = T_World_Tag
    //   T_World_Body = T_World_Tag * (T_Body_Tag)^-1
    
    Eigen::Affine3d T_World_Body = tag_pose_world_ * T_Body_Tag.inverse(); 

    // Prepare Data
    Eigen::Vector3d p = T_World_Body.translation();
    Eigen::Quaterniond q(T_World_Body.rotation());

    // --- Publish Debug Pose ---
    geometry_msgs::PoseStamped debug_pose;
    debug_pose.header.stamp = msg->header.stamp;
    debug_pose.header.frame_id = world_frame_id_; // Should be "world"
    
    debug_pose.pose.position.x = p.x();
    debug_pose.pose.position.y = p.y();
    debug_pose.pose.position.z = p.z() + camera_z_bias_;
    debug_pose.pose.orientation.w = q.w();
    debug_pose.pose.orientation.x = q.x();
    debug_pose.pose.orientation.y = q.y();
    debug_pose.pose.orientation.z = q.z();
    
    debug_pub_.publish(debug_pose);

    // --- Feed Filter ---
    nav_msgs::Odometry corrected_msg;
    corrected_msg.header = debug_pose.header;
    corrected_msg.child_frame_id = body_frame_id_;
    corrected_msg.pose.pose = debug_pose.pose;

    filter_.updateWithCam(corrected_msg);

    auto out = filter_.makeOdometry(msg->header.stamp, 
                                   world_frame_id_, 
                                   body_frame_id_);
    fused_pub_.publish(out);
   
  }
};

// ================================ main ======================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "eskf_node");
  ros::NodeHandle nh("~"); // private for params below ESKF node

  EskfNode node(nh);
  ros::spin();
  return 0;
}

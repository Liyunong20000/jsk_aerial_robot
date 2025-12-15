// eskf_node.cpp

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include <apriltag_ros/AprilTagDetectionArray.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "state_wrappers.hpp"  // for ERR_STATE_DIM, NOM_STATE_DIM
#include "eskf.hpp"

#include "DownwardCamera.hpp"

// ============================= ROS façade ==================================
class EskfNode
{
public:
  explicit EskfNode(ros::NodeHandle& nh)
    : nh_(nh)
    , filter_(loadQProcDiag(nh_), loadRCamDiag(nh_), loadRBoxDiag(nh_))
    , camera_pose_(loadCameraPose(nh_))
    , tag_pose_world_(loadTagPose(nh_))
  {
    // --- Topics ---
    nh_.param<std::string>("box_odom_topic",   box_topic_,   std::string("/xuanwu/noisy_odom"));
    nh_.param<std::string>("cam_odom_topic",   cam_topic_,   std::string("/xuanwu/tag_detections"));
    nh_.param<std::string>("fused_odom_topic", fused_topic_, std::string("/xuanwu/eskf/odom"));
    nh_.param<int>("target_tag_id", target_tag_id_, 0);

    box_sub_   = nh_.subscribe(box_topic_, 10, &EskfNode::boxCallback, this);
    cam_sub_   = nh_.subscribe(cam_topic_, 10, &EskfNode::camCallback, this);
    fused_pub_ = nh_.advertise<nav_msgs::Odometry>(fused_topic_, 10);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber box_sub_;
  ros::Subscriber cam_sub_;
  ros::Publisher  fused_pub_;
  std::string box_topic_, cam_topic_, fused_topic_;
  int target_tag_id_;

  std::string world_frame_id_ = "world";      // Default
  std::string body_frame_id_  = "base_link";  // Default

  Eskf filter_;
  ros::Time last_time_;

  DownwardCamera camera_pose_;
  Eigen::Affine3d tag_pose_world_;

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

  // Load Camera Pose from Embedded_Camera.yaml
  static DownwardCamera loadCameraPose(ros::NodeHandle& nh)
  {
    std::vector<double> trans_list;
    double z_rot = 90.0; // Default fallback

    // Attempt to load "camera_pose_body/translation_B_C" (x, y, z)
    Eigen::Vector3d offset(0.05, 0.0, -0.02); // Default fallback
    if (nh.getParam("camera_pose_body/translation_B_C", trans_list) && trans_list.size() == 3) {
        offset << trans_list[0], trans_list[1], trans_list[2];
    } else {
        ROS_WARN("ESKF: using default camera offset [0.05, 0, -0.02]");
    }

    // Attempt to load "camera_pose_body/z_rotation_deg"
    if (!nh.getParam("camera_pose_body/z_rotation_deg", z_rot)) {
        ROS_WARN("ESKF: using default camera rotation 90.0 deg");
    }

    return DownwardCamera(offset, z_rot);
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
    world_frame_id_ = msg->header.frame_id; // Capture "world" from blackbox
    body_frame_id_  = msg->child_frame_id;  // Capture "base_link" from blackbox
    
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

    // Extract Pose from the detected tag
    // The pose is in msg->detections[i].pose.pose.pose
    auto& tag_pose = msg->detections[found_index].pose.pose.pose;


    // Apply "Pseudo-Measurement" strategy before passing the message to the filter
    // 1st - Create Transform from Msg (Tag in Camera Frame)
    Eigen::Affine3d T_C_Tag = Eigen::Affine3d::Identity();
    T_C_Tag.translate(Eigen::Vector3d(tag_pose.position.x, tag_pose.position.y, tag_pose.position.z));
    T_C_Tag.rotate(Eigen::Quaterniond(tag_pose.orientation.w, tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z));    

    auto T_Cam_B = camera_pose_.getInverseExtrinsics_T_C_B();

    // 2nd - Invert Chain: Body_World = Tag_World * (Tag_Cam)^-1 * (Cam_Body)^-1
    // T_B_Cam_ and T_W_Tag_ should be member variables
    Eigen::Affine3d T_W_B = tag_pose_world_ * T_C_Tag * T_Cam_B; // Here in theory should be inverse at C Tag...

    // 3rd - Update Msg with calculated Body Pose
    nav_msgs::Odometry corrected_msg;
    corrected_msg.header = msg->header; // Copy timestamp/frame
    
    corrected_msg.header.frame_id = world_frame_id_;
    corrected_msg.child_frame_id  = body_frame_id_;

    Eigen::Vector3d p = T_W_B.translation();
    Eigen::Quaterniond q(T_W_B.rotation());

    corrected_msg.pose.pose.position.x = p.x();
    corrected_msg.pose.pose.position.y = p.y();
    corrected_msg.pose.pose.position.z = p.z();
    
    corrected_msg.pose.pose.orientation.w = q.w();
    corrected_msg.pose.pose.orientation.x = q.x();
    corrected_msg.pose.pose.orientation.y = q.y();
    corrected_msg.pose.pose.orientation.z = q.z();

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

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <aerial_robot_msgs/FlightNav.h> // JSK Standard

#include <dynamic_reconfigure/server.h>
#include <xuanwu/PlannerConfig.h> // Ensure this .cfg is generated

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <mutex>
#include <memory>
#include <cmath>

// Custom Headers
#include "quadrotor_model.hpp"
#include "mpc_controller.hpp" // Includes MPC and LMPC classes
#include "quat_helpers.hpp"

// Headers RViz Publish
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// TF2 Headers for the landmark
#include <tf2_ros/transform_listener.h> 
#include <tf2_ros/buffer.h>

// ============================= ROS façade ==================================
class PlannerNode
{
public:
  explicit PlannerNode(ros::NodeHandle& nh)
    : nh_(nh)
    , has_state_(false)
    , tf_listener_(tf_buffer_)
  {
    // --- Load Parameters ---
    loadParameters();

    // --- Init Architecture ---
    
    // 2. Initialize Quadrotor Model
    quad_model_.setMass(1.9);           // Mass [kg]
    quad_model_.setGravity(9.81);       // Gravity [m/s^2]
    quad_model_.setThrustLimit(14.6232);       // Thrust Limit [N]
    
    // Calculate accurate Arm Length (hypotenuse of rotor_x, rotor_y)
    // rotor_x = 0.12728, rotor_y = 0.12731
    double arm_length = std::hypot(0.12728, 0.12731); 
    quad_model_.setArmLength(arm_length); 
    quad_model_.setForceTorqueRatio(0.011); // Drag coeff ratio (m_f_rate)

    // 3. Setup Inertia (Frame Rotation)
    Eigen::Matrix3d J_robot;
    J_robot.setZero();
    J_robot(0,0) = 0.01400956434; // Ixx (Robot Frame)
    J_robot(1,1) = 0.01345117632; // Iyy
    J_robot(2,2) = 0.01544169758; // Izz
    
    quad_model_.setInertia(J_robot);

    // 4. Initialize MPC Controller Config
    MPC::Config mpc_cfg;
    mpc_cfg.N = prediction_horizon_;
    mpc_cfg.dt = 1.0 / control_rate_;
    
    // 5. Instantiate LMPC
    mpc_controller_ = std::make_unique<LMPC>(mpc_cfg, quad_model_);

    // --- Topics ---
    state_sub_ = nh_.subscribe(state_topic_, 1, &PlannerNode::stateCallback, this);
    cmd_pub_   = nh_.advertise<aerial_robot_msgs::FlightNav>(cmd_topic_, 1);
    debug_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/xuanwu/debug/mpc_setpoint", 1);

    // --- Dynamic Reconfigure ---
    dr_callback_ = boost::bind(&PlannerNode::reconfigureCallback, this, _1, _2);
    dr_server_.setCallback(dr_callback_);

    // --- Control Timer ---
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_), 
                                     &PlannerNode::controlLoop, this);
                                     
    ROS_INFO("MPC Planner Node Started. Rate: %.2f Hz", control_rate_);
  }

private:
  ros::NodeHandle nh_;

  // --- ROS Members ---
  ros::Subscriber state_sub_;
  ros::Publisher  cmd_pub_;
  ros::Publisher  debug_pub_;
  ros::Publisher mpc_path_pub_ = nh_.advertise<nav_msgs::Path>("mpc_prediction_path", 1);
  ros::Publisher mpc_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("mpc_prediction_poses", 1);
  ros::Timer      control_timer_;

  // TF2 Listener -- Land mark
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string state_topic_;
  std::string cmd_topic_;

  // --- Configuration ---
  double control_rate_;
  int    prediction_horizon_;
  int    lookahead_steps_; 

  // --- State Management ---
  // Stored in MODEL FRAME
  // 13-dim: [px, py, pz | qw, qx, qy, qz | vx, vy, vz | wx, wy, wz ]
  Eigen::Matrix<double, 13, 1> current_state_model_;
  bool has_state_;
  std::mutex state_mutex_;

  // --- Frame Transforms ---
  Eigen::Matrix3d R_robot2model_; // Matrix: Multiplies v_robot to get v_model
  Eigen::Quaterniond q_model2robot_; // Quat: Represents rotation of Model W.R.T

  // --- Controller Architecture ---
  LinearQuadrotorModel quad_model_;
  std::unique_ptr<LMPC> mpc_controller_;

  // --- Dynamic Reconfigure ---
  dynamic_reconfigure::Server<xuanwu::PlannerConfig> dr_server_;
  dynamic_reconfigure::Server<xuanwu::PlannerConfig>::CallbackType dr_callback_;

  // ----------------------- Initialization ---------------------------
  void loadParameters()
  {
    nh_.param<std::string>("state_topic", state_topic_, "/xuanwu/eskf/odom");
    nh_.param<std::string>("cmd_topic",   cmd_topic_,   "/xuanwu/uav/nav");
    
    nh_.param<double>("control_rate", control_rate_, 20.0);
    nh_.param<int>("prediction_horizon", prediction_horizon_, 20);
    nh_.param<int>("lookahead_steps", lookahead_steps_, 3); 
  }

  // ----------------------- Callbacks ---------------------------

  // 1. Read State & Convert to Model Frame
  void stateCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // 1. Position (World Frame) - No change
    current_state_model_(0) = msg->pose.pose.position.x;
    current_state_model_(1) = msg->pose.pose.position.y;
    current_state_model_(2) = msg->pose.pose.position.z;
 
    // 2. Orientation (World Frame) - DIRECT COPY (No offset)
    current_state_model_(3) = msg->pose.pose.orientation.w;
    current_state_model_(4) = msg->pose.pose.orientation.x;
    current_state_model_(5) = msg->pose.pose.orientation.y;
    current_state_model_(6) = msg->pose.pose.orientation.z;

    // 3. Linear Velocity (Body Frame) - DIRECT COPY (No Rotation matrix)
    current_state_model_(7) = msg->twist.twist.linear.x;
    current_state_model_(8) = msg->twist.twist.linear.y;
    current_state_model_(9) = msg->twist.twist.linear.z;
    
    // 4. Angular Velocity (Body Frame) - DIRECT COPY
    current_state_model_(10) = msg->twist.twist.angular.x;
    current_state_model_(11) = msg->twist.twist.angular.y;
    current_state_model_(12) = msg->twist.twist.angular.z;

    has_state_ = true;
  }

  // 2. Reconfigure
  void reconfigureCallback(xuanwu::PlannerConfig &config, uint32_t level) 
  {
    ROS_INFO("Reconfiguring MPC Planner...");

    lookahead_steps_ = config.lookahead_steps;

    Eigen::VectorXd Q_diag(12);
    Q_diag << config.Q_pos_x, config.Q_pos_y, config.Q_pos_z,
              config.Q_att_r, config.Q_att_p, config.Q_att_y,
              config.Q_vel_x, config.Q_vel_y, config.Q_vel_z,
              config.Q_omega, config.Q_omega, config.Q_omega;

    Eigen::VectorXd R_diag(4);
    R_diag << config.R_thrust, config.R_thrust, config.R_thrust, config.R_thrust;

    if(mpc_controller_) {
        mpc_controller_->updateWeights(Q_diag, R_diag);
    }
  }

  // 3. Main Control Loop
  void controlLoop(const ros::TimerEvent& event)
  {
    if (!mpc_controller_ || !has_state_) return;
    
    // 1. Get Dynamic Setpoint from "land_mark" TF ---
    geometry_msgs::TransformStamped transform;
    try {
        // Lookup transform from world to land_mark
        transform = tf_buffer_.lookupTransform("world", "land_mark", ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0, "[Planner] Waiting for 'land_mark' frame... (%s)", ex.what());
        return; // Skip this control cycle if tag is not visible
    }
    
    // Extract Yaw from the landmark (we ignore Roll/Pitch for the reference)
    tf2::Quaternion q_tf;
    tf2::fromMsg(transform.transform.rotation, q_tf);
    double roll_ref, pitch_ref, yaw_ref;
    tf2::Matrix3x3(q_tf).getRPY(roll_ref, pitch_ref, yaw_ref);

    // Construct the Reference State (x_ref)
    // 13-dim: [px, py, pz | qw, qx, qy, qz | vx, vy, vz | wx, wy, wz ]
    Eigen::Matrix<double, 13, 1> x_ref;
    x_ref.setZero();

    // Position: Landmark position + Z offset
    x_ref(0) = transform.transform.translation.x;
    x_ref(1) = transform.transform.translation.y;
    double z_offset = 1.0; // Adjust this offset distance (meters)
    x_ref(2) = transform.transform.translation.z + z_offset;

    // Orientation: Flat (Roll=0, Pitch=0) but aligned with Landmark Yaw
    tf2::Quaternion q_ref;
    q_ref.setRPY(0.0, 0.0, yaw_ref);
    x_ref(3) = q_ref.w();
    x_ref(4) = q_ref.x();
    x_ref(5) = q_ref.y();
    x_ref(6) = q_ref.z();
    
    // Velocity & Omega: Zero (we want to hover relative to the tag)
    // x_ref.tail<6>().setZero(); // Already zero from initialization

    // --- Get Current State ---

    Eigen::Matrix<double, 13, 1> x_current_model;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        x_current_model = current_state_model_;
    }

    // --- MPC Solve ---

    // 1. Position Error
    // We must rotate the World-Frame position error into the frame aligned with the robot's current Yaw.
    // Otherwise, the MPC (linearized at Yaw=0) will mix up X and Y axes when the robot turns.
    
    // Get current Yaw of the robot
    tf2::Quaternion q_curr_tf(x_current_model(4), x_current_model(5), x_current_model(6), x_current_model(3)); // x,y,z,w
    double r_curr, p_curr, y_curr;
    tf2::Matrix3x3(q_curr_tf).getRPY(r_curr, p_curr, y_curr);

    //// Calculate Raw World Error
    //Eigen::Vector3d dr_world = x_ref.head<3>() - x_current_model.head<3>();

    //// Create 2D Rotation Matrix for Yaw (Active Rotation)
    //// We want to project the error vector INTO the body-aligned frame, so we use Inverse Rotation (-Yaw)
    //double cy = cos(y_curr);
    //double sy = sin(y_curr);
    //
    //Eigen::Vector3d dr_body_aligned;
    //dr_body_aligned.x() =  cy * dr_world.x() + sy * dr_world.y();
    //dr_body_aligned.y() = -sy * dr_world.x() + cy * dr_world.y();
    //dr_body_aligned.z() = dr_world.z(); // Z is unaffected by Yaw
    //
    //// Use this rotated error for the MPC
    //Eigen::Vector3d dr = dr_body_aligned;
    
    Eigen::Vector3d dr_prev = x_ref.head<3>() - x_current_model.head<3>();  
    
    // Create Angle-Axis rotation for -yaw_ref around Z
    Eigen::Quaterniond q_yaw_eigen(
        Eigen::AngleAxisd(-y_curr, Eigen::Vector3d::UnitZ())
    );

    Eigen::Vector3d dr = q_yaw_eigen * dr_prev;  
    
    // 2. Attitude Error (Quaternion Manifold)
    Eigen::Vector4d q_target = x_ref.segment<4>(3);
    Eigen::Vector4d q_curr   = x_current_model.segment<4>(3);
    Eigen::Vector3d dtheta   = quatErrorRodrigues(q_curr, q_target);

    // 3. Velocity Error
    Eigen::Vector3d dv = x_ref.segment<3>(7) - x_current_model.segment<3>(7);
    
    // 4. Omega Error
    Eigen::Vector3d dw = x_ref.tail<3>() - x_current_model.tail<3>();

    Eigen::VectorXd x0_error(12);
    x0_error << dr, dtheta, dv, dw;

    Eigen::VectorXd u_opt_delta;
    std::vector<Eigen::VectorXd> prediction_horizon;
    
    bool success = mpc_controller_->solve(x0_error, u_opt_delta, prediction_horizon);

    if (!success) {
        ROS_WARN_THROTTLE(1.0, "[Planner] MPC Solver Failed/Infeasible!");
        return;
    } else {
      // Pass the error trajectory AND the reference state (x_ref)
      publishMPCTrajectory(mpc_path_pub_, mpc_poses_pub_, prediction_horizon, x_ref, "world");
    }

    // --- Output Conversion ---
    
    // Create Angle-Axis rotation for -yaw_ref around Z
    Eigen::Quaterniond q_inv_yaw_eigen(
        Eigen::AngleAxisd(y_curr, Eigen::Vector3d::UnitZ())
    ); 

    // Target Position
    Eigen::Vector3d r_cmd = x_ref.head<3>() + q_inv_yaw_eigen * prediction_horizon[0].head<3>();
    //Eigen::Vector3d r_cmd =  r_cmd_rot;

    // Target Velocity (Model Body Frame)
    Eigen::Vector3d v_body_model = x_ref.segment<3>(7) + prediction_horizon[0].segment<3>(6);
   
    // Convert to World Frame Velocity for JSK Controller
    // Since MPC runs in Model Frame, it outputs velocities in Model Body Frame.
    // To get World Velocity: v_world = q_model_world * v_body_model
    Eigen::Quaterniond q_model_now(
      x_current_model(3), x_current_model(4), x_current_model(5), x_current_model(6)
    );
    //Eigen::Vector3d v_world = q_inv_yaw_eigen * v_body_model;
    Eigen::Vector3d v_world = q_model_now * v_body_model;
    
    // --- Publish ---
    aerial_robot_msgs::FlightNav cmd_msg;
    cmd_msg.header.stamp = ros::Time::now();
    cmd_msg.header.frame_id = "world";

    cmd_msg.pos_xy_nav_mode = aerial_robot_msgs::FlightNav::POS_VEL_MODE;
    cmd_msg.pos_z_nav_mode  = aerial_robot_msgs::FlightNav::POS_VEL_MODE;
    cmd_msg.yaw_nav_mode    = aerial_robot_msgs::FlightNav::POS_MODE;
    cmd_msg.target          = aerial_robot_msgs::FlightNav::COG; 
    cmd_msg.control_frame   = aerial_robot_msgs::FlightNav::WORLD_FRAME;

    cmd_msg.target_pos_x = r_cmd.x();
    cmd_msg.target_pos_y = r_cmd.y();
    cmd_msg.target_pos_z = r_cmd.z();

    cmd_msg.target_vel_x = v_world.x();
    cmd_msg.target_vel_y = v_world.y();
    cmd_msg.target_vel_z = v_world.z();
 
    // 3. Target Yaw Calculation
    // ----------------------------------------------------------------------
    // Step A: Create Quaternion from Reference Yaw
    // We construct q_ref corresponding to (roll=0, pitch=0, yaw=yaw_ref)
    // Note: Your helper uses Eigen (w, x, y, z) order, TF2 uses (x, y, z, w)
    tf2::Quaternion q_ref_tf;
    q_ref_tf.setRPY(0.0, 0.0, yaw_ref);
    Eigen::Vector4d q_ref_eigen(q_ref_tf.w(), q_ref_tf.x(), q_ref_tf.y(), q_ref_tf.z());

    // Step B: Extract Rodrigues Error from MPC Horizon
    // Indices 3, 4, 5 correspond to the Rodrigues parameters (phi)
    Eigen::Vector3d phi_mpc = prediction_horizon[0].segment<3>(3);

    // Step C: Convert Rodrigues Error -> Quaternion Error (delta_q)
    // Using your helper function: q = [1; phi] / sqrt(1 + phi^2)
    Eigen::Vector4d delta_q = rodriguesToQuat<double>(phi_mpc);

    // Step D: Compose Rotations
    // q_cmd = q_ref * delta_q
    // This applies the MPC's optimal error correction to the reference
    Eigen::Vector4d q_cmd = quatMultiply<double>(q_ref_eigen, delta_q);

    // Step E: Extract Final Target Yaw
    // Convert back to TF2 to get Euler angles
    tf2::Quaternion q_cmd_final(q_cmd(1), q_cmd(2), q_cmd(3), q_cmd(0)); // x, y, z, w
    double cmd_roll, cmd_pitch, cmd_yaw;
    tf2::Matrix3x3(q_cmd_final).getRPY(cmd_roll, cmd_pitch, cmd_yaw);

    // Apply to message
    cmd_msg.target_yaw = cmd_yaw;
    
    cmd_pub_.publish(cmd_msg);

    // Debug Visualization
    geometry_msgs::PoseStamped debug_msg;
    debug_msg.header = cmd_msg.header;
    debug_msg.pose.position.x = r_cmd.x();
    debug_msg.pose.position.y = r_cmd.y();
    debug_msg.pose.position.z = r_cmd.z();

    // Visualize the Reference Orientation (Target Yaw)
    tf2::Quaternion q_debug;
    q_debug.setRPY(0, 0, cmd_yaw);
    debug_msg.pose.orientation = tf2::toMsg(q_debug);
    
    debug_pub_.publish(debug_msg); 
  }

  // Helper to visualize the 12-dim Error State Trajectory
  void publishMPCTrajectory(ros::Publisher& path_pub, 
                            ros::Publisher& pose_pub, 
                            const std::vector<Eigen::VectorXd>& horizon_errors,
                            const Eigen::Matrix<double, 13, 1>& x_ref, // Pass reference state (13-dim)
                            const std::string& frame_id = "world") 
  {
      if (horizon_errors.empty()) return;
  
      nav_msgs::Path path_msg;
      geometry_msgs::PoseArray poses_msg;
  
      path_msg.header.stamp = ros::Time::now();
      path_msg.header.frame_id = frame_id;
      poses_msg.header = path_msg.header;
  
      // Extract Reference parts once
      Eigen::Vector3d p_ref = x_ref.head<3>();
      Eigen::Vector4d q_ref = x_ref.segment<4>(3);
  
      for (const auto& dx : horizon_errors) {
          // dx is 12-dim: [dp(3), dphi(3), dv(3), dw(3)]
  
          geometry_msgs::PoseStamped pose_s;
          
          // --- 1. Position Reconstruction ---
          // p_new = p_ref + dp
          Eigen::Vector3d p_new = p_ref + dx.head<3>();
          
          pose_s.pose.position.x = p_new.x();
          pose_s.pose.position.y = p_new.y();
          pose_s.pose.position.z = p_new.z();
  
          // --- 2. Orientation Reconstruction ---
          // Indices 3,4,5 are Rodrigues parameters (phi)
          Eigen::Vector3d dphi = dx.segment<3>(3);
          
          // Convert Rodrigues error -> Quaternion error
          // Using your helper: q = [1; phi] / sqrt(1+phi^2)
          Eigen::Vector4d dq = rodriguesToQuat<double>(dphi);
  
          // Compose: q_new = q_ref * dq
          // Using your helper: quatMultiply(q1, q2) = L(q1)*q2
          Eigen::Vector4d q_new = quatMultiply<double>(q_ref, dq);
          
          // Normalize to be safe
          normalizeQuat<double>(q_new);
  
          // Convert Eigen Quat to Geometry Msg
          // Eigen/ROS use (x,y,z,w) order for msgs, but your vector is (w,x,y,z) or (s,v)
          // Your helper uses: q(0) is scalar (w), q(1,2,3) is vector (x,y,z)
          pose_s.pose.orientation.w = q_new(0);
          pose_s.pose.orientation.x = q_new(1);
          pose_s.pose.orientation.y = q_new(2);
          pose_s.pose.orientation.z = q_new(3);
  
          // Add to messages
          path_msg.poses.push_back(pose_s);
          poses_msg.poses.push_back(pose_s.pose);
      }
  
      path_pub.publish(path_msg);
      pose_pub.publish(poses_msg);
  }
};

// ================================ main ======================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_planner_node");
  ros::NodeHandle nh("~");

  PlannerNode node(nh);
  ros::spin();
  return 0;
}

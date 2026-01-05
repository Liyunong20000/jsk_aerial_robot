#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <aerial_robot_msgs/FlightNav.h> // JSK Standard
#include <std_msgs/Empty.h> // For Halt
#include <std_msgs/Bool.h>  // For Trigger

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

#include <chrono> // Added for timing

// ============================= ROS façade ==================================
class PlannerNode
{
public:
  // !!! CRITICAL FIX: Ensure 16-byte alignment for Eigen members !!!
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  explicit PlannerNode(ros::NodeHandle& nh)
    : nh_(nh)
    , has_state_(false)
    , landing_active_(false) // DEFAULT: Inactive (Waiting for activation)
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
    
    mpc_cfg.u_max = Eigen::VectorXd::Constant(4, quad_model_.getThrustLimit());

    // 5. Instantiate LMPC
    mpc_controller_ = std::make_unique<LMPC>(mpc_cfg, quad_model_);

    u_last_ = Eigen::VectorXd::Zero(4);

    std::cout << "PRE CALLBACK LOG INSTANCE!!" << std::endl;
    // --- Topics ---
    state_sub_ = nh_.subscribe(state_topic_, 1, &PlannerNode::stateCallback, this);

    std::cout << "stateCallback registered!!" << std::endl;
    cmd_pub_   = nh_.advertise<aerial_robot_msgs::FlightNav>(cmd_topic_, 1);

    std::cout << "command publisher registered!!" << std::endl;
    debug_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/xuanwu/debug/mpc_setpoint", 1);

    std::cout << "debug pub registered!!" << std::endl;
    
    halt_pub_    = nh_.advertise<std_msgs::Empty>("/xuanwu/teleop_command/halt", 1);

    std::cout << "halt pub registered!!" << std::endl;
    trigger_sub_ = nh_.subscribe("start_landing", 1, &PlannerNode::triggerCallback, this);

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
  ros::Subscriber trigger_sub_;
  ros::Publisher  cmd_pub_;
  ros::Publisher  debug_pub_;
  ros::Publisher  halt_pub_;
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
  
  // Timing / Profiling Variables
  std::vector<double> solve_times_ms_;
  ros::Time last_print_time_;
  const double print_interval_ = 0.5; // Print every 1 second

  // --- State Management ---
  // Stored in MODEL FRAME
  // 13-dim: [px, py, pz | qw, qx, qy, qz | vx, vy, vz | wx, wy, wz ]
  Eigen::Matrix<double, 13, 1> current_state_model_;
  Eigen::VectorXd u_last_; // Stores accumulated thrust deviation (u_{k-1})
  bool has_state_;
  std::mutex state_mutex_;
  bool landing_active_;

  // --- Frame Transforms ---
  Eigen::Matrix3d R_robot2model_; // Matrix: Multiplies v_robot to get v_model
  Eigen::Quaterniond q_model2robot_; // Quat: Represents rotation of Model W.R.T

  // --- Controller Architecture ---
  LinearQuadrotorModel quad_model_;
  std::unique_ptr<LMPC> mpc_controller_;

  // --- Dynamic Reconfigure ---
  dynamic_reconfigure::Server<xuanwu::PlannerConfig> dr_server_;
  dynamic_reconfigure::Server<xuanwu::PlannerConfig>::CallbackType dr_callback_;

  // Low Pass Filter Config
    double lpf_alpha_ = 0.1; // Factor 0.0 to 1.0.
                             // 1.0 = No filter (Raw). 0.1 = Very Smooth (High Lag).
                             // Start with 0.1 or 0.2.

    // Filter State
    bool filter_initialized_ = false;
    Eigen::Vector3d t_tag_smooth_;
    double yaw_tag_smooth_ = 0.0;

  // ----------------------- Initialization ---------------------------
  void loadParameters()
  {
    nh_.param<std::string>("state_topic", state_topic_, "/xuanwu/eskf/odom");
    nh_.param<std::string>("cmd_topic",   cmd_topic_,   "/xuanwu/uav/nav");
    
    nh_.param<double>("control_rate", control_rate_, 20.0);
    nh_.param<int>("prediction_horizon", prediction_horizon_, 20);
    nh_.param<int>("lookahead_steps", lookahead_steps_, 5); 
  }

  // ----------------------- Callbacks ---------------------------
  
  void triggerCallback(const std_msgs::BoolConstPtr& msg) {
      if (msg->data) {
          u_last_.setZero(4);
          landing_active_ = true;
          ROS_WARN(">> LANDING SEQUENCE ACTIVATED (C++ MPC Taking Control) <<");
      } else {
          landing_active_ = false;
          ROS_INFO(">> Landing Sequence Deactivated.");
      }
  }

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
  
      // 1. Check if Control Rate Changed -> Update Timer
      if (std::abs(config.control_rate - control_rate_) > 1e-3) {
          control_rate_ = config.control_rate;
          control_timer_.setPeriod(ros::Duration(1.0 / control_rate_));
          ROS_INFO(">> Timer updated to %.1f Hz", control_rate_);
      }
  
      // 2. Update Local Configs
      prediction_horizon_ = config.horizon; // Make sure cfg uses "horizon"
      lookahead_steps_    = config.lookahead_steps;
  
      // 3. Prepare Weights
      Eigen::VectorXd Q_diag(16);
      Q_diag << config.Q_pos_x, config.Q_pos_y, config.Q_pos_z,
                config.Q_att_r, config.Q_att_p, config.Q_att_y,
                config.Q_vel_x, config.Q_vel_y, config.Q_vel_z,
                config.Q_omega, config.Q_omega, config.Q_omega,
                  0.00001,0.00001,0.00001,0.00001;
  
      Eigen::VectorXd R_diag(4);
      R_diag << config.R_thrust, config.R_thrust, config.R_thrust, config.R_thrust;
  
      // 4. Update MPC (Resize solver if needed)
      if(mpc_controller_) {
          // Pass the NEW horizon and NEW dt
          mpc_controller_->updateConfig(prediction_horizon_, 1.0/control_rate_, Q_diag, R_diag);
      }
  }

  // 3. Main Control Loop
  void controlLoop(const ros::TimerEvent& event)
  {
    if (!mpc_controller_ || !has_state_ || !landing_active_) return;

    // --- 1. Get Raw Transform ---
    geometry_msgs::TransformStamped tf_msg;
    try {
        tf_msg = tf_buffer_.lookupTransform("world", "land_mark", ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0, "[Planner] Waiting for 'land_mark'...");
        filter_initialized_ = false; // Reset filter if we lose tracking!
        return;
    }

    // Convert Raw Translation
    Eigen::Vector3d t_raw(tf_msg.transform.translation.x,
                          tf_msg.transform.translation.y,
                          tf_msg.transform.translation.z + 0.30);

    // Convert Raw Orientation -> Extract Yaw
    tf2::Quaternion q_raw_tf;
    tf2::fromMsg(tf_msg.transform.rotation, q_raw_tf);
    double r_raw, p_raw, y_raw;
    tf2::Matrix3x3(q_raw_tf).getRPY(r_raw, p_raw, y_raw);

    // --- 2. Apply Low Pass Filter ---
    if (!filter_initialized_) {
        // First run: Initialize directly with raw values to prevent "jump" from 0
        t_tag_smooth_   = t_raw;
        yaw_tag_smooth_ = y_raw;
        filter_initialized_ = true;
    } else {
        // Position Filter: y_new = (1-alpha)*y_old + alpha*x_new
        t_tag_smooth_ = (1.0 - lpf_alpha_) * t_tag_smooth_ + lpf_alpha_ * t_raw;

        // Yaw Filter (Wrap-around Safe)
        // We calculate the shortest difference, scale it by alpha, and add it.
        double diff = y_raw - yaw_tag_smooth_;
        
        // Normalize diff to [-PI, PI]
        while (diff > M_PI)  diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;

        yaw_tag_smooth_ += lpf_alpha_ * diff;
        
        // Normalize result to [-PI, PI] just to be clean
        while (yaw_tag_smooth_ > M_PI)  yaw_tag_smooth_ -= 2.0 * M_PI;
        while (yaw_tag_smooth_ < -M_PI) yaw_tag_smooth_ += 2.0 * M_PI;
    }

    // --- 3. Construct the Smooth "World->Tag" Transform ---
    
    // Create Rotation from Smooth Yaw (Flat / Gravity Aligned)
    Eigen::Quaterniond q_tag(
        Eigen::AngleAxisd(yaw_tag_smooth_, Eigen::Vector3d::UnitZ())
    );

    Eigen::Isometry3d T_world_tag = Eigen::Isometry3d::Identity();
    T_world_tag.rotate(q_tag);
    T_world_tag.pretranslate(t_tag_smooth_); // Use smoothed position

    // --- 2. Transform Current State (World -> Tag) ---
    Eigen::Matrix<double, 13, 1> x_current_world;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        x_current_world = current_state_model_;
    }

    // A. Position: p_tag = T_inv * p_world
    Eigen::Vector3d p_world = x_current_world.head<3>();
    Eigen::Vector3d p_tag = T_world_tag.inverse() * p_world;

    // B. Orientation: q_tag = q_tag_world * q_world
    Eigen::Quaterniond q_world(x_current_world(3), x_current_world(4), 
                               x_current_world(5), x_current_world(6));
    // q_tag is World->Tag. We need the inverse (Tag->World) to rotate the robot's world orientation into the tag frame.
    Eigen::Quaterniond q_robot_tag = q_tag.inverse() * q_world;

    // C. Velocities (Body Frame) - DO NOT ROTATE
    // Why? The sensors measure velocity in the Body Frame. 
    // The Body Frame is "attached" to the drone, regardless of where the drone is.
    Eigen::Vector3d v_body = x_current_world.segment<3>(7);
    Eigen::Vector3d w_body = x_current_world.tail<3>();

    // Pack into local state vector
    Eigen::Matrix<double, 13, 1> x_current_local;
    x_current_local << p_tag, 
                       q_robot_tag.w(), q_robot_tag.vec(), 
                       v_body, w_body;

    // --- 3. Define Reference (in Tag Frame) ---
    Eigen::Matrix<double, 13, 1> x_ref;
    x_ref.setZero();
    x_ref(2) = 0.0; // 1.2m above the tag center (in Tag's Z axis)
    x_ref(3) = 1.0; // Identity Quaternion (Aligned with Tag)

    // --- 4. MPC Solve (in Tag Frame) ---
    
    // Calculate Errors
    // Position
    Eigen::Vector3d dr = x_ref.head<3>() - x_current_local.head<3>();
    
    // Attitude (Quaternion Error)
    Eigen::Vector4d q_ref_vec = x_ref.segment<4>(3);
    Eigen::Vector4d q_curr_vec = x_current_local.segment<4>(3);
    Eigen::Vector3d dtheta = quatErrorRodrigues(q_curr_vec, q_ref_vec);

    // Velocity (v_ref is 0, so error is just -v_current)
    Eigen::Vector3d dv = x_ref.segment<3>(7) - x_current_local.segment<3>(7);
    Eigen::Vector3d dw = x_ref.tail<3>() - x_current_local.tail<3>();

    Eigen::VectorXd x0_error(AUG_STATE_DIM);
    x0_error << dr, dtheta, dv, dw, u_last_;
    
    //Eigen::VectorXd x0_error(REDUCED_STATE_DIM);
    //x0_error << dr, dtheta, dv, dw;

    // --- TERMINATION CHECK ---
    // If we are close enough to the reference (which is the ground), KILL MOTORS.
    double pos_error = dr.norm();
    double vel_error = dv.norm();
    
    // Check: Total Error < 15cm AND Velocity < 0.2 m/s
    if (pos_error < 0.05 && vel_error < 0.05) {
        ROS_WARN(">> TOUCHDOWN DETECTED. HALTING MOTORS. <<");
        std_msgs::Empty halt_msg;
        halt_pub_.publish(halt_msg);
        
        landing_active_ = false; // Stop MPC
        return;
    }

    // --- START TIMER ---
    auto start_time = std::chrono::high_resolution_clock::now();

    Eigen::VectorXd u_opt;
    std::vector<Eigen::VectorXd> prediction_horizon;
    
    bool success = mpc_controller_->solve(x0_error, u_opt, prediction_horizon);
    
    // --- END TIMER ---
    auto end_time = std::chrono::high_resolution_clock::now();
    double duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count(); 

    // --- FREQUENCY MONITORING ---
    solve_times_ms_.push_back(duration_ms);

    if ((ros::Time::now() - last_print_time_).toSec() > print_interval_) {
        double sum = 0.0;
        double max_ms = 0.0;
        for (double t : solve_times_ms_) {
            sum += t;
            if (t > max_ms) max_ms = t;
        }
        double avg_ms = solve_times_ms_.empty() ? 0.0 : sum / solve_times_ms_.size();

        // Max Theor. Freq = 1000ms / avg_solve_time
        double max_freq = (avg_ms > 0) ? (1000.0 / avg_ms) : 0.0;

        ROS_INFO_THROTTLE(1.0,
            "[MPC Stats] Avg Time: %.3f ms | Max Time: %.3f ms | Max Theor. Freq: %.1f Hz",
            avg_ms, max_ms, max_freq);

        solve_times_ms_.clear();
        last_print_time_ = ros::Time::now();
    }

    if (!success) {
        ROS_WARN_THROTTLE(1.0, "MPC Failed");
        return;
    }

    // --- 8. Update Integrator ---
    // u_k = u_{k-1} + delta_u
    u_last_ += u_opt;

    // Visualize (Optional: Send x_ref and prediction_horizon directly, 
    // but remember to visualize in "land_mark" frame, not "world")
    publishMPCTrajectory(mpc_path_pub_, mpc_poses_pub_, prediction_horizon, x_ref, "land_mark");

    // --- 5. Convert Output (Tag -> World) ---
    // The MPC output is x_{k+1}. We need to transform this back to World for the drone.

    // A. Target Position (Tag Frame)
    // x_ref is [0,0,1.2]. horizon[0] is the error dx.
    // target = ref + error
    Eigen::Vector3d p_target_tag = x_ref.head<3>() + prediction_horizon[lookahead_steps_].head<3>();

    // Transform to World: p_world = T * p_tag
    Eigen::Vector3d p_target_world = T_world_tag * p_target_tag;

    // B. Target Orientation (Tag Frame)
    // Calculate target quaternion in Tag Frame
    Eigen::Vector3d phi_next = prediction_horizon[lookahead_steps_].segment<3>(3);
    Eigen::Vector4d dq_next = rodriguesToQuat<double>(phi_next);
    Eigen::Vector4d q_target_tag_vec = quatMultiply<double>(q_ref_vec, dq_next);
    Eigen::Quaterniond q_target_tag(q_target_tag_vec(0), q_target_tag_vec(1), 
                                    q_target_tag_vec(2), q_target_tag_vec(3));
    
    // Transform to World: q_world = q_tag_world * q_tag
    // q_tag is World->Tag. We use it to rotate the local target back to world.
    Eigen::Quaterniond q_target_world = q_tag * q_target_tag;
    
    // C. Target Velocity (Body Frame)
    // The controller output v is in Body Frame. 
    // We only need to rotate it to World Frame for the message.
    Eigen::Vector3d v_target_body = x_ref.segment<3>(7) + prediction_horizon[lookahead_steps_].segment<3>(6);
    
    // Rotate Body -> World (using the NEW target orientation)
    Eigen::Vector3d v_target_world = q_target_world * v_target_body;

    // --- 6. Publish ---
    aerial_robot_msgs::FlightNav cmd_msg;
    cmd_msg.header.stamp = ros::Time::now();
    cmd_msg.header.frame_id = "world";

    // Use POS_MODE first to stop jiggling!
    cmd_msg.pos_xy_nav_mode = aerial_robot_msgs::FlightNav::POS_VEL_MODE;
    cmd_msg.pos_z_nav_mode  = aerial_robot_msgs::FlightNav::POS_VEL_MODE;
    cmd_msg.yaw_nav_mode    = aerial_robot_msgs::FlightNav::POS_MODE;
    cmd_msg.control_frame   = aerial_robot_msgs::FlightNav::WORLD_FRAME;
    cmd_msg.target          = aerial_robot_msgs::FlightNav::COG;

    // 2. Velocidades (Feedforward: A velocidade que o MPC calculou para esse ponto)
    cmd_msg.target_vel_x = v_target_world.x();
    cmd_msg.target_vel_y = v_target_world.y();
    cmd_msg.target_vel_z = v_target_world.z();

    // Positions
    cmd_msg.target_pos_x = p_target_world.x();
    cmd_msg.target_pos_y = p_target_world.y();
    cmd_msg.target_pos_z = p_target_world.z();

    // Yaw
    double r_cmd, p_cmd, y_cmd;
    tf2::Matrix3x3(tf2::Quaternion(q_target_world.x(), q_target_world.y(), 
                                   q_target_world.z(), q_target_world.w())).getRPY(r_cmd, p_cmd, y_cmd);
    cmd_msg.target_yaw = y_cmd;

    cmd_pub_.publish(cmd_msg);
    // Debug
    geometry_msgs::PoseStamped debug_msg;
    debug_msg.header = cmd_msg.header;
    debug_msg.pose.position.x = p_target_world.x();
    debug_msg.pose.position.y = p_target_world.y();
    debug_msg.pose.position.z = p_target_world.z();
    debug_msg.pose.orientation.w = q_target_world.w();
    debug_msg.pose.orientation.x = q_target_world.x();
    debug_msg.pose.orientation.y = q_target_world.y();
    debug_msg.pose.orientation.z = q_target_world.z();
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

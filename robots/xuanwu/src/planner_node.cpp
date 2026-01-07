#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

// JSK / Custom Msgs
#include <aerial_robot_msgs/FlightNav.h> 

// TF2
#include <tf2_ros/transform_listener.h> 
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include <xuanwu/PlannerConfig.h> 

// Eigen & Std
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <mutex>
#include <memory>
#include <cmath>
#include <chrono>

// Custom Project Headers
#include "quadrotor_model.hpp"
#include "mpc_controller.hpp"
#include "quat_helpers.hpp"

/**
 * @brief MPC Planner Node for Xuanwu Quadrotor
 * Handles state estimation callbacks, MPC solving loops, and trajectory interpolation/publishing.
 */
class PlannerNode
{
public:
  explicit PlannerNode(ros::NodeHandle& nh)
    : nh_(nh)
    , has_state_(false)
    , landing_active_(false)
    , tf_listener_(tf_buffer_)
    , filter_initialized_(false)
  {
    // 1. Load Parameters (ROS Params & Physical Properties)
    loadParameters();

    // 2. Initialize Model (Physics)
    initQuadrotorModel();

    // 3. Initialize Controller (MPC)
    initMPC();

    // 4. Initialize ROS Communication (Pubs/Subs)
    initRosComms();

    // 5. Initialize Timers
    // MPC Control Loop (e.g., 20Hz)
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_), 
                                     &PlannerNode::mpcLoop, this);
    
    // Trajectory Publishing Loop (High Freq, e.g., 50Hz/100Hz)
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / trajectory_pub_rate_), 
                                     &PlannerNode::publishLoop, this);

    // 6. Init Dynamic Reconfigure
    dr_callback_ = boost::bind(&PlannerNode::reconfigureCallback, this, _1, _2);
    dr_server_.setCallback(dr_callback_);

    ROS_INFO("[PlannerNode] Initialized. Control Rate: %.2f Hz, Pub Rate: %.2f Hz", 
             control_rate_, trajectory_pub_rate_);
  }

private:
  // ========================================================================
  // 1. MEMBERS & STRUCTURES
  // ========================================================================
  
  ros::NodeHandle nh_;

  // --- ROS Communication ---
  ros::Subscriber state_sub_;
  ros::Subscriber trigger_sub_;
  
  ros::Publisher  cmd_pub_;
  ros::Publisher  halt_pub_;       // Emergency/Stop command
  ros::Publisher  perf_pub_;       // Profiling data
  ros::Publisher  mpc_path_pub_;   // Visual Path
  ros::Publisher  mpc_poses_pub_;  // Visual Poses (with orientation)

  ros::Timer      control_timer_;
  ros::Timer      publish_timer_;

  // --- TF2 (Landmark Tracking) ---
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // --- Configuration Parameters ---
  std::string state_topic_;
  std::string cmd_topic_;
  
  double control_rate_;         // MPC Frequency (Hz)
  double trajectory_pub_rate_;  // Publishing Frequency (Hz)
  int    prediction_horizon_;
  
  // Physical Parameters (Loaded via Param)
  double robot_mass_;
  double robot_gravity_;
  double thrust_limit_;
  double arm_length_;
  double force_torque_ratio_;
  std::vector<double> inertia_diag_; // [Ixx, Iyy, Izz]

  // Integrator Weights (for the extended state)
  double w_integrator_; 

  // --- Trajectory Sharing (Thread Safe) ---
  struct TrajectorySegment {
      ros::Time start_time;                 // When this trajectory was calculated
      double dt;                            // MPC time step
      std::vector<Eigen::VectorXd> states;  // Predicted states in WORLD frame
  };
  
  TrajectorySegment active_trajectory_;
  std::mutex traj_mutex_; 

  // --- State Management ---
  // Stored in MODEL FRAME
  // 13-dim: [px, py, pz | qw, qx, qy, qz | vx, vy, vz | wx, wy, wz ]
  Eigen::Matrix<double, 13, 1> current_state_model_;
  Eigen::VectorXd u_last_; // Accumulated thrust deviation (integrator state)
  
  bool has_state_;
  bool landing_active_;
  std::mutex state_mutex_;

  // --- Controller Architecture ---
  LinearQuadrotorModel quad_model_;
  std::unique_ptr<LMPC> mpc_controller_;

  // --- Dynamic Reconfigure ---
  dynamic_reconfigure::Server<xuanwu::PlannerConfig> dr_server_;
  dynamic_reconfigure::Server<xuanwu::PlannerConfig>::CallbackType dr_callback_;

  // --- Low Pass Filter (LPF) for Tag ---
  double lpf_alpha_ = 0.1; 
  bool filter_initialized_;
  Eigen::Vector3d t_tag_smooth_;
  double yaw_tag_smooth_ = 0.0;

  // ========================================================================
  // 2. INITIALIZATION METHODS
  // ========================================================================

  void loadParameters()
  {
    // Topics
    nh_.param<std::string>("state_topic", state_topic_, "/xuanwu/eskf/odom");
    nh_.param<std::string>("cmd_topic",   cmd_topic_,   "/xuanwu/uav/nav");

    // Loop Rates
    nh_.param<double>("control_rate", control_rate_, 20.0);
    nh_.param<double>("trajectory_pub_rate", trajectory_pub_rate_, 100.0);

    // MPC Config
    nh_.param<int>("prediction_horizon", prediction_horizon_, 20);

    // Physical Properties (Default to Xuanwu Specs if not set)
    nh_.param<double>("mass", robot_mass_, 1.9);
    nh_.param<double>("gravity", robot_gravity_, 9.81);
    nh_.param<double>("thrust_limit", thrust_limit_, 14.6232);
    
    // Arm length derived from: hypot(0.12728, 0.12731)
    nh_.param<double>("arm_length", arm_length_, 0.180); 
    nh_.param<double>("force_torque_ratio", force_torque_ratio_, 0.011);

    // Inertia
    nh_.param<std::vector<double>>("inertia_diag", inertia_diag_, {0.014009, 0.013451, 0.015441});

    // Default weight for the integrator (can be overwritten by cfg later if added)
    nh_.param<double>("integrator_weight_init", w_integrator_, 0.00001);
  }

  void initQuadrotorModel()
  {
    quad_model_.setMass(robot_mass_);
    quad_model_.setGravity(robot_gravity_);
    quad_model_.setThrustLimit(thrust_limit_);
    quad_model_.setArmLength(arm_length_); 
    quad_model_.setForceTorqueRatio(force_torque_ratio_);

    Eigen::Matrix3d J_robot = Eigen::Matrix3d::Zero();
    if(inertia_diag_.size() == 3) {
        J_robot(0,0) = inertia_diag_[0];
        J_robot(1,1) = inertia_diag_[1];
        J_robot(2,2) = inertia_diag_[2];
    } else {
        ROS_ERROR("Inertia param must have 3 elements! Using identity.");
        J_robot.setIdentity();
    }
    quad_model_.setInertia(J_robot);
  }

  void initMPC()
  {
    MPC::Config mpc_cfg;
    mpc_cfg.N = prediction_horizon_;
    mpc_cfg.dt = 1.0 / control_rate_;
    mpc_cfg.u_max = Eigen::VectorXd::Constant(4, quad_model_.getThrustLimit());

    mpc_controller_ = std::make_unique<LMPC>(mpc_cfg, quad_model_);
    u_last_ = Eigen::VectorXd::Zero(4);
  }

  void initRosComms()
  {
    state_sub_ = nh_.subscribe(state_topic_, 1, &PlannerNode::stateCallback, this);
    trigger_sub_ = nh_.subscribe("start_landing", 1, &PlannerNode::triggerCallback, this);

    cmd_pub_        = nh_.advertise<aerial_robot_msgs::FlightNav>(cmd_topic_, 1);
    // Removed unused debug_pose_pub_
    halt_pub_       = nh_.advertise<std_msgs::Empty>("/xuanwu/teleop_command/halt", 1);
    perf_pub_       = nh_.advertise<std_msgs::Float32MultiArray>("debug/mpc_performance", 1);
    
    // Visualization
    mpc_path_pub_  = nh_.advertise<nav_msgs::Path>("mpc_prediction_path", 1);
    mpc_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("mpc_prediction_poses", 1);
  }

  // ========================================================================
  // 3. CALLBACKS
  // ========================================================================

  void triggerCallback(const std_msgs::BoolConstPtr& msg) 
  {
      if (msg->data) {
          u_last_.setZero(4); // Reset integrator
          landing_active_ = true;
          ROS_WARN(">> LANDING SEQUENCE ACTIVATED (MPC Taking Control) <<");
      } else {
          landing_active_ = false;
          ROS_INFO(">> Landing Sequence Deactivated.");
      }
  }

  void stateCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Direct Copy: Odometry is expected to be in World Frame, Body Velocity
    // 1. Position
    current_state_model_(0) = msg->pose.pose.position.x;
    current_state_model_(1) = msg->pose.pose.position.y;
    current_state_model_(2) = msg->pose.pose.position.z;
 
    // 2. Orientation
    current_state_model_(3) = msg->pose.pose.orientation.w;
    current_state_model_(4) = msg->pose.pose.orientation.x;
    current_state_model_(5) = msg->pose.pose.orientation.y;
    current_state_model_(6) = msg->pose.pose.orientation.z;

    // 3. Linear Velocity (Body Frame)
    current_state_model_(7) = msg->twist.twist.linear.x;
    current_state_model_(8) = msg->twist.twist.linear.y;
    current_state_model_(9) = msg->twist.twist.linear.z;
    
    // 4. Angular Velocity (Body Frame)
    current_state_model_(10) = msg->twist.twist.angular.x;
    current_state_model_(11) = msg->twist.twist.angular.y;
    current_state_model_(12) = msg->twist.twist.angular.z;

    has_state_ = true;
  }

  void reconfigureCallback(xuanwu::PlannerConfig &config, uint32_t level)
  {
      ROS_INFO("[Planner] Reconfigure Request Received.");
  
      // 1. Update Control Rate
      if (std::abs(config.control_rate - control_rate_) > 1e-3) {
          control_rate_ = config.control_rate;
          control_timer_.setPeriod(ros::Duration(1.0 / control_rate_));
          ROS_INFO(">> MPC Rate updated to %.1f Hz", control_rate_);
      }

      // 2. Update Parameters
      prediction_horizon_ = config.horizon; 
      // Removed unused lookahead_steps
  
      // 3. Update Weights
      double w_int = config.Q_integrator; 

      Eigen::VectorXd Q_diag(16);
      Q_diag << config.Q_pos_x, config.Q_pos_y, config.Q_pos_z,   // Pos
                config.Q_att_r, config.Q_att_p, config.Q_att_y,   // Att
                config.Q_vel_x, config.Q_vel_y, config.Q_vel_z,   // Vel
                config.Q_omega, config.Q_omega, config.Q_omega,   // Omega
                w_int, w_int, w_int, w_int;                       // Integrator (u_last)

      Eigen::VectorXd R_diag(4);
      R_diag.setConstant(config.R_thrust);
  
      // 4. Propagate to Controller
      if(mpc_controller_) {
          mpc_controller_->updateConfig(prediction_horizon_, 1.0/control_rate_, Q_diag, R_diag);
      }
  }

  // ========================================================================
  // 4. MAIN LOOPS
  // ========================================================================

  void mpcLoop(const ros::TimerEvent& event)
  {
    if (!mpc_controller_ || !has_state_ || !landing_active_) return;

    auto start_loop_time = std::chrono::high_resolution_clock::now();

    // 1. Get Reference Frame (Landmark)
    Eigen::Isometry3d T_world_tag;
    if (!getSmoothedTagTransform(T_world_tag)) {
        return; // Filter/TF not ready
    }

    // 2. Prepare State (Transform World -> Tag)
    Eigen::Matrix<double, 13, 1> x_current_local;
    Eigen::Matrix<double, 13, 1> x_current_world;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        x_current_world = current_state_model_;
    }
    transformStateWorldToTag(x_current_world, T_world_tag, x_current_local);

    // 3. Define Reference (in Tag Frame)
    Eigen::Matrix<double, 13, 1> x_ref = Eigen::Matrix<double, 13, 1>::Zero();
    x_ref(2) = 0.0; // Target: 0.0m z-axis of tag (Assuming tag is grounded or target)
    x_ref(3) = 1.0; // Identity Quaternion

    // 4. Termination Check (Touchdown)
    double pos_error = (x_ref.head<3>() - x_current_local.head<3>()).norm();
    double vel_error = (x_ref.segment<3>(7) - x_current_local.segment<3>(7)).norm();

    // Thresholds could also be parameters
    if (pos_error < 0.05 && vel_error < 0.20) {
        ROS_WARN(">> TOUCHDOWN DETECTED (Err: %.2fm). HALTING MOTORS. <<", pos_error);
        halt_pub_.publish(std_msgs::Empty());
        landing_active_ = false;
        return;
    }

    // 5. Formulate Error State
    Eigen::VectorXd x0_error = computeErrorState(x_current_local, x_ref);

    // 6. SOLVE MPC
    auto start_opt_time = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd u_opt;
    std::vector<Eigen::VectorXd> prediction_horizon;
    
    bool success = mpc_controller_->solve(x0_error, u_opt, prediction_horizon);
    auto end_opt_time = std::chrono::high_resolution_clock::now();

    if (!success) {
        ROS_WARN_THROTTLE(1.0, "[MPC] Solver Failed.");
        return;
    }

    // 7. Update Integrator
    u_last_ += u_opt;

    // 8. Process Output for Publisher (Back to World Frame)
    processAndShareTrajectory(prediction_horizon, x_ref, T_world_tag);

    // 9. Visualization & Profiling
    publishVisuals(prediction_horizon, x_ref, "land_mark");
    publishProfiling(start_loop_time, start_opt_time, end_opt_time);
  }

  void publishLoop(const ros::TimerEvent& event)
  {
      if (!landing_active_) return;
  
      Eigen::VectorXd target_state; // [Px Py Pz Qw Qx Qy Qz Vx Vy Vz]
  
      // 1. Thread-safe trajectory read
      {
          std::lock_guard<std::mutex> lock(traj_mutex_);
          if (active_trajectory_.states.empty()) return;
  
          double time_elapsed = (ros::Time::now() - active_trajectory_.start_time).toSec();
          double dt = active_trajectory_.dt;
          
          // Interpolate
          interpolateTrajectory(active_trajectory_.states, dt, time_elapsed, target_state);
      }
  
      // 2. Publish Command
      aerial_robot_msgs::FlightNav cmd_msg;
      cmd_msg.header.stamp = ros::Time::now();
      cmd_msg.header.frame_id = "world";
      
      cmd_msg.pos_xy_nav_mode = aerial_robot_msgs::FlightNav::POS_VEL_MODE;
      cmd_msg.pos_z_nav_mode  = aerial_robot_msgs::FlightNav::POS_VEL_MODE;
      cmd_msg.yaw_nav_mode    = aerial_robot_msgs::FlightNav::POS_MODE;
      cmd_msg.control_frame   = aerial_robot_msgs::FlightNav::WORLD_FRAME;
      cmd_msg.target          = aerial_robot_msgs::FlightNav::COG;
  
      cmd_msg.target_pos_x = target_state(0);
      cmd_msg.target_pos_y = target_state(1);
      cmd_msg.target_pos_z = target_state(2);
  
      // Convert Quaternion to Yaw for Msg
      double r, p, y_cmd;
      tf2::Quaternion q_tf(target_state(4), target_state(5), target_state(6), target_state(3));
      tf2::Matrix3x3(q_tf).getRPY(r, p, y_cmd);
      cmd_msg.target_yaw = y_cmd;
  
      cmd_msg.target_vel_x = target_state(7);
      cmd_msg.target_vel_y = target_state(8);
      cmd_msg.target_vel_z = target_state(9);
  
      cmd_pub_.publish(cmd_msg);
  }

  // ========================================================================
  // 5. HELPER LOGIC
  // ========================================================================

  bool getSmoothedTagTransform(Eigen::Isometry3d& T_out)
  {
      try {
        geometry_msgs::TransformStamped tf_msg;
        tf_msg = tf_buffer_.lookupTransform("world", "land_mark", ros::Time(0));

        // Extract Raw
        Eigen::Vector3d t_raw(tf_msg.transform.translation.x,
                              tf_msg.transform.translation.y,
                              tf_msg.transform.translation.z + 0.15); // Offset? Make parameter if needed.

        tf2::Quaternion q_raw_tf;
        tf2::fromMsg(tf_msg.transform.rotation, q_raw_tf);
        double r_raw, p_raw, y_raw;
        tf2::Matrix3x3(q_raw_tf).getRPY(r_raw, p_raw, y_raw);

        // Apply LPF
        if (!filter_initialized_) {
            t_tag_smooth_   = t_raw;
            yaw_tag_smooth_ = y_raw;
            filter_initialized_ = true;
        } else {
            t_tag_smooth_ = (1.0 - lpf_alpha_) * t_tag_smooth_ + lpf_alpha_ * t_raw;
            
            // Yaw unwrapping logic
            double diff = y_raw - yaw_tag_smooth_;
            while (diff > M_PI)  diff -= 2.0 * M_PI;
            while (diff < -M_PI) diff += 2.0 * M_PI;
            yaw_tag_smooth_ += lpf_alpha_ * diff;
        }

        // Construct Transform
        Eigen::Quaterniond q_tag(Eigen::AngleAxisd(yaw_tag_smooth_, Eigen::Vector3d::UnitZ()));
        T_out = Eigen::Isometry3d::Identity();
        T_out.rotate(q_tag);
        T_out.pretranslate(t_tag_smooth_);

        return true;

    } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0, "[Planner] Waiting for 'land_mark' transform...");
        filter_initialized_ = false; 
        return false;
    }
  }

  void transformStateWorldToTag(const Eigen::Matrix<double, 13, 1>& x_world, 
                                const Eigen::Isometry3d& T_world_tag,
                                Eigen::Matrix<double, 13, 1>& x_local)
  {
      // 1. Position: p_tag = T_inv * p_world
      Eigen::Vector3d p_world = x_world.head<3>();
      Eigen::Vector3d p_tag = T_world_tag.inverse() * p_world;

      // 2. Orientation: q_tag = q_tag_world_inv * q_world
      Eigen::Quaterniond q_world(x_world(3), x_world(4), x_world(5), x_world(6));
      Eigen::Quaterniond q_tag_world(T_world_tag.rotation());
      Eigen::Quaterniond q_robot_tag = q_tag_world.inverse() * q_world;

      // 3. Velocities (Body Frame) - Unchanged
      Eigen::Vector3d v_body = x_world.segment<3>(7);
      Eigen::Vector3d w_body = x_world.tail<3>();

      x_local << p_tag, 
                 q_robot_tag.w(), q_robot_tag.vec(), 
                 v_body, w_body;
  }

  Eigen::VectorXd computeErrorState(const Eigen::Matrix<double, 13, 1>& x_curr, 
                                    const Eigen::Matrix<double, 13, 1>& x_ref)
  {
      Eigen::Vector3d dr = x_ref.head<3>() - x_curr.head<3>();
      
      Eigen::Vector4d q_ref_vec = x_ref.segment<4>(3);
      Eigen::Vector4d q_curr_vec = x_curr.segment<4>(3);
      Eigen::Vector3d dtheta = quatErrorRodrigues(q_curr_vec, q_ref_vec); // Helper fn

      Eigen::Vector3d dv = x_ref.segment<3>(7) - x_curr.segment<3>(7);
      Eigen::Vector3d dw = x_ref.tail<3>() - x_curr.tail<3>();

      Eigen::VectorXd x0_error(16);
      x0_error << dr, dtheta, dv, dw, u_last_; // 16-dim
      return x0_error;
  }

  void processAndShareTrajectory(const std::vector<Eigen::VectorXd>& horizon_errors,
                                 const Eigen::Matrix<double, 13, 1>& x_ref,
                                 const Eigen::Isometry3d& T_world_tag)
  {
      std::vector<Eigen::VectorXd> horizon_world;
      horizon_world.reserve(horizon_errors.size());

      Eigen::Vector3d p_ref_tag = x_ref.head<3>();
      Eigen::Vector4d q_ref_vec = x_ref.segment<4>(3);
      Eigen::Quaterniond q_world_tag(T_world_tag.rotation());

      for (const auto& x_err : horizon_errors) {
          // Reconstruct State in Tag Frame
          Eigen::Vector3d p_target_tag = p_ref_tag + x_err.head<3>();
          
          Eigen::Vector3d phi_next = x_err.segment<3>(3);
          Eigen::Vector4d dq_next = rodriguesToQuat<double>(phi_next);
          Eigen::Vector4d q_target_tag_vec = quatMultiply<double>(q_ref_vec, dq_next); // Helper

          // Transform to World
          Eigen::Vector3d p_target_world = T_world_tag * p_target_tag;
          
          Eigen::Quaterniond q_target_tag(q_target_tag_vec(0), q_target_tag_vec(1), 
                                          q_target_tag_vec(2), q_target_tag_vec(3));
          Eigen::Quaterniond q_target_world = q_world_tag * q_target_tag;

          // Velocity (Body -> World)
          Eigen::Vector3d v_target_body = x_ref.segment<3>(7) + x_err.segment<3>(6);
          Eigen::Vector3d v_target_world = q_target_world * v_target_body;

          // Store [Px Py Pz Qw Qx Qy Qz Vx Vy Vz]
          Eigen::VectorXd state_world(10);
          state_world << p_target_world, 
                         q_target_world.w(), q_target_world.x(), q_target_world.y(), q_target_world.z(),
                         v_target_world;
          horizon_world.push_back(state_world);
      }

      {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        active_trajectory_.start_time = ros::Time::now();
        active_trajectory_.dt = 1.0 / control_rate_;
        active_trajectory_.states = horizon_world;
      }
  }

  void interpolateTrajectory(const std::vector<Eigen::VectorXd>& states, 
                           double dt, double time_elapsed, 
                           Eigen::VectorXd& result)
  {
      int idx = std::floor(time_elapsed / dt);

      // Boundary check
      if (idx < 0) idx = 0;
      if (idx >= states.size() - 1) {
          result = states.back();
          return;
      }

      double alpha = (time_elapsed - (idx * dt)) / dt;
      const Eigen::VectorXd& s0 = states[idx];
      const Eigen::VectorXd& s1 = states[idx + 1];

      result = Eigen::VectorXd(10);

      // Position (Linear)
      result.head<3>() = (1.0 - alpha) * s0.head<3>() + alpha * s1.head<3>();

      // Orientation (Slerp)
      Eigen::Quaterniond q0(s0(3), s0(4), s0(5), s0(6));
      Eigen::Quaterniond q1(s1(3), s1(4), s1(5), s1(6));
      Eigen::Quaterniond q_interp = q0.slerp(alpha, q1);
      result(3) = q_interp.w();
      result(4) = q_interp.x();
      result(5) = q_interp.y();
      result(6) = q_interp.z();

      // Velocity (Linear)
      result.tail<3>() = (1.0 - alpha) * s0.tail<3>() + alpha * s1.tail<3>();
  }

  // ========================================================================
  // 6. DEBUG & VISUALIZATION
  // ========================================================================

  void publishVisuals(const std::vector<Eigen::VectorXd>& horizon_errors,
                      const Eigen::Matrix<double, 13, 1>& x_ref,
                      const std::string& frame_id) 
  {
      if (mpc_path_pub_.getNumSubscribers() == 0 && mpc_poses_pub_.getNumSubscribers() == 0) return;

      nav_msgs::Path path_msg;
      geometry_msgs::PoseArray poses_msg;
      path_msg.header.stamp = ros::Time::now();
      path_msg.header.frame_id = frame_id;
      poses_msg.header = path_msg.header;

      // Use helper to reconstruct poses from error state (Assuming implementation is available)
      // For brevity, using local logic similar to existing code
      Eigen::Vector3d p_ref = x_ref.head<3>();
      Eigen::Vector4d q_ref = x_ref.segment<4>(3);

      for (const auto& dx : horizon_errors) {
          geometry_msgs::PoseStamped pose_s;
          Eigen::Vector3d p_new = p_ref + dx.head<3>();
          
          pose_s.pose.position.x = p_new.x();
          pose_s.pose.position.y = p_new.y();
          pose_s.pose.position.z = p_new.z();

          Eigen::Vector3d dphi = dx.segment<3>(3);
          Eigen::Vector4d dq = rodriguesToQuat<double>(dphi);
          Eigen::Vector4d q_new = quatMultiply<double>(q_ref, dq);
          normalizeQuat<double>(q_new);

          pose_s.pose.orientation.w = q_new(0);
          pose_s.pose.orientation.x = q_new(1);
          pose_s.pose.orientation.y = q_new(2);
          pose_s.pose.orientation.z = q_new(3);

          path_msg.poses.push_back(pose_s);
          poses_msg.poses.push_back(pose_s.pose);
      }
      
      mpc_path_pub_.publish(path_msg);
      mpc_poses_pub_.publish(poses_msg);
  }

  void publishProfiling(std::chrono::time_point<std::chrono::high_resolution_clock> start_loop,
                        std::chrono::time_point<std::chrono::high_resolution_clock> start_opt,
                        std::chrono::time_point<std::chrono::high_resolution_clock> end_opt)
  {
      if (perf_pub_.getNumSubscribers() == 0) return;

      auto end_loop = std::chrono::high_resolution_clock::now();
      double opt_ms = std::chrono::duration<double, std::milli>(end_opt - start_opt).count();
      double loop_ms = std::chrono::duration<double, std::milli>(end_loop - start_loop).count();

      std_msgs::Float32MultiArray perf_msg;
      perf_msg.data.push_back(static_cast<float>(opt_ms));
      perf_msg.data.push_back(static_cast<float>(loop_ms));
      perf_pub_.publish(perf_msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_planner_node");
  ros::NodeHandle nh("~");
  PlannerNode node(nh);
  ros::spin();
  return 0;
}

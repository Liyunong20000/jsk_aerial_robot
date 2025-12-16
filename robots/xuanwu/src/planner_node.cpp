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

// ============================= ROS façade ==================================
class PlannerNode
{
public:
  explicit PlannerNode(ros::NodeHandle& nh)
    : nh_(nh)
    , has_state_(false)
  {
    // --- 1. Define Frame Transform (Robot -> Model) ---
    // User specified: "rotate my frame [Model] 135 degres around x you got the frame of xuanwu [Robot]"
    // This means R_off maps Model vectors to Robot vectors.
    // v_robot = R_off * v_model  =>  v_model = R_off^T * v_robot
    double angle_rad = 135.0 * M_PI / 180.0;
    R_off_ = Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();
    q_off_ = Eigen::Quaterniond(R_off_);

    ROS_INFO("Frame Offset R_off (Model->Robot, X-axis 135 deg) initialized.");

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
    
    // Transform Inertia to Model Frame: J_model = R^T * J_robot * R
    Eigen::Matrix3d J_model = R_off_.transpose() * J_robot * R_off_;
    quad_model_.setInertia(J_model);

    ROS_INFO_STREAM("Inertia Rotated to Model Frame:\n" << J_model);

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
  ros::Timer      control_timer_;

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
  Eigen::Matrix3d R_off_;
  Eigen::Quaterniond q_off_;

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
    
    // Position (World Frame - Assumed Shared Origin)
    current_state_model_(0) = msg->pose.pose.position.x;
    current_state_model_(1) = msg->pose.pose.position.y;
    current_state_model_(2) = msg->pose.pose.position.z;
    
    // Orientation (World Frame)
    // q_robot maps Robot->World. We need q_model maps Model->World.
    // Relation: Robot = R_off * Model
    // R_robot_world = R_model_world * R_off_transpose (if R maps Body to World?)
    // Standard: R_world_robot = R_world_model * R_model_robot
    // q_robot = q_model * q_off
    // => q_model = q_robot * q_off.inverse()
    Eigen::Quaterniond q_robot(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );
    Eigen::Quaterniond q_model = q_robot * q_off_.conjugate();
    
    current_state_model_(3) = q_model.w();
    current_state_model_(4) = q_model.x();
    current_state_model_(5) = q_model.y();
    current_state_model_(6) = q_model.z();
    
    // Linear Velocity (Body Frame)
    // v_model = R_off^T * v_robot
    Eigen::Vector3d v_robot(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z
    );
    Eigen::Vector3d v_model = R_off_.transpose() * v_robot;
    
    current_state_model_(7) = v_model.x();
    current_state_model_(8) = v_model.y();
    current_state_model_(9) = v_model.z();
    
    // Angular Velocity (Body Frame)
    Eigen::Vector3d w_robot(
        msg->twist.twist.angular.x,
        msg->twist.twist.angular.y,
        msg->twist.twist.angular.z
    );
    Eigen::Vector3d w_model = R_off_.transpose() * w_robot;

    current_state_model_(10) = w_model.x();
    current_state_model_(11) = w_model.y();
    current_state_model_(12) = w_model.z();

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

    Eigen::Matrix<double, 13, 1> x_current_model;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        x_current_model = current_state_model_;
    }

    // --- MPC Solve (In Model Frame) ---
    auto [x_hover, u_hover] = quad_model_.findHoverConditions();

    // 1. Position Error
    Eigen::Vector3d dr = x_current_model.head<3>() - x_hover.head<3>();
    
    // 2. Attitude Error (Quaternion Manifold)
    Eigen::Vector4d q_hover = x_hover.segment<4>(3);
    Eigen::Vector4d q_curr  = x_current_model.segment<4>(3);
    Eigen::Vector3d dtheta  = quatErrorRodrigues(q_hover, q_curr);

    // 3. Velocity Error
    Eigen::Vector3d dv = x_current_model.segment<3>(7) - x_hover.segment<3>(7);
    
    // 4. Omega Error
    Eigen::Vector3d dw = x_current_model.tail<3>() - x_hover.tail<3>();

    Eigen::VectorXd x0_error(12);
    x0_error << dr, dtheta, dv, dw;

    Eigen::VectorXd u_opt_delta;
    Eigen::VectorXd x_pred_first;
    
    bool success = mpc_controller_->solve(x0_error, u_opt_delta, x_pred_first);

    if (!success) {
        ROS_WARN_THROTTLE(1.0, "[Planner] MPC Solver Failed/Infeasible!");
        return;
    }

    // --- Output Conversion ---
    // Target Position
    Eigen::Vector3d r_cmd = x_hover.head<3>() + x_pred_first.head<3>();
    
    // Target Velocity (Model Body Frame)
    Eigen::Vector3d v_body_model = x_hover.segment<3>(7) + x_pred_first.segment<3>(6);
    
    // Convert to World Frame Velocity for JSK Controller
    // v_world = q_model * v_body_model
    Eigen::Quaterniond q_model_now(q_curr(0), q_curr(1), q_curr(2), q_curr(3));
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
    
    // Target Yaw (World Frame). For regulation to origin, 0.0 is fine.
    cmd_msg.target_yaw = 0.0; 

    cmd_pub_.publish(cmd_msg);

    // Debug Visualization
    geometry_msgs::PoseStamped debug_msg;
    debug_msg.header = cmd_msg.header;
    debug_msg.pose.position.x = r_cmd.x();
    debug_msg.pose.position.y = r_cmd.y();
    debug_msg.pose.position.z = r_cmd.z();
    debug_msg.pose.orientation.w = 1.0; // Identity orientation for viz
    debug_pub_.publish(debug_msg);
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

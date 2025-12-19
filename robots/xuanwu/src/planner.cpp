// planner.cpp
#include <ros/ros.h> // Added for ROS logging

// qpOASES Include
#include <qpOASES.hpp>

#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm> 
#include <Eigen/Dense>

// Autodiff headers
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

// Your provided helpers
#include "quat_helpers.hpp"

// =========================================================================
// TYPES
// =========================================================================
using Scalar = double;
using Vector13 = Eigen::Matrix<Scalar, 13, 1>; // r(3), q(4), v_body(3), w_body(3)
using Vector12 = Eigen::Matrix<Scalar, 12, 1>; // Reduced state (Gibbs)
using Vector4u = Eigen::Matrix<Scalar, 4, 1>;  // Inputs (Forces in Newtons)

// =========================================================================
// RK4 FREE FUNCTION
// =========================================================================
template <typename StateType, typename ControlType, typename T, typename Func>
StateType rk4_step(Func dynamics, const StateType& x, const ControlType& u, T h) {
    StateType k1 = dynamics(x, u);
    StateType k2 = dynamics(x + T(0.5) * h * k1, u);
    StateType k3 = dynamics(x + T(0.5) * h * k2, u);
    StateType k4 = dynamics(x + h * k3, u);

    return x + (h / T(6.0)) * (k1 + T(2.0) * k2 + T(2.0) * k3 + k4);
}

// =========================================================================
// CLASS: QUADROTOR
// =========================================================================
class Quadrotor {
protected:
    // Physical Parameters
    Scalar m_;                  // Mass (kg)
    Scalar g_;                  // Gravity (m/s^2)
    Scalar l_;                  // Arm length (m)
    Scalar force_torque_ratio_; // Ratio km/kt (m)
    Scalar thrust_limit_;       // Max thrust per motor (N)

    Eigen::Matrix3d J_;     // Inertia Matrix
    Eigen::Matrix3d J_inv_; // Inverse Inertia

public:
    Quadrotor() 
        : m_(1.0), g_(9.81), l_(0.25), force_torque_ratio_(0.02), thrust_limit_(10.0)
    {
        J_ = Eigen::Matrix3d::Identity() * 0.01;
        J_inv_ = J_.inverse();
    }

    void set_thrust_limit(Scalar limit_newtons) {
        thrust_limit_ = limit_newtons;
    }
    
    // Set other parameters
    void set_params(Scalar m, Scalar g, Scalar l, Scalar ratio, const Eigen::Matrix3d& J) {
        m_ = m; g_ = g; l_ = l; force_torque_ratio_ = ratio;
        J_ = J; J_inv_ = J_.inverse();
    }

    // =====================================================================
    // CONTINUOUS DYNAMICS
    // =====================================================================
    // x: [r(3); q(4); v_body(3); w_body(3)]
    // u: [f1; f2; f3; f4] (Newtons)
    template <typename T>
    Eigen::Matrix<T, 13, 1> continuousDynamics(const Eigen::Matrix<T, 13, 1>& x, const Eigen::Matrix<T, 4, 1>& u) const {
        using Vector3T = Eigen::Matrix<T, 3, 1>;
        using Vector4T = Eigen::Matrix<T, 4, 1>;
        using Matrix3T = Eigen::Matrix<T, 3, 3>;

        // 1. Unpack State
        Vector3T r = x.template head<3>();
        Vector4T q = x.template segment<4>(3); 
        
        // Normalize Quaternion
        T q_norm = q.norm();
        q = q / q_norm;

        Vector3T v_body = x.template segment<3>(7); 
        Vector3T w      = x.template tail<3>();

        // 2. Rotation Matrix Q (Body -> World)
        Matrix3T Q = quatToRotationMatrix<T>(q); 
        
        // 3. Position Dynamics: dr = Q * v
        Vector3T r_dot = Q * v_body;

        // 4. Attitude Dynamics: dq = 0.5 * L(q) * H * w
        Vector4T w_expanded; 
        w_expanded << T(0.0), w;
        Vector4T q_dot = T(0.5) * Lquat<T>(q) * w_expanded;

        // 5. Velocity Dynamics (Body Frame)
        Vector3T g_world; 
        g_world << T(0.0), T(0.0), T(-g_);
        
        // Transform gravity to body frame
        Vector3T g_body = Q.transpose() * g_world;

        // Thrust (sum of forces in Body Z)
        T total_force = u.sum();
        Vector3T thrust_accel; 
        thrust_accel << T(0.0), T(0.0), total_force / T(m_);

        Vector3T coriolis = w.cross(v_body);

        Vector3T v_dot = g_body + thrust_accel - coriolis;

        // 6. Angular Velocity Dynamics
        // Torque Matrix based on force inputs:
        T l_val = T(l_);
        T ratio = T(force_torque_ratio_);

        Vector3T tau;
        // Row 1: l*u2 - l*u4
        tau(0) = l_val * (u(1) - u(3));
        // Row 2: -l*u1 + l*u3
        tau(1) = l_val * (u(2) - u(0)); 
        // Row 3: ratio*(u1 - u2 + u3 - u4)
        tau(2) = ratio * (u(0) - u(1) + u(2) - u(3));

        Vector3T w_cross_Jw = w.cross(J_.cast<T>() * w);
        Vector3T w_dot = J_inv_.cast<T>() * (tau - w_cross_Jw);

        // Pack output
        Eigen::Matrix<T, 13, 1> x_dot;
        x_dot << r_dot, q_dot, v_dot, w_dot;
        
        return x_dot;
    }

    // =====================================================================
    // HOVER CONDITIONS
    // =====================================================================
    std::pair<Vector13, Vector4u> findHoverConditions() const {
        Vector13 x_hov = Vector13::Zero(); 
        x_hov(3) = 1.0;    // qw (Identity)
        
        // Hover: Total Force = m * g
        // 4 * u_i = m * g
        Scalar u_val = (m_ * g_) / 4.0;
        
        // ROS WARN CHECK
        if (u_val > thrust_limit_) {
            ROS_WARN("Hover thrust (%.2f N) exceeds limit (%.2f N)!", u_val, thrust_limit_);
        }

        Vector4u u_hov = Vector4u::Constant(u_val);

        return {x_hov, u_hov};
    } 
};

// =========================================================================
// CLASS: LINEAR QUADROTOR
// =========================================================================
class LinearQuadrotor : public Quadrotor {
public:
    using Quadrotor::Quadrotor;

    void discretizeAndLinearize(const Vector13& x_nom, const Vector4u& u_nom, Scalar dt,
                                Eigen::Matrix<Scalar, 13, 13>& A_out, 
                                Eigen::Matrix<Scalar, 13, 4>& B_out) 
    {
        using autodiff::real;
        using VectorXreal = Eigen::Matrix<real, Eigen::Dynamic, 1>;
        
        // Combined state for differentiation [x; u] (Size 17)
        VectorXreal z(17);
        for(int i=0; i<13; ++i) z(i) = x_nom(i);
        for(int i=0; i<4; ++i)  z(13+i) = u_nom(i);

        // Lambda for discrete step
        auto discrete_step_func = [&](const VectorXreal& z_in) -> VectorXreal {
            Eigen::Matrix<real, 13, 1> x = z_in.head<13>();
            Eigen::Matrix<real, 4, 1>  u = z_in.tail<4>();

            auto continuous_func = [&](const Eigen::Matrix<real, 13, 1>& s, const Eigen::Matrix<real, 4, 1>& c) {
                return this->continuousDynamics<real>(s, c);
            };

            return rk4_step(continuous_func, x, u, real(dt));
        };

        // Variable to hold the output of the function (x_next)
        VectorXreal F_real; 

        // Variable to hold the Jacobian
        Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic> J_real;

        // CORRECT CALL: Pass 5 arguments (func, wrt, at, output, jacobian)
        autodiff::jacobian(discrete_step_func, wrt(z), at(z), F_real, J_real);

        // Extract A (dx/dx) and B (dx/du)
        A_out = J_real.block<13, 13>(0, 0).cast<Scalar>();
        B_out = J_real.block<13, 4>(0, 13).cast<Scalar>();
    }
    
    // =====================================================================
    // REDUCE STATE
    // =====================================================================
    Vector12 reduceState(const Vector13& x_full) const {
        Vector12 x_red;
        x_red.head<3>() = x_full.head<3>(); // r
        
        Eigen::Vector4d q = x_full.segment<4>(3);
        q.normalize();
        
        // q -> Rodrigues
        x_red.segment<3>(3) = quatToRodrigues<Scalar>(q);
        
        x_red.tail<6>() = x_full.tail<6>(); // v, w
        return x_red;
    }
};

// =========================================================================
// MAIN NODE EXAMPLE
// =========================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "quadrotor_model_node");
    ros::NodeHandle nh;

    LinearQuadrotor model;
    
    // Set a low limit to trigger warning for demonstration
    model.set_thrust_limit(5.0); 

    ROS_INFO("Calculating hover conditions...");
    auto [x_hover, u_hover] = model.findHoverConditions();
    
    ROS_INFO_STREAM("Hover Force per motor: " << u_hover(0) << " N");

    Eigen::Matrix<double, 13, 13> A;
    Eigen::Matrix<double, 13, 4> B;
    
    ROS_INFO("Linearizing system...");
    model.discretizeAndLinearize(x_hover, u_hover, 0.05, A, B);

    ROS_INFO_STREAM("Linearization Complete. A(0,0): " << A(0,0));
    ROS_INFO_STREAM("B_z influence: " << B(9, 0));
    
    // --- Test qpOASES (basic hello world) ---
    
    // Matrix H (Hessian) - 1x1 matrix
    qpOASES::real_t H[1 * 1] = { 1.0 };
    
    // Vector g (Gradient)
    qpOASES::real_t g[1] = { 1.0 };
    
    // Lower and Upper Bounds
    qpOASES::real_t lb[1] = { 1.0 };
    qpOASES::real_t ub[1] = { 2.0 };

    // 1. Create a QProblem object (1 variable, 0 constraints)
    qpOASES::QProblem example(1, 0);

    // 2. Set options (disable print to stdout to avoid clutter)
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;
    example.setOptions(options);

    // 3. Solve the QP
    int nWSR = 10;
    qpOASES::returnValue status = example.init(H, g, nullptr, lb, ub, nullptr, nullptr, nWSR);

    // 4. Check results using ROS Logging
    if (status == qpOASES::SUCCESSFUL_RETURN) {
        qpOASES::real_t xOpt[1];
        example.getPrimalSolution(xOpt);
        
        ROS_INFO("SUCCESS: qpOASES is linked correctly!");
        ROS_INFO("Solution x = %f (Expected: 1.0)", xOpt[0]);
    } else {
        ROS_ERROR("FAILURE: qpOASES init failed with status %d", status);
    }

    // Keep node alive if needed
    // ros::spin(); 
    return 0;
}

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <cmath>
#include <iostream>

// Note: autodiff includes removed from here to speed up compilation of other nodes!

// Your provided helpers
#include "quat_helpers.hpp"

// =========================================================================
// CONSTANTS & TYPES
// =========================================================================
constexpr int FULL_STATE_DIM    = 13; // [r(3), q(4), v(3), w(3)]
constexpr int REDUCED_STATE_DIM = 12; // [dr(3), phi(3), dv(3), dw(3)]
constexpr int INPUT_DIM         = 4;  // [f1, f2, f3, f4]

using Scalar     = double;
using VectorFull = Eigen::Matrix<Scalar, FULL_STATE_DIM, 1>;
using VectorRed  = Eigen::Matrix<Scalar, REDUCED_STATE_DIM, 1>;
using VectorIn   = Eigen::Matrix<Scalar, INPUT_DIM, 1>;

using MatrixA    = Eigen::Matrix<Scalar, REDUCED_STATE_DIM, REDUCED_STATE_DIM>;
using MatrixB    = Eigen::Matrix<Scalar, REDUCED_STATE_DIM, INPUT_DIM>;
using MatrixE    = Eigen::Matrix<Scalar, FULL_STATE_DIM, REDUCED_STATE_DIM>;

// =========================================================================
// RK4 INTEGRATOR (Template must remain in header)
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
// BASE CLASS: QUADROTOR MODEL
// =========================================================================
class QuadrotorModel {
protected:
    // Physical Parameters
    Scalar m_;
    Scalar g_;
    Scalar l_;
    Scalar force_torque_ratio_;
    Scalar thrust_limit_;

    Eigen::Matrix3d J_;
    Eigen::Matrix3d J_inv_;

public:
    QuadrotorModel();
    virtual ~QuadrotorModel() = default;

    // Setters & Getters (Implementations moved to CPP if not trivial, 
    // but trivial ones are fine here or in CPP. Moved to CPP for cleanliness)
    void setMass(Scalar m);
    Scalar getMass() const;
    void setGravity(Scalar g);
    Scalar getGravity() const;
    void setArmLength(Scalar l);
    Scalar getArmLength() const;
    void setForceTorqueRatio(Scalar ratio);
    Scalar getForceTorqueRatio() const;
    void setInertia(const Eigen::Matrix3d& J);
    const Eigen::Matrix3d& getInertia() const;
    void setThrustLimit(Scalar limit);
    Scalar getThrustLimit() const;

    // Template method must stay in Header
    template <typename T>
    Eigen::Matrix<T, FULL_STATE_DIM, 1> 
    continuousDynamics(const Eigen::Matrix<T, FULL_STATE_DIM, 1>& x, const Eigen::Matrix<T, INPUT_DIM, 1>& u) const 
    {
        using Vector3T = Eigen::Matrix<T, 3, 1>;
        using Vector4T = Eigen::Matrix<T, 4, 1>;
        using Matrix3T = Eigen::Matrix<T, 3, 3>;

        // 1. Unpack State
        Vector3T r = x.template head<3>();
        Vector4T q = x.template segment<4>(3);
        
        T q_norm = q.norm();
        q = q / q_norm;

        Vector3T v_body = x.template segment<3>(7); 
        Vector3T w      = x.template tail<3>();

        // 2. Rotation Matrix R(q)
        Matrix3T R = quatToRotationMatrix<T>(q); 
        
        // 3. Position Dynamics
        Vector3T r_dot = R * v_body;

        // 4. Attitude Dynamics
        Vector4T w_expanded; 
        w_expanded << T(0.0), w;
        Vector4T q_dot = T(0.5) * Lquat<T>(q) * w_expanded;

        // 5. Velocity Dynamics
        Vector3T g_world; 
        g_world << T(0.0), T(0.0), T(-g_);
        Vector3T g_body = R.transpose() * g_world;
        T total_thrust = u.sum();
        Vector3T thrust_accel; 
        thrust_accel << T(0.0), T(0.0), total_thrust / T(m_);
        Vector3T coriolis = w.cross(v_body);
        Vector3T v_dot = g_body + thrust_accel - coriolis;

        // 6. Angular Velocity Dynamics
        T l_val = T(l_);
        T ratio = T(force_torque_ratio_);
        
        Vector3T tau_std;
        tau_std(0) = l_val * (u(1) - u(3));
        tau_std(1) = l_val * (u(2) - u(0)); 
        tau_std(2) = ratio * (u(0) - u(1) + u(2) - u(3));

        // Frame Rotation Fix (-135 deg)
        T angle_rad = T(-135.0 * M_PI / 180.0);
        T c = cos(angle_rad);
        T s = sin(angle_rad);

        Vector3T tau;
        tau(0) = c * tau_std(0) - s * tau_std(1);
        tau(1) = s * tau_std(0) + c * tau_std(1);
        tau(2) = tau_std(2); 

        Vector3T w_cross_Jw = w.cross(J_.cast<T>() * w);
        Vector3T w_dot = J_inv_.cast<T>() * (tau - w_cross_Jw);

        Eigen::Matrix<T, FULL_STATE_DIM, 1> x_dot;
        x_dot << r_dot, q_dot, v_dot, w_dot;
        
        return x_dot;
    }

    std::pair<VectorFull, VectorIn> findHoverConditions() const;
};

// =========================================================================
// DERIVED CLASS: LINEAR QUADROTOR
// =========================================================================
class LinearQuadrotorModel : public QuadrotorModel {
private:
    MatrixA A_; 
    MatrixB B_; 

public:
    using QuadrotorModel::QuadrotorModel; 

    const MatrixA& getA() const;
    const MatrixB& getB() const;

    // Definition moved to CPP to hide autodiff dependencies!
    void linearize(const VectorFull& x_nom, const VectorIn& u_nom, Scalar dt);
};

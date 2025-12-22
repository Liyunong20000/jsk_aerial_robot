#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <utility>
#include <cmath>

// Autodiff headers
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

// Your provided helpers (Must contain errorStateLift function)
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
// RK4 INTEGRATOR
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
    Scalar m_;                  // Mass (kg)
    Scalar g_;                  // Gravity (m/s^2)
    Scalar l_;                  // Arm length (m)
    Scalar force_torque_ratio_; // Ratio km/kt (m)
    Scalar thrust_limit_;       // Max thrust per motor (N)

    Eigen::Matrix3d J_;     // Inertia Matrix
    Eigen::Matrix3d J_inv_; // Inverse Inertia

public:
    QuadrotorModel() 
        : m_(1.0), g_(9.81), l_(0.25), force_torque_ratio_(0.02), thrust_limit_(15.0)
    {
        J_ = Eigen::Matrix3d::Identity() * 0.01;
        J_inv_ = J_.inverse();
    }

    virtual ~QuadrotorModel() = default;

    // ---------------------------------------------------------------------
    // Getters & Setters
    // ---------------------------------------------------------------------
    void setMass(Scalar m) { m_ = m; }
    Scalar getMass() const { return m_; }

    void setGravity(Scalar g) { g_ = g; }
    Scalar getGravity() const { return g_; }

    void setArmLength(Scalar l) { l_ = l; }
    Scalar getArmLength() const { return l_; }

    void setForceTorqueRatio(Scalar ratio) { force_torque_ratio_ = ratio; }
    Scalar getForceTorqueRatio() const { return force_torque_ratio_; }

    void setInertia(const Eigen::Matrix3d& J) { 
        J_ = J; 
        J_inv_ = J_.inverse(); 
    }
    const Eigen::Matrix3d& getInertia() const { return J_; }

    void setThrustLimit(Scalar limit) { thrust_limit_ = limit; }
    Scalar getThrustLimit() const { return thrust_limit_; }

    // ---------------------------------------------------------------------
    // CONTINUOUS DYNAMICS
    // x: [r(3); q(4); v_body(3); w_body(3)]
    // u: [f1; f2; f3; f4]
    // ---------------------------------------------------------------------
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
        
        // Normalize Quaternion
        T q_norm = q.norm();
        q = q / q_norm;

        Vector3T v_body = x.template segment<3>(7); 
        Vector3T w      = x.template tail<3>();

        // 2. Rotation Matrix R(q) (Body -> World)
        Matrix3T R = quatToRotationMatrix<T>(q); 
        
        // 3. Position Dynamics: dr = R * v_body
        Vector3T r_dot = R * v_body;

        // 4. Attitude Dynamics: dq = 0.5 * L(q) * H * w
        // Note: Lquat is defined in quat_helpers.hpp
        Vector4T w_expanded; 
        w_expanded << T(0.0), w;
        Vector4T q_dot = T(0.5) * Lquat<T>(q) * w_expanded;

        // 5. Velocity Dynamics (Body Frame)
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
        T ratio = T(force_torque_ratio_);// / T(thrust_limit_);
        
        // --- STANDARD TORQUE (Raw Geometry) ---
        // Assumes standard alignment (e.g., Motors 0,2 on Axis A; 1,3 on Axis B)
        Vector3T tau_std;
        tau_std(0) = l_val * (u(1) - u(3));
        tau_std(1) = l_val * (u(2) - u(0)); 
        tau_std(2) = ratio * (u(0) - u(1) + u(2) - u(3));

        // --- FRAME ROTATION FIX ---
        // User requested: "rotates 135 degrees counterclockwise"
        // If the Body Frame rotates +135 deg (CCW), the vector coordinates of the 
        // physical motor torques must rotate -135 deg (CW) to match the new axes.
        
        T angle_rad = T(-135.0 * M_PI / 180.0); // -135 deg for coordinate transform
        T c = cos(angle_rad);
        T s = sin(angle_rad);

        Vector3T tau;
        // Rotation Rz(angle) applied to the standard torque vector
        tau(0) = c * tau_std(0) - s * tau_std(1);
        tau(1) = s * tau_std(0) + c * tau_std(1);
        tau(2) = tau_std(2); // Z-torque is invariant to Z-rotation

        Vector3T w_cross_Jw = w.cross(J_.cast<T>() * w);
        Vector3T w_dot = J_inv_.cast<T>() * (tau - w_cross_Jw);

        // Pack output
        Eigen::Matrix<T, FULL_STATE_DIM, 1> x_dot;
        x_dot << r_dot, q_dot, v_dot, w_dot;
        
        return x_dot;
    }

    // ---------------------------------------------------------------------
    // Helper: Find Hover State & Input
    // ---------------------------------------------------------------------
    std::pair<VectorFull, VectorIn> findHoverConditions() const {
        VectorFull x_hov = VectorFull::Zero(); 
        x_hov(3) = 1.0; // Identity Quaternion (w=1)
        
        Scalar u_val = (m_ * g_) / 4.0;
        
        if (u_val > thrust_limit_) {
            std::cerr << "[QuadrotorModel] WARNING: Hover thrust exceeds limit!" << std::endl;
        }

        VectorIn u_hov = VectorIn::Constant(u_val);
        return {x_hov, u_hov};
    } 
};

// =========================================================================
// DERIVED CLASS: LINEAR QUADROTOR
// =========================================================================
class LinearQuadrotorModel : public QuadrotorModel {
private:
    MatrixA A_; // 12x12 (Reduced State)
    MatrixB B_; // 12x4  (Reduced State)

public:
    using QuadrotorModel::QuadrotorModel; // Inherit constructor

    // ---------------------------------------------------------------------
    // Getters for Matrices
    // ---------------------------------------------------------------------
    const MatrixA& getA() const { return A_; }
    const MatrixB& getB() const { return B_; }

    // ---------------------------------------------------------------------
    // LINEARIZE (Using Error State Lift Projection)
    // Computes A = E' * A_full * E and B = E' * B_full
    // ---------------------------------------------------------------------
    void linearize(const VectorFull& x_nom, const VectorIn& u_nom, Scalar dt) 
    {
        using autodiff::real;
        using VectorXreal = Eigen::Matrix<real, Eigen::Dynamic, 1>;
        
        // 1. Construct Combined State [x(13); u(4)] for Autodiff
        VectorXreal z(FULL_STATE_DIM + INPUT_DIM);
        for(int i=0; i<FULL_STATE_DIM; ++i) z(i) = x_nom(i);
        for(int i=0; i<INPUT_DIM; ++i)      z(FULL_STATE_DIM + i) = u_nom(i);

        //// 2. Define Discrete Step Function
        //// x_k+1 = f(x_k, u_k) via RK4
        //auto discrete_step_func = [&](const VectorXreal& z_in) -> VectorXreal {
        //    auto x = z_in.head<FULL_STATE_DIM>();
        //    auto u = z_in.tail<INPUT_DIM>();

        //    auto continuous_func = [&](const auto& s, const auto& c) {
        //        return this->continuousDynamics<real>(s, c);
        //    };

        //    return rk4_step(continuous_func, x, u, real(dt));
        //};
        auto discrete_step_func = [&](const VectorXreal& z_in) -> VectorXreal {
            // FIX: Explicitly cast Eigen::Block to Eigen::Matrix (Value Copy)
            // This prevents rk4_step template from deducing "Block" type which cannot be assigned to.
            Eigen::Matrix<real, FULL_STATE_DIM, 1> x = z_in.head<FULL_STATE_DIM>();
            Eigen::Matrix<real, INPUT_DIM, 1>      u = z_in.tail<INPUT_DIM>();

            auto continuous_func = [&](const auto& s, const auto& c) {
                return this->continuousDynamics<real>(s, c);
            };

            return rk4_step(continuous_func, x, u, real(dt));
        };

        // 3. Compute Full Jacobian (13x17)
        VectorXreal F_real; 
        Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic> J_real;
        
        // Autodiff: Compute J = [ dx_next/dx_curr | dx_next/du_curr ]
        autodiff::jacobian(discrete_step_func, autodiff::wrt(z), autodiff::at(z), F_real, J_real);

        // 4. Extract Full State Matrices
        // A_full is 13x13, B_full is 13x4
        Eigen::Matrix<Scalar, 13, 13> A_full = J_real.block<13, 13>(0, 0).cast<Scalar>();
        Eigen::Matrix<Scalar, 13, 4>  B_full = J_real.block<13, 4>(0, 13).cast<Scalar>();

        // 5. Compute Error State Lift Matrix E(q)
        // Extract quaternion from nominal state (indices 3,4,5,6)
        Eigen::Vector4d q_nom = x_nom.segment<4>(3);
        
        // E is 13x12 mapping Reduced -> Full
        MatrixE E = errorStateLift(q_nom); 

        // 6. Project to Reduced State
        // A_red = E^T * A_full * E
        // B_red = E^T * B_full
        
        // Note: For Hover Linearization, x_next approx x_curr, so E_next approx E_curr
        // Reference: "Planning With Attitude", Eq 28 [cite: 191]
        
        A_ = E.transpose() * A_full * E;
        B_ = E.transpose() * B_full;
    }
};

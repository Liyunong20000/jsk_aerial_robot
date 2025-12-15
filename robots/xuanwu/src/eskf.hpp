// eskf.hpp
#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include <Eigen/Dense>
#include <vector>

#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

#include "quat_helpers.hpp"
#include "state_wrappers.hpp"

// ======================= ESKF core (no ROS node stuff) ======================
class Eskf
{
public:
  Eskf(const Eigen::Matrix<double, ERR_STATE_DIM, 1>& q_proc_diag,
       const Eigen::Matrix<double, 6, 1>&             r_cam_diag,
       const Eigen::Matrix<double, ERR_STATE_DIM, 1>& r_box_diag)
  : initialized_(false)
  {
    // Nominal state init
    x_nom_.setZero();
    x_nom_(3) = 1.0;  // identity quaternion

    // Covariance init
    P_.setIdentity();
    P_ *= 1e3;

    Q_proc_ = q_proc_diag.asDiagonal();
    R_cam_  = r_cam_diag.asDiagonal();
    R_box_  = r_box_diag.asDiagonal();
  }

  // --- Setters for Dynamic Reconfigure ---
  void setProcessNoise(const Eigen::Matrix<double, ERR_STATE_DIM, ERR_STATE_DIM>& Q) {
    Q_proc_ = Q;
  }

  void setCamMeasurementNoise(const Eigen::Matrix<double, 6, 6>& R) {
    R_cam_ = R;
  }

  void setBoxMeasurementNoise(const Eigen::Matrix<double, ERR_STATE_DIM, ERR_STATE_DIM>& R) {
    R_box_ = R;
  }

  bool isInitialized() const { return initialized_; }

  // Initialize nominal state from black-box odometry
  void initializeFromBox(const nav_msgs::Odometry& msg)
  {
    auto [r, q, v, w] = stateFromMsg(msg);
    x_nom_ << r, q, v, w;

    P_.setIdentity();
    P_ *= 1.0;  // smaller uncertainty after init

    initialized_ = true;
  }

  // Prediction step with dt
  void predict(double dt)
  {
    if (dt <= 0.0) return;

    Eigen::Matrix<double, NOM_STATE_DIM,1> x_prev = x_nom_;
    Eigen::Vector4d q_prev = x_prev.segment<4>(3);

    Eigen::Matrix<double,NOM_STATE_DIM,NOM_STATE_DIM> Gx =
        computeProcessJacobian(x_prev, dt);

    propagateNominal(dt);
    Eigen::Vector4d q_next = x_nom_.segment<4>(3);

    Eigen::Matrix<double,13,12> E_prev = errorStateLift(q_prev);
    Eigen::Matrix<double,13,12> E_next = errorStateLift(q_next);

    Eigen::Matrix<double,ERR_STATE_DIM,ERR_STATE_DIM> F_delta =
        E_next.transpose() * Gx * E_prev;

    P_ = F_delta * P_ * F_delta.transpose() + Q_proc_;
  }

  // Measurement update with black-box odom
  void updateWithBox(const nav_msgs::Odometry& msg)
  {
    auto [r_nom, q_nom, v_nom, w_nom] = unpackNominalState(x_nom_);
    auto [r_meas, q_meas, v_meas, w_meas] = stateFromMsg(msg);

    Eigen::Matrix<double, ERR_STATE_DIM, 1> dz;
    dz.segment<3>(0) = r_meas - r_nom;
    dz.segment<3>(3) = quatErrorRodrigues(q_nom, q_meas);
    dz.segment<3>(6) = v_meas - v_nom;
    dz.segment<3>(9) = w_meas - w_nom;

    Eigen::Matrix<double, ERR_STATE_DIM, ERR_STATE_DIM> H;
    H.setIdentity();

    Eigen::Matrix<double, ERR_STATE_DIM, ERR_STATE_DIM> S =
        H * P_ * H.transpose() + R_box_; // Eq(33)
    Eigen::Matrix<double, ERR_STATE_DIM, ERR_STATE_DIM> K =
        P_ * H.transpose() * S.inverse(); // Eq(33)

    Eigen::Matrix<double, ERR_STATE_DIM, 1> dx = K * dz; // Eq(34)

    Eigen::Matrix<double, ERR_STATE_DIM, ERR_STATE_DIM> I;
    I.setIdentity();
    P_ = (I - K * H) * P_; // Eq(36)

    injectError(dx); // Eq(35)
  }

  // Measurement update with camera odom (pos + attitude)
  void updateWithCam(const nav_msgs::Odometry& msg)
  {
    Eigen::Vector3d r_nom = x_nom_.segment<3>(0);
    Eigen::Vector4d q_nom = x_nom_.segment<4>(3);

    Eigen::Vector3d r_meas;
    r_meas << msg.pose.pose.position.x,
              msg.pose.pose.position.y,
              msg.pose.pose.position.z;

    Eigen::Vector4d q_meas = quatFromMsg(msg.pose.pose.orientation);

    Eigen::Matrix<double, 6, 1> dz;
    dz.segment<3>(0) = r_meas - r_nom;
    dz.segment<3>(3) = quatErrorRodrigues(q_nom, q_meas);

    Eigen::Matrix<double, 6, ERR_STATE_DIM> H;
    H.setZero();
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    H.block<3,3>(3,3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R_cam_; // Eq(33)
    Eigen::Matrix<double, ERR_STATE_DIM, 6> K = P_ * H.transpose() * S.inverse(); // Eq(33)

    Eigen::Matrix<double, ERR_STATE_DIM, 1> dx = K * dz; // Eq (34)

    Eigen::Matrix<double, ERR_STATE_DIM, ERR_STATE_DIM> I;
    I.setIdentity();
    P_ = (I - K * H) * P_; // Eq(36)

    injectError(dx); // Eq(35)
  }

  // Build an Odometry message from the nominal state (no publishing here)
  nav_msgs::Odometry makeOdometry(const ros::Time& stamp,
                                  const std::string& frame_id,
                                  const std::string& child_frame_id) const
  {
    nav_msgs::Odometry out;
    out.header.stamp = stamp;
    out.header.frame_id = frame_id;
    out.child_frame_id = child_frame_id;

    auto [r, q, v, w] = unpackNominalState(x_nom_);

    out.pose.pose.position.x = r(0);
    out.pose.pose.position.y = r(1);
    out.pose.pose.position.z = r(2);
    quatToMsg(q, out.pose.pose.orientation);

    out.twist.twist.linear.x  = v(0);
    out.twist.twist.linear.y  = v(1);
    out.twist.twist.linear.z  = v(2);
    out.twist.twist.angular.x = w(0);
    out.twist.twist.angular.y = w(1);
    out.twist.twist.angular.z = w(2);

    return out;
  }

private:
  // State
  Eigen::Matrix<double, NOM_STATE_DIM, 1> x_nom_;
  Eigen::Matrix<double, ERR_STATE_DIM, ERR_STATE_DIM> P_;
  Eigen::Matrix<double, ERR_STATE_DIM, ERR_STATE_DIM> Q_proc_;
  Eigen::Matrix<double, ERR_STATE_DIM, ERR_STATE_DIM> R_box_;
  Eigen::Matrix<double, 6, 6> R_cam_;
  bool initialized_;

  // --- Helpers ---

  void injectError(const Eigen::Matrix<double, ERR_STATE_DIM, 1>& dx)
  {
    auto [dr, phi, dv, dw] = unpackErrorState(dx);
    auto [r, q, v, w] = unpackNominalState(x_nom_);

    r += dr;
    v += dv;
    w += dw;

    Eigen::Vector4d dq = rodriguesToQuat<double>(phi);
    q = quatMultiply<double>(q, dq);
    normalizeQuat(q);

    x_nom_ << r, q, v, w;
  }

  void propagateNominal(double dt)
  {
    auto [r, q, v, w] = unpackNominalState(x_nom_);

    Eigen::Matrix3d A = quatToRotationMatrix<double>(q);

    r += A * v * dt;
    q = integrateQuatEuler<double>(q, w, dt);

    x_nom_ << r, q, v, w;
  }

  Eigen::Matrix<double, NOM_STATE_DIM, NOM_STATE_DIM>
  computeProcessJacobian(const Eigen::Matrix<double, NOM_STATE_DIM, 1>& x,
                         double dt) const
  {
    using autodiff::real;
    using autodiff::VectorXreal;

    VectorXreal xad(NOM_STATE_DIM);
    for (int i = 0; i < NOM_STATE_DIM; ++i)
      xad(i) = real(x(i));

    auto g_fun = [dt](const VectorXreal& xad_in) -> VectorXreal
    {
      VectorXreal y(NOM_STATE_DIM);

      Eigen::Matrix<real,3,1> r = xad_in.segment(0, 3);
      Eigen::Matrix<real,4,1> q = xad_in.segment(3, 4);
      Eigen::Matrix<real,3,1> v = xad_in.segment(7, 3);
      Eigen::Matrix<real,3,1> w = xad_in.segment(10, 3);

      r = r + v * real(dt);
      q = integrateQuatEuler<real>(q, w, dt);

      y.segment(0,3)   = r;
      y.segment(3,4)   = q;
      y.segment(7,3)   = v;
      y.segment(10,3)  = w;

      return y;
    };

    VectorXreal y;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J;
    autodiff::jacobian(g_fun, autodiff::wrt(xad), autodiff::at(xad), y, J);

    Eigen::Matrix<double, NOM_STATE_DIM, NOM_STATE_DIM> Jd = J;
    return Jd;
  }
};

// state_wrappers.hpp
#pragma once

#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

#include "quat_helpers.hpp"

// Nominal state: [ r(3), q(4), v(3), w(3) ]  (q = [qs, qx, qy, qz])
  static constexpr int NOM_STATE_DIM = 13;
  // Error state: [ δr(3), φ(3), δv(3), δω(3) ]
  static constexpr int ERR_STATE_DIM = 12;


struct StateVectors {
    Eigen::Vector3d r;
    Eigen::Vector4d q;
    Eigen::Vector3d v;
    Eigen::Vector3d w;
};

struct ErrorStateVectors {
  Eigen::Vector3d dr;
  Eigen::Vector3d phi; // Gibbs - Rodrigues parameters
  Eigen::Vector3d dv;
  Eigen::Vector3d dw;
};

struct NominalStateVectors {
  Eigen::Vector3d r;
  Eigen::Vector4d q;
  Eigen::Vector3d v;
  Eigen::Vector3d w;
};

inline ErrorStateVectors unpackErrorState(const Eigen::Matrix<double, ERR_STATE_DIM, 1>& dx)
{
  return {
    dx.segment<3>(0),  // dr
    dx.segment<3>(3),  // phi (Rodrigues)
    dx.segment<3>(6),  // dv
    dx.segment<3>(9)   // dw
  };
}

inline NominalStateVectors unpackNominalState(const Eigen::Matrix<double, NOM_STATE_DIM, 1>& x)
{
  return {
    x.segment<3>(0),   // r
    x.segment<4>(3),   // q
    x.segment<3>(7),   // v
    x.segment<3>(10)   // w
  };
}

// Convert ROS quaternion (x,y,z,w) to scalar-first Eigen q = [w,x,y,z]^T
static Eigen::Vector4d quatFromMsg(const geometry_msgs::Quaternion& msg)
{
  Eigen::Vector4d q;
  q(0) = msg.w;
  q(1) = msg.x;
  q(2) = msg.y;
  q(3) = msg.z;
  normalizeQuat(q);
  return q;
}

// Convert scalar-first Eigen q = [w,x,y,z]^T back to ROS msg (x,y,z,w)
static void quatToMsg(const Eigen::Vector4d& q, geometry_msgs::Quaternion& msg)
{
  Eigen::Vector4d qn = q;
  normalizeQuat(qn);
  msg.w = qn(0);
  msg.x = qn(1);
  msg.y = qn(2);
  msg.z = qn(3);
}

StateVectors stateFromMsg(const nav_msgs::Odometry& msg)
{
    Eigen::Vector3d r;
    r << msg.pose.pose.position.x,
         msg.pose.pose.position.y,
         msg.pose.pose.position.z;

    Eigen::Vector4d q = quatFromMsg(msg.pose.pose.orientation);

    Eigen::Vector3d v;
    v << msg.twist.twist.linear.x,
         msg.twist.twist.linear.y,
         msg.twist.twist.linear.z;

    Eigen::Vector3d w;
    w << msg.twist.twist.angular.x,
         msg.twist.twist.angular.y,
         msg.twist.twist.angular.z;

    // aggregate initialization of the struct
    return {r, q, v, w};
}

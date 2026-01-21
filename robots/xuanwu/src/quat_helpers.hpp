#ifndef QUATERNION_HELPERS_HPP
#define QUATERNION_HELPERS_HPP

#include <Eigen/Dense>
//#include <geometry_msgs/Quaternion.h>
//#include <ostream>
//#include <cmath>

// ==================== Quaternion / Jackson-style helpers ====================

// Skew-symmetric hat(v) used in L(q)
template<typename Scalar>
static Eigen::Matrix<Scalar,3,3> hat(const Eigen::Matrix<Scalar,3,1>& v)
{
  Eigen::Matrix<Scalar,3,3> M;
  M << Scalar(0),      -v(2),        v(1),
       v(2),      Scalar(0),       -v(0),
      -v(1),          v(0),  Scalar(0);
  return M;
}

// Left quaternion multiplication matrix L(q) such that q ⊗ p = L(q) p
template<typename Scalar>
static Eigen::Matrix<Scalar,4,4> Lquat(const Eigen::Matrix<Scalar,4,1>& q)
{
  const Scalar s = q(0);
  const Eigen::Matrix<Scalar,3,1> v = q.template segment<3>(1);

  Eigen::Matrix<Scalar,4,4> L;
  L.setZero();
  L(0,0) = s;
  L.template block<1,3>(0,1) = -v.transpose();
  L.template block<3,1>(1,0) =  v;
  L.template block<3,3>(1,1) =  s * Eigen::Matrix<Scalar,3,3>::Identity() + hat(v);
  return L;
}

// Quaternion conjugate
template<typename Scalar>
static Eigen::Matrix<Scalar,4,1> quatConjugate(const Eigen::Matrix<Scalar,4,1>& q)
{
  Eigen::Matrix<Scalar,4,1> qc;
  qc(0) = q(0);
  qc.template segment<3>(1) = -q.template segment<3>(1);
  return qc;
}

// Quaternion product via L(q1)
template<typename Scalar>
static Eigen::Matrix<Scalar,4,1> quatMultiply(const Eigen::Matrix<Scalar,4,1>& q1,
                                              const Eigen::Matrix<Scalar,4,1>& q2)
{
  return Lquat(q1) * q2;
}

// Normalize quaternion (no special-casing zero; we assume we're never *that* broken)
template<typename Scalar>
static void normalizeQuat(Eigen::Matrix<Scalar,4,1>& q)
{
  Scalar n = q.norm();
  q /= n;
}

// Rodrigues parameter -> quaternion (rptoq)
// ϕ ∈ R^3, q = [1; ϕ] / sqrt(1 + ϕᵀϕ)
template<typename Scalar>
static Eigen::Matrix<Scalar,4,1> rodriguesToQuat(const Eigen::Matrix<Scalar,3,1>& phi)
{
  Scalar one = Scalar(1);
  Scalar denom = sqrt(one + phi.dot(phi));
  Eigen::Matrix<Scalar,4,1> q;
  q(0) = one / denom;
  q.template segment<3>(1) = phi / denom;
  return q;
}

// Quaternion -> Rodrigues parameter (qtorp)
// q = [s; v], ϕ = v / s
template<typename Scalar>
static Eigen::Matrix<Scalar,3,1> quatToRodrigues(const Eigen::Matrix<Scalar,4,1>& q)
{
  Scalar s = q(0);
  Eigen::Matrix<Scalar,3,1> v = q.template segment<3>(1);
  return v / s;
}

// Attitude Jacobian G(q) = L(q) H, H = [0; I]
template<typename Scalar>
static Eigen::Matrix<Scalar,4,3> Gquat(const Eigen::Matrix<Scalar,4,1>& q)
{
  Eigen::Matrix<Scalar,4,3> H;
  H.setZero();
  H.template block<3,3>(1,0) = Eigen::Matrix<Scalar,3,3>::Identity();
  return Lquat(q) * H;
}

// State-lift matrix E(q): 13 x 12
// δx_full = E(q) δx_err
// Full: [δr(3); δq(4); δv(3); δω(3)]
// Err : [δr(3); φ(3); δv(3); δω(3)]
static Eigen::Matrix<double,13,12> errorStateLift(const Eigen::Vector4d& q)
{
  Eigen::Matrix<double,13,12> E;
  E.setZero();

  // δr
  E.block<3,3>(0,0) = Eigen::Matrix3d::Identity();

  // δq ≈ G(q) φ
  Eigen::Matrix<double,4,3> Gq = Gquat<double>(q);
  E.block<4,3>(3,3) = Gq;

  // δv
  E.block<3,3>(7,6)  = Eigen::Matrix3d::Identity();
  // δω
  E.block<3,3>(10,9) = Eigen::Matrix3d::Identity();

  return E;
}

// Integrate quaternion with Euler step on q̇ = 0.5 L(q)[0;ω] (no trig, AD-friendly)
template<typename Scalar>
static Eigen::Matrix<Scalar,4,1> integrateQuatEuler(
    const Eigen::Matrix<Scalar,4,1>& q,
    const Eigen::Matrix<Scalar,3,1>& w,
    double dt)
{
  Eigen::Matrix<Scalar,4,1> omega_quat;
  omega_quat(0) = Scalar(0);
  omega_quat.template segment<3>(1) = w;

  Eigen::Matrix<Scalar,4,1> qdot = Scalar(0.5) * (Lquat(q) * omega_quat);
  Eigen::Matrix<Scalar,4,1> qnew = q + Scalar(dt) * qdot;
  normalizeQuat(qnew);
  return qnew;
}

// Attitude error as Rodrigues parameter:
// q_rel = q_nom^{-1} ⊗ q_meas, enforce shortest rotation, φ = qtorp(q_rel)
static Eigen::Vector3d quatErrorRodrigues(const Eigen::Vector4d& q_nom,
                                          const Eigen::Vector4d& q_meas)
{
  Eigen::Vector4d q_nom_conj = quatConjugate<double>(q_nom);
  Eigen::Vector4d q_rel = quatMultiply<double>(q_nom_conj, q_meas);
  // Use shortest rotation
  if (q_rel(0) < 0.0)
    q_rel = -q_rel;
  return quatToRodrigues<double>(q_rel);
}

// Quaternion -> rotation matrix A(q)
//   A = H' * T * L(q) * T * L(q) * H
// where T = diag(1, -1, -1, -1), H = [0; I].
template<typename Scalar>
static Eigen::Matrix<Scalar,3,3> quatToRotationMatrix(
    Eigen::Matrix<Scalar,4,1> q)
{
  // Ensure unit quaternion
  normalizeQuat(q);

  // T = diag(1, -1, -1, -1)
  Eigen::Matrix<Scalar,4,4> T = Eigen::Matrix<Scalar,4,4>::Identity();
  T(1,1) = Scalar(-1);
  T(2,2) = Scalar(-1);
  T(3,3) = Scalar(-1);

  // H = [0; I_3]  (4x3)
  Eigen::Matrix<Scalar,4,3> H;
  H.setZero();
  H.template block<3,3>(1,0) = Eigen::Matrix<Scalar,3,3>::Identity();

  // L(q)
  Eigen::Matrix<Scalar,4,4> L = Lquat(q);

  // A = H' * T * L(q) * T * L(q) * H
  Eigen::Matrix<Scalar,3,3> A =
      H.transpose() * T * L * T * L * H;

  return A;
}

#endif // QUATERNION_HELPERS_HPP


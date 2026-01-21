#include "quadrotor_model.hpp"

// Autodiff headers - ONLY INCLUDED HERE
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

// =========================================================================
// QuadrotorModel Implementations
// =========================================================================

QuadrotorModel::QuadrotorModel() 
    : m_(1.0), g_(9.81), l_(0.25), force_torque_ratio_(0.02), thrust_limit_(15.0)
{
    J_ = Eigen::Matrix3d::Identity() * 0.01;
    J_inv_ = J_.inverse();
}

void QuadrotorModel::setMass(Scalar m) { m_ = m; }
Scalar QuadrotorModel::getMass() const { return m_; }

void QuadrotorModel::setGravity(Scalar g) { g_ = g; }
Scalar QuadrotorModel::getGravity() const { return g_; }

void QuadrotorModel::setArmLength(Scalar l) { l_ = l; }
Scalar QuadrotorModel::getArmLength() const { return l_; }

void QuadrotorModel::setForceTorqueRatio(Scalar ratio) { force_torque_ratio_ = ratio; }
Scalar QuadrotorModel::getForceTorqueRatio() const { return force_torque_ratio_; }

void QuadrotorModel::setInertia(const Eigen::Matrix3d& J) { 
    J_ = J; 
    J_inv_ = J_.inverse(); 
}

// FIX: Added QuadrotorModel:: scope here
const Eigen::Matrix3d& QuadrotorModel::getInertia() const { return J_; }

void QuadrotorModel::setThrustLimit(Scalar limit) { thrust_limit_ = limit; }
Scalar QuadrotorModel::getThrustLimit() const { return thrust_limit_; }

std::pair<VectorFull, VectorIn> QuadrotorModel::findHoverConditions() const {
    VectorFull x_hov = VectorFull::Zero(); 
    x_hov(3) = 1.0; // Ensure unit quaternion 
    
    Scalar u_val = (m_ * g_) / 4.0;
    
    if (u_val > thrust_limit_) {
        std::cerr << "[QuadrotorModel] WARNING: Hover thrust exceeds limit!" << std::endl;
    }

    VectorIn u_hov = VectorIn::Constant(u_val);
    return {x_hov, u_hov};
}

// =========================================================================
// LinearQuadrotorModel Implementations
// =========================================================================

const MatrixA& LinearQuadrotorModel::getA() const { return A_; }
const MatrixA_aug& LinearQuadrotorModel::getA_aug() const { return A_aug_; }
const MatrixB& LinearQuadrotorModel::getB() const { return B_; }
const MatrixB_aug& LinearQuadrotorModel::getB_aug() const { return B_aug_; }

void LinearQuadrotorModel::linearize(const VectorFull& x_nom, const VectorIn& u_nom, Scalar dt) 
{
    using autodiff::real;
    using VectorXreal = Eigen::Matrix<real, Eigen::Dynamic, 1>;
    
    // 1. Construct Combined State
    VectorXreal z(FULL_STATE_DIM + INPUT_DIM);
    for(int i=0; i<FULL_STATE_DIM; ++i) z(i) = x_nom(i);
    for(int i=0; i<INPUT_DIM; ++i)      z(FULL_STATE_DIM + i) = u_nom(i);

    // 2. Define Discrete Step Function
    auto discrete_step_func = [&](const VectorXreal& z_in) -> VectorXreal {
        // Fix for Block assignment issues in templates
        Eigen::Matrix<real, FULL_STATE_DIM, 1> x = z_in.head<FULL_STATE_DIM>();
        Eigen::Matrix<real, INPUT_DIM, 1>      u = z_in.tail<INPUT_DIM>();

        auto continuous_func = [&](const auto& s, const auto& c) {
            return this->continuousDynamics<real>(s, c);
        };

        return rk4_step(continuous_func, x, u, real(dt));
    };

    // 3. Compute Jacobian
    VectorXreal F_real; 
    Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic> J_real;
    
    autodiff::jacobian(discrete_step_func, autodiff::wrt(z), autodiff::at(z), F_real, J_real);

    MatrixA_full A_full = J_real.block<FULL_STATE_DIM, FULL_STATE_DIM>(0, 0).cast<Scalar>();
    MatrixB_full B_full = J_real.block<FULL_STATE_DIM, INPUT_DIM>(0, FULL_STATE_DIM).cast<Scalar>();

    // 4. Lift Projection
    Eigen::Vector4d q_nom = x_nom.segment<INPUT_DIM>(3);
    MatrixE E = errorStateLift(q_nom); 

    A_ = E.transpose() * A_full * E;
    B_ = E.transpose() * B_full;

    // 5. Construct Augmented Matrices
    A_aug_.setZero();
    B_aug_.setZero();
    
    // --------------------------------------------------------
    // Formulation:
    //   x_{k+1} = A * x_k     + B * u_{k-1} + B * du_k
    //   u_{k}   = 0 * x_k     + I * u_{k-1} + I * du_k
    // --------------------------------------------------------
    // Augmented State: z_k = [x_k, u_{k-1}]
    // Augmented Input: v_k = du_k


    A_aug_.block(0 , 0, REDUCED_STATE_DIM, REDUCED_STATE_DIM) = A_;
    A_aug_.block(0 , REDUCED_STATE_DIM, REDUCED_STATE_DIM, INPUT_DIM) = B_;
    A_aug_.block(REDUCED_STATE_DIM, REDUCED_STATE_DIM, INPUT_DIM, INPUT_DIM) = Eigen::MatrixXd::Identity(INPUT_DIM, INPUT_DIM);
    B_aug_.block(0, 0, REDUCED_STATE_DIM, INPUT_DIM) = B_;
    B_aug_.block(REDUCED_STATE_DIM, 0, INPUT_DIM, INPUT_DIM) = Eigen::MatrixXd::Identity(INPUT_DIM, INPUT_DIM);
}

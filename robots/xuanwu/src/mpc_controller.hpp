#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <qpOASES.hpp>

#include "quadrotor_model.hpp"

// =========================================================================
// HELPER: Robust DARE Solver (Structure-Preserving Doubling Algorithm)
// =========================================================================
inline Eigen::MatrixXd solveDARE(const Eigen::MatrixXd& A, 
                                 const Eigen::MatrixXd& B, 
                                 const Eigen::MatrixXd& Q, 
                                 const Eigen::MatrixXd& R, 
                                 double tolerance = 1e-10, 
                                 int max_iter = 50) 
{
    Eigen::MatrixXd A_k = A;
    Eigen::MatrixXd G_k = B * R.inverse() * B.transpose();
    Eigen::MatrixXd H_k = Q;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(A.rows(), A.cols());

    for (int k = 0; k < max_iter; ++k) {
        // Symplectic Doubling Steps
        Eigen::MatrixXd W = (I + G_k * H_k).inverse();
        Eigen::MatrixXd V = (I + H_k * G_k).inverse();

        Eigen::MatrixXd A_next = A_k * W * A_k;
        Eigen::MatrixXd G_next = G_k + A_k * W * G_k * A_k.transpose();
        Eigen::MatrixXd H_next = H_k + A_k.transpose() * H_k * V * A_k;

        double diff = (H_next - H_k).norm();
        A_k = A_next; G_k = G_next; H_k = H_next;

        if (diff < tolerance) return H_k;
    }
    std::cerr << "[MPC] Warning: DARE solver reached max iter!" << std::endl;
    return H_k;
}

// =========================================================================
// CLASS: MPC (Base - Configuration & Interface)
// =========================================================================
class MPC {
public:
    struct Config {
        int N = 20;
        double dt = 0.05;
        
        Eigen::VectorXd Q_diag; // State Weights
        Eigen::VectorXd R_diag; // Input Weights
        
        Eigen::VectorXd u_min;
        Eigen::VectorXd u_max;
        
        Config() {
            Q_diag = Eigen::VectorXd::Ones(12);
            R_diag = Eigen::VectorXd::Ones(4);
            u_min = Eigen::VectorXd::Zero(4);
            u_max = Eigen::VectorXd::Constant(4, 10.0);
        }
    };

protected:
    Config config_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd P_; // Terminal Cost

public:
    MPC(const Config& cfg) : config_(cfg) {
        Q_ = cfg.Q_diag.asDiagonal();
        R_ = cfg.R_diag.asDiagonal();
    }
    
    virtual ~MPC() = default;

    // --- Interface ---

    // Forces the derived class to re-linearize and re-build matrices
    virtual void reset() = 0;

    // Updates weights and triggers a reset automatically
    void updateWeights(const Eigen::VectorXd& q_diag, const Eigen::VectorXd& r_diag) {
        config_.Q_diag = q_diag;
        config_.R_diag = r_diag;
        Q_ = q_diag.asDiagonal();
        R_ = r_diag.asDiagonal();
        
        // Notify derived class to rebuild P and QP matrices
        reset(); 
    }
};

// =========================================================================
// CLASS: LMPC (Linear MPC with qpOASES)
// =========================================================================
class LMPC : public MPC {
private:
    // Reference to external model (Ownership lies outside)
    LinearQuadrotorModel& model_;
    
    // QP Dimensions
    int nx_ = REDUCED_STATE_DIM; // 12
    int nu_ = INPUT_DIM;         // 4
    int n_vars_;
    int n_cons_;

    // Solver
    std::unique_ptr<qpOASES::QProblem> solver_;
    qpOASES::Options options_;
    bool is_initialized_ = false;

    // Flattened Data Storage for qpOASES
    std::vector<qpOASES::real_t> H_data_;
    std::vector<qpOASES::real_t> g_data_;
    std::vector<qpOASES::real_t> A_data_;
    std::vector<qpOASES::real_t> lb_data_;
    std::vector<qpOASES::real_t> ub_data_;
    std::vector<qpOASES::real_t> lbA_data_;
    std::vector<qpOASES::real_t> ubA_data_;

public:
    // Constructor takes reference to model
    LMPC(const MPC::Config& cfg, LinearQuadrotorModel& model_ref) 
        : MPC(cfg), model_(model_ref) 
    {
        n_vars_ = config_.N * (nu_ + nx_); 
        n_cons_ = config_.N * nx_; 

        setupSolver();
        
        // Perform first build
        reset();
    }

    // ---------------------------------------------------------------------
    // RESET (Re-Linearize & Re-Build)
    // ---------------------------------------------------------------------
    void reset() override {
        // 1. Re-Linearize the External Model
        //    (It uses whatever mass/inertia is currently set on it)
        auto [x_hover, u_hover] = model_.findHoverConditions();
        model_.linearize(x_hover, u_hover, config_.dt);
        
        // 2. Re-Compute Terminal Cost P (DARE)
        //    Using the NEW A and B matrices from the model
        P_ = solveDARE(model_.getA(), model_.getB(), Q_, R_);
        
        // 3. Re-Construct QP Matrices
        constructQPMatrices();
        
        // 4. Reset qpOASES Solver
        initQProblem();
    }

    // ---------------------------------------------------------------------
    // SOLVE (Hotstart)
    // ---------------------------------------------------------------------
    bool solve(const Eigen::VectorXd& x0_error, Eigen::VectorXd& u_opt, Eigen::VectorXd& x_pred_first) {
        if (!is_initialized_) return false;

        // Update Initial Condition Constraint: -x1 + B*u0 = -A*x0
        Eigen::VectorXd b_eq_first = -model_.getA() * x0_error;

        // Update the bounds for the first equality constraints
        for(int i=0; i<nx_; ++i) {
            lbA_data_[i] = b_eq_first(i);
            ubA_data_[i] = b_eq_first(i);
        }

        int nWSR = 1000;
        qpOASES::returnValue status = solver_->hotstart(
            g_data_.data(),
            lb_data_.data(), ub_data_.data(),
            lbA_data_.data(), ubA_data_.data(),
            nWSR
        );

        if (status != qpOASES::SUCCESSFUL_RETURN) {
            // Optional: Try full init if hotstart fails
            solver_->init(H_data_.data(), g_data_.data(), A_data_.data(),
                          lb_data_.data(), ub_data_.data(), lbA_data_.data(), ubA_data_.data(), nWSR);
             if (status != qpOASES::SUCCESSFUL_RETURN) return false;
        }

        // Extract Control
        std::vector<qpOASES::real_t> primal_sol(n_vars_);
        solver_->getPrimalSolution(primal_sol.data());

        u_opt = Eigen::VectorXd::Zero(nu_);
        for(int i=0; i<nu_; ++i) u_opt(i) = primal_sol[i];

        x_pred_first = Eigen::VectorXd::Zero(nx_);
        for(int i=0; i<nx_; ++i) x_pred_first(i) = primal_sol[nu_ + i];

        return true;
    }

private:
    void setupSolver() {
        options_.setToMPC(); 
        options_.printLevel = qpOASES::PL_NONE;
        solver_ = std::make_unique<qpOASES::QProblem>(n_vars_, n_cons_);
        solver_->setOptions(options_);
    }

    void constructQPMatrices() {
        // --- Hessian H ---
        H_data_.assign(n_vars_ * n_vars_, 0.0);
        auto setH = [&](int r, int c, const Eigen::MatrixXd& M) {
            for(int i=0; i<M.rows(); ++i) 
                for(int j=0; j<M.cols(); ++j) 
                    H_data_[(r+i)*n_vars_ + (c+j)] = M(i,j);
        };

        int off = 0;
        for(int k=0; k < config_.N; ++k) {
            setH(off, off, R_); off += nu_;
            if (k < config_.N - 1) setH(off, off, Q_); 
            else setH(off, off, P_); // Terminal Cost
            off += nx_;
        }

        // --- Constraint A ---
        A_data_.assign(n_cons_ * n_vars_, 0.0);
        Eigen::MatrixXd NegI = -Eigen::MatrixXd::Identity(nx_, nx_);
        const Eigen::MatrixXd& A_sys = model_.getA();
        const Eigen::MatrixXd& B_sys = model_.getB();

        auto setA = [&](int r_cons, int c_var, const Eigen::MatrixXd& M) {
            for(int i=0; i<M.rows(); ++i)
                for(int j=0; j<M.cols(); ++j)
                    A_data_[(r_cons*nx_ + i)*n_vars_ + (c_var + j)] = M(i,j);
        };

        int col = 0;
        for(int k=0; k < config_.N; ++k) {
            if (k > 0) setA(k, col - nx_, A_sys); // A*x_k
            setA(k, col, B_sys);                   // B*u_k
            col += nu_;
            setA(k, col, NegI);                    // -I*x_{k+1}
            col += nx_;
        }

        // --- Bounds ---
        auto [x_hov, u_hov] = model_.findHoverConditions();
        Eigen::VectorXd u_lb = config_.u_min - u_hov;
        Eigen::VectorXd u_ub = config_.u_max - u_hov;

        lb_data_.assign(n_vars_, -qpOASES::INFTY);
        ub_data_.assign(n_vars_, qpOASES::INFTY);

        int b_idx = 0;
        for(int k=0; k < config_.N; ++k) {
            for(int i=0; i<nu_; ++i) {
                lb_data_[b_idx + i] = u_lb(i);
                ub_data_[b_idx + i] = u_ub(i);
            }
            b_idx += nu_ + nx_;
        }

        // Equality Constraints (Initialized to 0)
        lbA_data_.assign(n_cons_, 0.0);
        ubA_data_.assign(n_cons_, 0.0);
        g_data_.assign(n_vars_, 0.0);
    }

    void initQProblem() {
        int nWSR = 1000;
        qpOASES::returnValue status = solver_->init(
            H_data_.data(), g_data_.data(), A_data_.data(),
            lb_data_.data(), ub_data_.data(), lbA_data_.data(), ubA_data_.data(), nWSR
        );
        is_initialized_ = (status == qpOASES::SUCCESSFUL_RETURN);
        if(!is_initialized_) std::cerr << "[LMPC] Solver Init Failed!" << std::endl;
    }
};

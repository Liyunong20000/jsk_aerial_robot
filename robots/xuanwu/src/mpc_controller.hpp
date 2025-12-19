#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <qpOASES.hpp>

#include "quadrotor_model.hpp"
#include "debug.hpp"

#include "JuliaSparse2Eigen.hpp"
#include "HardcodedMatrices.hpp"

// =========================================================================
// HELPER: Robust DARE Solver
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
// CLASS: MPC (Base)
// =========================================================================
class MPC {
public:
    struct Config {
        int N = 20;
        double dt = 0.05;
        
        Eigen::VectorXd Q_diag; 
        Eigen::VectorXd R_diag; 
        
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
    Eigen::MatrixXd P_; 

public:
    MPC(const Config& cfg) : config_(cfg) {
        Q_ = cfg.Q_diag.asDiagonal();
        R_ = cfg.R_diag.asDiagonal();
    }
    virtual ~MPC() = default;
    virtual void reset() = 0;

    void updateWeights(const Eigen::VectorXd& q_diag, const Eigen::VectorXd& r_diag) {
        config_.Q_diag = q_diag;
        config_.R_diag = r_diag;
        Q_ = q_diag.asDiagonal();
        R_ = r_diag.asDiagonal();
        reset(); 
    }
};

// =========================================================================
// CLASS: LMPC (Linear MPC with qpOASES)
// =========================================================================
class LMPC : public MPC {
private:
    LinearQuadrotorModel& model_;
    
    // QP Dimensions
    int nx_ = REDUCED_STATE_DIM; // 12
    int nu_ = INPUT_DIM;         // 4
    int n_vars_; // N * (nu + nx)
    int n_cons_; // N * nx (Dynamics) + N * nu (Input Limits)

    // Solver
    std::unique_ptr<qpOASES::QProblem> solver_;
    qpOASES::Options options_;
    bool is_initialized_ = false;

    // Flattened Data Storage for qpOASES
    // Note: lb_data and ub_data removed as requested
    std::vector<qpOASES::real_t> H_data_;
    std::vector<qpOASES::real_t> g_data_;
    std::vector<qpOASES::real_t> A_data_; 
    std::vector<qpOASES::real_t> lbA_data_;
    std::vector<qpOASES::real_t> ubA_data_;

public:
    LMPC(const MPC::Config& cfg, LinearQuadrotorModel& model_ref) 
        : MPC(cfg), model_(model_ref) 
    {
        n_vars_ = config_.N * (nu_ + nx_); 
        
        // JULIA LOGIC: Constraints include Dynamics AND Input Bounds
        // Rows 0 to N*nx-1       -> Dynamics equality
        // Rows N*nx to N*nx+N*nu -> Input box inequality
        n_cons_ = (config_.N * nx_) + (config_.N * nu_); 

        setupSolver();
        reset();
    }

    void reset() override {
        // 1. Re-Linearize
        auto [x_hover, u_hover] = model_.findHoverConditions();
        model_.linearize(x_hover, u_hover, config_.dt);
        
        // 2. Re-Compute Terminal Cost P
        P_ = getHardcodedP(); //solveDARE(model_.getA(), model_.getB(), Q_, R_);
        
        // 3. Re-Construct QP Matrices
        constructQPMatrices();
        
        // 4. Reset qpOASES Solver
        initQProblem();
    }

    bool solve(const Eigen::VectorXd& x0_error, Eigen::VectorXd& u_opt, Eigen::VectorXd& x_pred_first) {
        if (!is_initialized_) return false;

        // Update Initial Condition Constraint (First block of Dynamics)
        // -x1 + B*u0 = -A*x0  =>  lbA = -A*x0, ubA = -A*x0
        Eigen::VectorXd b_eq_first = -model_.getA() * x0_error;

        // The dynamics constraints are the first N*nx rows of A
        for(int i=0; i<nx_; ++i) {
            lbA_data_[i] = b_eq_first(i);
            ubA_data_[i] = b_eq_first(i);
        }

        int nWSR = 1000;
        
        // NOTE: lb and ub passed as nullptr (variables are unbounded, restricted only by A)
        qpOASES::returnValue status = solver_->hotstart(
            g_data_.data(),
            nullptr, nullptr, // No Simple Bounds
            lbA_data_.data(), ubA_data_.data(),
            nWSR
        );

        if (status != qpOASES::SUCCESSFUL_RETURN) {
            solver_->init(H_data_.data(), g_data_.data(), A_data_.data(),
                          nullptr, nullptr, // No Simple Bounds
                          lbA_data_.data(), ubA_data_.data(), nWSR);
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

    // TEMPORARY DEBUG FUNCTION
    Eigen::MatrixXd getHardcodedP() {
        Eigen::MatrixXd P_hard(12, 12);
        
        // Copy-paste of your Julia matrix P
        P_hard << 
        2993.191746067216, 7.643122562915009e-8, -5.913007989695964e-9, -9.28616218499025e-8, 4560.715476039981, 1.7738847143353973e-9, 1164.9694135340114, 3.2624759660209675e-8, -2.2306160053921872e-10, -1.0307437375502265e-9, 52.492546080989555, 4.7702239460995246e-11,
        7.643122562915009e-8, 2993.2006447390672, -2.156240812151433e-9, -4560.816287925652, 4.824355385698593e-7, -1.2294191860585146e-10, 4.8176201353623836e-8, 1164.9825087961212, -1.1004340330481478e-10, -52.50201378684936, 5.849585656862987e-9, -1.495959238111324e-11,
        -5.913007989695964e-9, -2.156240812151433e-9, 2127.297753316401, -2.8070399619757805e-8, -2.2607528595047863e-8, -8.884065832583255e-11, -4.262509382453313e-9, 7.00299929384762e-10, 78.16648898343139, -3.4873201843718864e-10, -2.6610319080860605e-10, -4.068769740109136e-12,
        -9.28616218499025e-8, -4560.816287925652, -2.8070399619757805e-8, 49041.226650059216, -1.3305497747113748e-6, 2.430364671451857e-12, -9.16450059371545e-8, -6593.757826069858, -1.4333986087141666e-9, 586.3338052313793, -1.6296508562031158e-8, 1.2154190046189835e-11,
        4560.715476039981, 4.824355385698593e-7, -2.2607528595047863e-8, -1.3305497747113748e-6, 49040.045653350375, -4.2916230097488204e-8, 6593.608168023469, 2.6003934621422956e-7, -9.852020294538002e-10, -1.5626931903241342e-8, 586.2191661897657, -4.565514271806237e-10,
        1.7738847143353973e-9, -1.2294191860585146e-10, -8.884065832583255e-11, 2.430364671451857e-12, -4.2916230097488204e-8, 4116.620471289904, -5.753817274645028e-10, 1.6481737087294554e-10, -1.2583227961082652e-11, 2.965284956473765e-11, -5.351739702757059e-10, 66.8627571880012,
        1164.9694135340114, 4.8176201353623836e-8, -4.262509382453313e-9, -9.16450059371545e-8, 6593.608168023469, -5.753817274645028e-10, 1531.9118123642243, 2.4400909390832753e-8, -1.7421219527327074e-10, -1.0475605013444073e-9, 76.77273964956036, 1.9430960030121907e-11,
        3.2624759660209675e-8, 1164.9825087961212, 7.00299929384762e-10, -6593.757826069858, 2.6003934621422956e-7, 1.6481737087294554e-10, 2.4400909390832753e-8, 1531.931128469369, 5.956398598080878e-11, -76.78689538464639, 3.1589876936730615e-9, -4.129294821136545e-12,
        -2.2306160053921872e-10, -1.1004340330481478e-10, 78.16648898343139, -1.4333986087141666e-9, -9.852020294538002e-10, -1.2583227961082652e-11, -1.7421219527327074e-10, 5.956398598080878e-11, 131.18753597498, -1.7710625145959045e-11, -1.1645820467690336e-11, -4.1960891303814374e-13,
        -1.0307437375502265e-9, -52.50201378684936, -3.4873201843718864e-10, 586.3338052313793, -1.5626931903241342e-8, 2.965284956473765e-11, -1.0475605013444073e-9, -76.78689538464639, -1.7710625145959045e-11, 107.15445030371637, -1.9151329713255978e-10, 6.177951647737246e-13,
        52.492546080989555, 5.849585656862987e-9, -2.6610319080860605e-10, -1.6296508562031158e-8, 586.2191661897657, -5.351739702757059e-10, 76.77273964956036, 3.1589876936730615e-9, -1.1645820467690336e-11, -1.9151329713255978e-10, 107.14242265853194, -5.789787660619679e-12,
        4.7702239460995246e-11, -1.495959238111324e-11, -4.068769740109136e-12, 1.2154190046189835e-11, -4.565514271806237e-10, 66.8627571880012, 1.9430960030121907e-11, -4.129294821136545e-12, -4.1960891303814374e-13, 6.177951647737246e-13, -5.789787660619679e-12, 117.97636428685736;
        
        return P_hard;
    }

    void eigenToFlat(const Eigen::MatrixXd& mat, std::vector<qpOASES::real_t>& flat_data) {
    // qpOASES expects Row-Major by default if we fill it linearly
    flat_data.resize(mat.rows() * mat.cols());
    for(int i=0; i<mat.rows(); ++i) {
        for(int j=0; j<mat.cols(); ++j) {
            flat_data[i * mat.cols() + j] = mat(i, j);
        }
    }
}
    void constructQPMatrices() {
        //// --- 1. Hessian H (Same as before) ---
        //H_data_.assign(n_vars_ * n_vars_, 0.0);
        //auto setH = [&](int r, int c, const Eigen::MatrixXd& M) {
        //    for(int i=0; i<M.rows(); ++i) 
        //        for(int j=0; j<M.cols(); ++j) 
        //            H_data_[(r+i)*n_vars_ + (c+j)] = M(i,j);
        //};

        //int off = 0;
        //for(int k=0; k < config_.N; ++k) {
        //    setH(off, off, R_); off += nu_;
        //    if (k < config_.N - 1) setH(off, off, Q_); 
        //    else setH(off, off, P_); 
        //    off += nx_;
        //}
        
        Eigen::MatrixXd H_eigen = JuliaSparse2Eigen::convert(B_rows, B_cols, B_vals, 320, 320);
        eigenToFlat(H_eigen, H_data_);

        //// --- 2. Constraint Matrix A (Augmented) ---
        //// Structure:
        //// [ Dynamics Constraints (N * nx rows)    ]
        //// [ Input Limit Constraints (N * nu rows) ]
        //
        //A_data_.assign(n_cons_ * n_vars_, 0.0);
        //
        //// Helpers
        //Eigen::MatrixXd NegI = -Eigen::MatrixXd::Identity(nx_, nx_);
        //Eigen::MatrixXd EyeNu = Eigen::MatrixXd::Identity(nu_, nu_);
        //const Eigen::MatrixXd& A_sys = model_.getA();
        //const Eigen::MatrixXd& B_sys = model_.getB();

        //auto setA = [&](int r_cons, int c_var, const Eigen::MatrixXd& M) {
        //    for(int i=0; i<M.rows(); ++i)
        //        for(int j=0; j<M.cols(); ++j)
        //            A_data_[(r_cons)*n_vars_ + (c_var + j)] = M(i,j);
        //};

        //// --- 2a. Fill Dynamics Part (Rows 0 to N*nx - 1) ---
        //int row_dyn = 0;
        //int col = 0;
        //for(int k=0; k < config_.N; ++k) {
        //    if (k > 0) setA(row_dyn, col - nx_, A_sys); // A*x_k
        //    setA(row_dyn, col, B_sys);                  // B*u_k
        //    col += nu_;
        //    setA(row_dyn, col, NegI);                   // -I*x_{k+1}
        //    col += nx_;
        //    row_dyn += nx_; // Move to next dynamic constraint block
        //}

        //// --- 2b. Fill Input Limits Part (Rows N*nx to End) ---
        //int row_lim = config_.N * nx_; // Start after dynamics
        //col = 0;
        //for(int k=0; k < config_.N; ++k) {
        //    // Place Identity matrix at the column corresponding to u_k
        //    setA(row_lim, col, EyeNu);
        //    
        //    // Advance indices
        //    col += nu_ + nx_; // Jump over u_k and x_{k+1} to next u_{k+1}
        //    row_lim += nu_;   // Next block of input limits
        //}

        // 2. Construct A (Constraints) from Hardcoded D
        Eigen::MatrixXd A_eigen = JuliaSparse2Eigen::convert(D_rows, D_cols, D_vals, 320, 320);
        eigenToFlat(A_eigen, A_data_);

        // --- 3. Constraint Bounds (lbA, ubA) ---
        lbA_data_.assign(n_cons_, 0.0);
        ubA_data_.assign(n_cons_, 0.0);

        // 3a. Dynamics Bounds (Equality = 0)
        // (Already 0.0 from assign, but explicit for clarity)
        for(int i=0; i < config_.N * nx_; ++i) {
            lbA_data_[i] = 0.0;
            ubA_data_[i] = 0.0;
        }

        // 3b. Input Limit Bounds
        auto [x_hov, u_hov] = model_.findHoverConditions();
        Eigen::VectorXd u_lb = config_.u_min - u_hov;
        Eigen::VectorXd u_ub = config_.u_max - u_hov;

        int limit_idx = config_.N * nx_; // Start index for limits
        for(int k=0; k < config_.N; ++k) {
            for(int i=0; i < nu_; ++i) {
                lbA_data_[limit_idx + i] = u_lb(i);
                ubA_data_[limit_idx + i] = u_ub(i);
            }
            limit_idx += nu_;
        }

        // --- 4. Gradient g (Zero) ---
        g_data_.assign(n_vars_, 0.0);
    }

    void initQProblem() {
        int nWSR = 1000;
        
        // --- DEBUG DUMP ---
        std::string prefix = "/tmp/mpc_dump_";
        std::cout << "[LMPC] Dumping QP matrices..." << std::endl;
        dumpRawData(prefix + "H.txt", H_data_, n_vars_, n_vars_);
        dumpRawData(prefix + "A.txt", A_data_, n_cons_, n_vars_);
        dumpRawData(prefix + "g.txt",  g_data_,  n_vars_, 1);
        dumpRawData(prefix + "lbA.txt", lbA_data_, n_cons_, 1);
        dumpRawData(prefix + "ubA.txt", ubA_data_, n_cons_, 1);
        dumpEigenData(prefix + "A_model.txt", model_.getA());
        dumpEigenData(prefix + "B_model.txt", model_.getB());
        dumpEigenData(prefix + "P_matrix.txt", P_);
        // --- END DEBUG DUMP ---
        
        // Pass nullptr for simple bounds (lb, ub)
        qpOASES::returnValue status = solver_->init(
            H_data_.data(), g_data_.data(), A_data_.data(),
            nullptr, nullptr, // <--- NO VARIABLE BOUNDS
            lbA_data_.data(), ubA_data_.data(), nWSR
        );
        
        is_initialized_ = (status == qpOASES::SUCCESSFUL_RETURN);
        
        if(!is_initialized_) {
            std::cerr << "[LMPC] Solver Init Failed! Code: " << status << std::endl;
            solver_->printOptions(); 
        }
    }
};

#include "mpc_controller.hpp"

// =========================================================================
// HELPER: Robust Iterative DARE Solver (Internal)
// =========================================================================
namespace {
    Eigen::MatrixXd solveDARE( const Eigen::MatrixXd& A, 
                               const Eigen::MatrixXd& B, 
                               const Eigen::MatrixXd& Q, 
                               const Eigen::MatrixXd& R, 
                               double tolerance = 1e-9, 
                               int max_iter = 5000) 
    {
        Eigen::MatrixXd P = Q; 
        Eigen::MatrixXd P_next = P;
        
        Eigen::MatrixXd At = A.transpose();
        Eigen::MatrixXd Bt = B.transpose();
        
        for (int k = 0; k < max_iter; ++k) {
            Eigen::MatrixXd R_total = R + Bt * P * B;
            Eigen::MatrixXd BPA = Bt * P * A;
            Eigen::MatrixXd K = R_total.ldlt().solve(BPA);

            Eigen::MatrixXd APA = At * P * A;
            P_next = APA - At * P * B * K + Q;
            P_next = 0.5 * (P_next + P_next.transpose());

            double diff = (P_next - P).lpNorm<Eigen::Infinity>();
            P = P_next;
            
            if (diff < tolerance) {
                return P;
            }
        }
        
        std::cerr << "[MPC] Warning: DARE Iterative solver did not fully converge." << std::endl;
        return P;
    }
}

// =========================================================================
// MPC BASE CLASS
// =========================================================================

MPC::Config::Config() {
    N = 20;
    dt = 0.05;
    Q_diag = Eigen::VectorXd::Ones(AUG_STATE_DIM);
    R_diag = Eigen::VectorXd::Ones(4);
    u_min = Eigen::VectorXd::Zero(4);
    u_max = Eigen::VectorXd::Constant(4, 10.0);
}

MPC::MPC(const Config& cfg) : config_(cfg) {
    Q_ = cfg.Q_diag.asDiagonal();
    R_ = cfg.R_diag.asDiagonal();
}

void MPC::updateWeights(const Eigen::VectorXd& q_diag, const Eigen::VectorXd& r_diag) {
    config_.Q_diag = q_diag;
    config_.R_diag = r_diag;
    Q_ = q_diag.asDiagonal();
    R_ = r_diag.asDiagonal();
    reset(); 
}

// =========================================================================
// LMPC IMPLEMENTATION
// =========================================================================

LMPC::LMPC(const MPC::Config& cfg, LinearQuadrotorModel& model_ref) 
    : MPC(cfg), model_(model_ref) 
{
    nx_ = AUG_STATE_DIM; // 16
    nu_ = INPUT_DIM;         // 4
    
    n_vars_ = config_.N * (nu_ + nx_); 
    n_cons_ = (config_.N * nx_) + (config_.N * nu_) + config_.N; 

    setupSolver();
    reset();
}

void LMPC::setupSolver() {
    options_.setToMPC(); 
    options_.printLevel = qpOASES::PL_NONE;
    solver_ = std::make_unique<qpOASES::QProblem>(n_vars_, n_cons_);
    solver_->setOptions(options_);
}

void LMPC::reset() {
    // 1. Re-Linearize
    auto [x_hover, u_hover] = model_.findHoverConditions();
    model_.linearize(x_hover, u_hover, config_.dt);
    
    // 2. Re-Compute Terminal Cost P
    P_ = solveDARE(model_.getA_aug(), model_.getB_aug(), Q_, R_);
    P_.setZero();
    
    // 3. Re-Construct QP Matrices
    constructQPMatrices();
    
    // 4. Reset qpOASES Solver
    initQProblem();
}

void LMPC::constructQPMatrices() {
    // --- 1. Hessian H ---
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
        else setH(off, off, P_); 
        off += nx_;
    }

    // --- 2. Constraint Matrix A ---
    A_data_.assign(n_cons_ * n_vars_, 0.0);
    
    Eigen::MatrixXd NegI = -Eigen::MatrixXd::Identity(nx_, nx_);
    Eigen::MatrixXd EyeNu = Eigen::MatrixXd::Identity(nu_, nu_);
    const Eigen::MatrixXd& A_sys = model_.getA_aug();
    const Eigen::MatrixXd& B_sys = model_.getB_aug();

    auto setA = [&](int r_cons, int c_var, const Eigen::MatrixXd& M) {
        for(int i=0; i<M.rows(); ++i)
            for(int j=0; j<M.cols(); ++j)
                A_data_[(r_cons + i)*n_vars_ + (c_var + j)] = M(i,j);
    };

    // Dynamics Part
    int row_dyn = 0;
    int col = 0;
    for(int k=0; k < config_.N; ++k) {
        if (k > 0) setA(row_dyn, col - nx_, A_sys); 
        setA(row_dyn, col, B_sys);                  
        col += nu_;
        setA(row_dyn, col, NegI);                   
        col += nx_;
        row_dyn += nx_; 
    }

    // Input Limits Part
    int row_lim = config_.N * nx_; 
    col = 16;
    for(int k=0; k < config_.N; ++k) {
        setA(row_lim, col, EyeNu);
        col += nu_ + nx_; 
        row_lim += nu_;   
    }

    // --- 3. Constraint Bounds ---
    lbA_data_.assign(n_cons_, 0.0);
    ubA_data_.assign(n_cons_, 0.0);

    // Dynamics Bounds are 0.0

    // Input Limit Bounds
    auto [x_hov, u_hov] = model_.findHoverConditions();
    Eigen::VectorXd u_lb = config_.u_min - u_hov;
    Eigen::VectorXd u_ub = config_.u_max - u_hov;

    int limit_idx = config_.N * nx_; 
    for(int k=0; k < config_.N; ++k) {
        for(int i=0; i < nu_; ++i) {
            lbA_data_[limit_idx + i] = u_lb(i);
            ubA_data_[limit_idx + i] = u_ub(i);
        }
        limit_idx += nu_;
    }

    // Ground Constraint
    int row_gnd = (config_.N * nx_) + (config_.N * nu_); 
    col = 0;
    for(int k=0; k < config_.N; ++k) {
        int z_global_idx = col + nu_ + 2;
        A_data_[row_gnd * n_vars_ + z_global_idx] = 1.0;
        lbA_data_[row_gnd] = -qpOASES::INFTY;
        ubA_data_[row_gnd] = 0.10; 
        col += nu_ + nx_; 
        row_gnd++;        
    }

    // --- 4. Gradient g ---
    g_data_.assign(n_vars_, 0.0);
}

void LMPC::initQProblem() {
    int nWSR = 1000;
    
    qpOASES::returnValue status = solver_->init(
        H_data_.data(), g_data_.data(), A_data_.data(),
        nullptr, nullptr, 
        lbA_data_.data(), ubA_data_.data(), nWSR
    );
    
    is_initialized_ = (status == qpOASES::SUCCESSFUL_RETURN);
    
    if(!is_initialized_) {
        std::cerr << "[LMPC] Solver Init Failed! Code: " << status << std::endl;
    }
}

bool LMPC::solve(const Eigen::VectorXd& x0_error, Eigen::VectorXd& u_opt, std::vector<Eigen::VectorXd>& horizon_states) {
    if (!is_initialized_) return false;

    // Update Initial Condition Constraint
    Eigen::VectorXd b_eq_first = -model_.getA_aug() * x0_error;
    for(int i=0; i<nx_; ++i) {
        lbA_data_[i] = b_eq_first(i);
        ubA_data_[i] = b_eq_first(i);
    } 

    int nWSR = 1000;
    
    qpOASES::returnValue status = solver_->hotstart(
        g_data_.data(),
        nullptr, nullptr, 
        lbA_data_.data(), ubA_data_.data(),
        nWSR
    );

    if (status != qpOASES::SUCCESSFUL_RETURN) {
        status = solver_->init(H_data_.data(), g_data_.data(), A_data_.data(),
                      nullptr, nullptr, 
                      lbA_data_.data(), ubA_data_.data(), nWSR);
         if (status != qpOASES::SUCCESSFUL_RETURN) return false;
    }

    // Extract Control
    std::vector<qpOASES::real_t> primal_sol(n_vars_);
    solver_->getPrimalSolution(primal_sol.data());

    u_opt = Eigen::VectorXd::Zero(nu_);
    for(int i=0; i<nu_; ++i) u_opt(i) = primal_sol[i];
    
    // Extract Horizon
    horizon_states.clear();
    horizon_states.reserve(config_.N);

    int offset = 0;
    for (int k = 0; k < config_.N; ++k) {
        offset += nu_; 
        Eigen::VectorXd x_k(nx_);
        for (int i = 0; i < nx_; ++i) x_k(i) = primal_sol[offset + i];
        horizon_states.push_back(x_k);
        offset += nx_; 
    }
    return true;
}

void LMPC::updateConfig(int N, double dt, const Eigen::VectorXd& q_diag, const Eigen::VectorXd& r_diag) {
    bool need_solver_reset = (N != config_.N);

    // 1. Update Config
    config_.N = N;
    config_.dt = dt;
    config_.Q_diag = q_diag;
    config_.R_diag = r_diag;

    // Update Matrices
    Q_ = q_diag.asDiagonal();
    R_ = r_diag.asDiagonal();

    // 2. If Horizon changed, we must resize everything
    if (need_solver_reset) {
        // Re-calculate dimensions
        n_vars_ = config_.N * (nu_ + nx_);
        n_cons_ = (config_.N * nx_) + (config_.N * nu_) + config_.N;

        // Re-allocate qpOASES solver
        setupSolver(); // This creates a new QProblem with new sizes
    }

    // 3. Reset internal state (Linearization + Cost)
    reset();
}

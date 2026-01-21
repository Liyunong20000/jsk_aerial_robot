#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <qpOASES.hpp>

#include "quadrotor_model.hpp"
#include "debug.hpp" // Assuming this exists in your project

// =========================================================================
// CLASS: MPC (Base)
// =========================================================================
class MPC {
public:
    struct Config {
        int N;
        double dt;
        
        Eigen::VectorXd Q_diag; 
        Eigen::VectorXd R_diag; 
        
        // Absolute Motor Limits
        Eigen::VectorXd u_min;
        Eigen::VectorXd u_max;

        Config(); // Implementation moved to cpp
    };

protected:
    Config config_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd P_; 

public:
    MPC(const Config& cfg);
    virtual ~MPC() = default;
    virtual void reset() = 0;
    virtual void updateConfig(int N, double dt, const Eigen::VectorXd& q_diag, const Eigen::VectorXd& r_diag) = 0;

    void updateWeights(const Eigen::VectorXd& q_diag, const Eigen::VectorXd& r_diag);
};

// =========================================================================
// CLASS: LMPC (Linear MPC with qpOASES)
// =========================================================================
class LMPC : public MPC {
private:
    LinearQuadrotorModel& model_;
    
    // QP Dimensions
    int nx_; // Will be 16 (12 State + 4 Integrator)
    int nu_; // Will be  4 (Delta U)
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
    std::vector<qpOASES::real_t> lbA_data_;
    std::vector<qpOASES::real_t> ubA_data_;

    void setupSolver();
    void constructQPMatrices();
    void initQProblem();

public:
    LMPC(const MPC::Config& cfg, LinearQuadrotorModel& model_ref);

    void updateConfig(int N, double dt, const Eigen::VectorXd& q_diag, const Eigen::VectorXd& r_diag) override;
    
    void reset() override;

    // x0_error is now 16-dim
    bool solve(const Eigen::VectorXd& x0_error, Eigen::VectorXd& u_opt, std::vector<Eigen::VectorXd>& horizon_states);
};

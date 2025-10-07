#ifndef MPCCONTROLLER_HPP
#define MPCCONTROLLER_HPP

#include "MpcState.hpp"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>  

class MpcController {
public:
    MpcController();
    
    // Khởi tạo MPC với weights
    void init(float Q1_weight, float Q2_weight, float R_weight);

    void debugMatrices();
    
    // Tính góc lái từ MpcState
    float computeSteeringAngle(const MpcState& state, float velocity);
    
    // Setters
    void setVehicleParams(float wheelbase, float mass, float Lf, float Lr, 
                         float Caf, float Car, float Iz);
    void setPredictionHorizon(int N);
    
private:
    // Vehicle parameters
    float wheelbase_;
    float mass_;
    float Lf_, Lr_;
    float Caf_, Car_;
    float Iz_;
    
    // MPC parameters
    int N_;                // Prediction horizon
    float Q1_, Q2_, R_;    // Weights
    float Ts_;             // Sample time
    bool initialized_;
    
    // MPC matrices
    Eigen::MatrixXd A_d_, B1_d_, B2_d_;
    Eigen::MatrixXd AX_, BU_, BV_, H_;
    
    // Constraint
    double umin_, umax_;

    std::unique_ptr<OsqpEigen::Solver> solver_;

    bool solver_initialized_;
    
    // Helper functions
    void buildMpcMatrices(float Vx);
    Eigen::MatrixXd matrixPower(const Eigen::MatrixXd& A, int p);
    float solveQP(const Eigen::VectorXd& x0, const Eigen::VectorXd& v_k);
};

#endif
#ifndef MPCCONTROLLER_HPP
#define MPCCONTROLLER_HPP

#include "MpcState.hpp"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

class MpcController {
public:
    MpcController();
    void init(float Q1_weight, float Q2_weight, float R_weight);
    void debugMatrices();
    float computeSteeringAngle(const MpcState& state, float velocity);
    void setVehicleParams(float wheelbase, float mass, float Lf, float Lr,
                         float Caf, float Car, float Iz);
    void setPredictionHorizon(int N);

private:
    float wheelbase_, mass_, Lf_, Lr_, Caf_, Car_, Iz_;
    int N_;
    float Q1_, Q2_, R_, Ts_;
    bool initialized_, solver_initialized_;
    std::unique_ptr<OsqpEigen::Solver> solver_;
    Eigen::MatrixXd A_d_, B1_d_, B2_d_, AX_, BU_, BV_, H_;
    double umin_, umax_;
    void buildMpcMatrices(float Vx);
    Eigen::MatrixXd matrixPower(const Eigen::MatrixXd& A, int p);
    float solveQP(const Eigen::VectorXd& x0, const Eigen::VectorXd& v_k);
};

#endif

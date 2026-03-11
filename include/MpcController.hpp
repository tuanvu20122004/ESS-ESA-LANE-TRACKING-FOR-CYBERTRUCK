#ifndef MPCCONTROLLER_HPP
#define MPCCONTROLLER_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <memory>
#include <vector>

// Struct containing MPC parameters
struct MpcState {
    std::vector<float> curvature;       // Curvature vector (1/m)
    float lateral_deviation;             // Lateral offset (m)
    float yaw_angle;                     // Yaw angle (rad)
    bool is_valid;
    
    MpcState() 
        : curvature(10, 0.0f),
          lateral_deviation(0.0f),
          yaw_angle(0.0f),
          is_valid(false) {}
};

class MpcController {
public:
    MpcController();
    
    void init(float Q1_weight, float Q2_weight, float R_weight);
    void debugMatrices();
    
    // Compute MPC parameters from centerline
    MpcState computeMpcParameters(const std::vector<cv::Point>& centerline,
                                  const cv::Mat& birdEyeView);
    
    // Compute steering angle from MPC state
    float computeSteeringAngle(const MpcState& state, float velocity);
    
    void setVehicleParams(float wheelbase, float mass, float Lf, float Lr, 
                         float Caf, float Car, float Iz);
    void setPredictionHorizon(int N);
    void setVehiclePosition(float x, float y);
    
private:
    // Vehicle parameters
    float wheelbase_;
    float mass_;
    float Lf_, Lr_;
    float Caf_, Car_;
    float Iz_;
    
    // MPC parameters
    int N_;
    float Q1_, Q2_, R_;
    float Ts_;
    bool initialized_;
    
    // MPC matrices
    Eigen::MatrixXd A_d_, B1_d_, B2_d_;
    Eigen::MatrixXd AX_, BU_, BV_, H_;
    
    // Constraint
    double umin_, umax_;
    std::unique_ptr<OsqpEigen::Solver> solver_;
    bool solver_initialized_;
    
    // MPC computation parameters
    float pixel_per_meter_;
    float vehicle_x_;
    float vehicle_y_;
    
    // Helper functions for MPC optimization
    void buildMpcMatrices(float Vx);
    Eigen::MatrixXd matrixPower(const Eigen::MatrixXd& A, int p);
    float solveQP(const Eigen::VectorXd& x0, const Eigen::VectorXd& v_k);
    
    // MPC computation methods (from ComputeMpc)
    cv::Vec3f fitCenterlinePoly(const std::vector<cv::Point>& centerline);
    std::vector<float> computeMultipleCurvatures(const cv::Vec3f& coeffs, int N = 10);
    float computeLateralDeviation(const cv::Vec3f& coeffs, const cv::Mat& birdEyeView);
    float computeYawAngle(const cv::Vec3f& coeffs, float y);
};

#endif

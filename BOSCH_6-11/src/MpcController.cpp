#include "MpcController.hpp"
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <algorithm>
#include <cmath>

const float DISTANCE_TO_AXLE = 0.15f;  // 15cm from camera to axle
const float DEFAULT_BIRD_EYE_WIDTH = 640.0f;   // Default bird's eye view width
const float DEFAULT_BIRD_EYE_HEIGHT = 480.0f;  // Default bird's eye view height

static int prev_z_dim = -1;
static int prev_constraint_dim = -1;

float vehicle_x_;
float vehicle_y_;

MpcController::MpcController()
    : wheelbase_(0.2515f),
    mass_(2.3f),
    Lf_(0.132f),
    Lr_(0.12f),
    Caf_(0.04f),
    Car_(0.02f),
    Iz_(0.04f),
    N_(10),
    Q1_(1000.0f),
    Q2_(50.0f),
    R_(5.0f),
    Ts_(0.071f),
    initialized_(false),
    solver_(nullptr),
    umin_(-28.0 * M_PI / 180.0),
    umax_(28.0 * M_PI / 180.0),
    solver_initialized_(false),
    pixel_per_meter_(0.0002f),
    vehicle_x_(0.0f),
    vehicle_y_(0.0f) {
}

void MpcController::setVehicleParams(float wheelbase, float mass, float Lf, float Lr,
    float Caf, float Car, float Iz) {
    wheelbase_ = wheelbase;
    mass_ = mass;
    Lf_ = Lf;
    Lr_ = Lr;
    Caf_ = Caf;
    Car_ = Car;
    Iz_ = Iz;
}

void MpcController::setPredictionHorizon(int N) {
    N_ = N;
    initialized_ = false;
}

Eigen::MatrixXd MpcController::matrixPower(const Eigen::MatrixXd& A, int p) {
    if (p == 0) return Eigen::MatrixXd::Identity(A.rows(), A.cols());
    if (p == 1) return A;
    Eigen::MatrixXd result = A;
    for (int i = 1; i < p; ++i) {
        result = result * A;
    }
    return result;
}

void MpcController::init(float Q1_weight, float Q2_weight, float R_weight) {
    Q1_ = Q1_weight;
    Q2_ = Q2_weight;
    R_ = R_weight;

    buildMpcMatrices(0.3f);

    if (!solver_) solver_ = std::make_unique<OsqpEigen::Solver>();
    solver_->settings()->setVerbosity(false);
    solver_->settings()->setWarmStart(true);
    solver_->settings()->setMaxIteration(4000);
    solver_->settings()->setAbsoluteTolerance(1e-4);
    solver_->settings()->setRelativeTolerance(1e-4);

    initialized_ = true;
    std::cout << "[MPC] Controller initialized successfully" << std::endl;
}

void MpcController::buildMpcMatrices(float Vx) {
    // Continuous state-space
    Eigen::MatrixXd A_c(4, 4);
    A_c << 0, 1, 0, 0,
           0, -(2*Caf_ + 2*Car_)/(mass_*Vx), (2*Caf_ + 2*Car_)/mass_,
           (-2*Caf_*Lf_ + 2*Car_*Lr_)/(mass_*Vx),
           0, 0, 0, 1,
           0, (-2*Caf_*Lf_ + 2*Car_*Lr_)/(Iz_*Vx),
           (2*Caf_*Lf_ - 2*Car_*Lr_)/Iz_,
           (-2*Caf_*Lf_*Lf_ - 2*Car_*Lr_*Lr_)/(Iz_*Vx);

    Eigen::MatrixXd B_c(4, 2);
    B_c << 0, 0,
           2*Caf_/mass_, (-2*Caf_*Lf_ + 2*Car_*Lr_)/(mass_*Vx) - Vx,
           0, 0,
           2*Caf_*Lf_/Iz_, (-2*Caf_*Lf_*Lf_ - 2*Car_*Lr_*Lr_)/(Iz_*Vx);

    // Discretization
    Eigen::MatrixXd M(6, 6);
    M.setZero();
    M.block(0, 0, 4, 4) = A_c;
    M.block(0, 4, 4, 2) = B_c;
    M = M * Ts_;

    Eigen::MatrixXd Md = M.exp();
    A_d_ = Md.block(0, 0, 4, 4);
    Eigen::MatrixXd B_d = Md.block(0, 4, 4, 2);
    B1_d_ = B_d.col(0);
    B2_d_ = B_d.col(1);

    int n = 4;  // State dimension
    int m = 1;  // Control input dimension

    AX_ = Eigen::MatrixXd::Zero((N_ + 1) * n, n);
    for (int i = 0; i <= N_; ++i) {
        AX_.block(i * n, 0, n, n) = matrixPower(A_d_, i);
    }

    BU_ = Eigen::MatrixXd::Zero((N_ + 1) * n, N_ * m);
    for (int i = 1; i <= N_; ++i) {
        for (int j = 0; j < i; ++j) {
            BU_.block(i * n, j * m, n, m) = matrixPower(A_d_, i - j - 1) * B1_d_;
        }
    }

    BV_ = Eigen::MatrixXd::Zero((N_ + 1) * n, N_ * m);
    for (int i = 1; i <= N_; ++i) {
        for (int j = 0; j < i; ++j) {
            BV_.block(i * n, j * m, n, m) = matrixPower(A_d_, i - j - 1) * B2_d_;
        }
    }

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
    Q(0, 0) = Q1_;
    Q(2, 2) = Q2_;
    Eigen::MatrixXd QN = Q;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1) * R_;

    Eigen::MatrixXd QX = Eigen::MatrixXd::Zero((N_ + 1) * n, (N_ + 1) * n);
    for (int i = 0; i < N_; ++i)
        QX.block(i * n, i * n, n, n) = Q;
    QX.block(N_ * n, N_ * n, n, n) = QN;

    Eigen::MatrixXd RU = Eigen::MatrixXd::Zero(N_ * m, N_ * m);
    for (int i = 0; i < N_; ++i)
        RU.block(i * m, i * m, m, m) = R;

    H_ = Eigen::MatrixXd::Zero((N_ + 1) * n + N_ * m, (N_ + 1) * n + N_ * m);
    H_.block(0, 0, (N_ + 1) * n, (N_ + 1) * n) = QX;
    H_.block((N_ + 1) * n, (N_ + 1) * n, N_ * m, N_ * m) = RU;
    H_ += Eigen::MatrixXd::Identity(H_.rows(), H_.cols()) * 1e-6;
}

void MpcController::debugMatrices() {
    std::cout << "[MPC DEBUG] Matrix sizes:" << std::endl;
    std::cout << "  A_d_: " << A_d_.rows() << "x" << A_d_.cols() << std::endl;
    std::cout << "  B1_d_: " << B1_d_.rows() << "x" << B1_d_.cols() << std::endl;
    std::cout << "  B2_d_: " << B2_d_.rows() << "x" << B2_d_.cols() << std::endl;
    std::cout << "  H_: " << H_.rows() << "x" << H_.cols() << std::endl;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(H_);
    double min_eigenvalue = es.eigenvalues().minCoeff();
    std::cout << "  H_ min eigenvalue: " << min_eigenvalue << std::endl;
}

float MpcController::solveQP(const Eigen::VectorXd& x0, const Eigen::VectorXd& v_k) {
    int nx = 4;
    int nu = 1;
    int z_dim = (N_ + 1) * nx + N_ * nu;
    int constraint_dim = (N_ + 1) * nx;

    if (x0.size() != nx || v_k.size() != N_) {
        std::cerr << "[MPC] Invalid input sizes!" << std::endl;
        return 0.0f;
    }

    Eigen::MatrixXd G = H_;
    Eigen::VectorXd g = Eigen::VectorXd::Zero(z_dim);

    Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero((N_ + 1) * nx, z_dim);
    Aeq.block(0, 0, (N_ + 1) * nx, (N_ + 1) * nx) = 
        Eigen::MatrixXd::Identity((N_ + 1) * nx, (N_ + 1) * nx);
    Aeq.block(0, (N_ + 1) * nx, (N_ + 1) * nx, N_ * nu) = -BU_;
    Eigen::VectorXd beq = AX_ * x0 + BV_ * v_k;

    if (!G.allFinite() || !Aeq.allFinite() || !beq.allFinite()) {
        std::cerr << "[MPC] Matrix contains NaN/Inf!" << std::endl;
        return 0.0f;
    }

    Eigen::SparseMatrix<double> G_sparse = G.sparseView();
    Eigen::SparseMatrix<double> Aeq_sparse = Aeq.sparseView();

    Eigen::VectorXd lb = beq;
    Eigen::VectorXd ub = beq;

    // If solver not initialized or dimensions changed, reinitialize
    if (!solver_initialized_ || prev_z_dim != z_dim || prev_constraint_dim != constraint_dim) {
        solver_.reset(new OsqpEigen::Solver());
        solver_->settings()->setVerbosity(false);
        solver_->settings()->setWarmStart(true);
        solver_->settings()->setMaxIteration(4000);
        solver_->settings()->setAbsoluteTolerance(1e-4);
        solver_->settings()->setRelativeTolerance(1e-4);

        solver_->data()->setNumberOfVariables(z_dim);
        solver_->data()->setNumberOfConstraints(constraint_dim);

        if (!solver_->data()->setHessianMatrix(G_sparse) ||
            !solver_->data()->setGradient(g) ||
            !solver_->data()->setLinearConstraintsMatrix(Aeq_sparse) ||
            !solver_->data()->setLowerBound(lb) ||
            !solver_->data()->setUpperBound(ub)) {
            std::cerr << "[MPC] Failed to set solver data!" << std::endl;
            return 0.0f;
        }

        if (!solver_->initSolver()) {
            std::cerr << "[MPC] Failed to init solver!" << std::endl;
            return 0.0f;
        }

        solver_initialized_ = true;
        prev_z_dim = z_dim;
        prev_constraint_dim = constraint_dim;
    } else {
        // Fast update for next iteration
        if (!solver_->updateHessianMatrix(G_sparse) ||
            !solver_->updateGradient(g) ||
            !solver_->updateBounds(lb, ub)) {
            std::cerr << "[MPC] Failed to update solver!" << std::endl;
            return 0.0f;
        }
    }

    if (solver_->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        std::cerr << "[MPC] Solve failed!" << std::endl;
        return 0.0f;
    }

    Eigen::VectorXd z_opt = solver_->getSolution();
    if (z_opt.size() != z_dim || !z_opt.allFinite()) {
        std::cerr << "[MPC] Invalid solution!" << std::endl;
        return 0.0f;
    }

    double u_cmd = z_opt((N_ + 1) * nx);
    u_cmd = std::clamp(u_cmd, static_cast<double>(umin_), static_cast<double>(umax_));

    //return static_cast<float>(-u_cmd * 180.0 / M_PI);
    return static_cast<float>(u_cmd * 180.0 / M_PI);
}

float MpcController::computeSteeringAngle(const MpcState& state, float velocity) {
    if (!initialized_) {
        std::cerr << "[MPC] Not initialized!" << std::endl;
        return 0.0f;
    }

    static float cached_velocity = -1.0f;
    if (std::abs(velocity - cached_velocity) > 0.01f) {
        buildMpcMatrices(velocity);
        cached_velocity = velocity;
        prev_z_dim = -1;
        prev_constraint_dim = -1;
    }

    if (!state.is_valid) {
        return 0.0f;
    }

    if (state.curvature.size() < static_cast<size_t>(N_)) {
        std::cerr << "[MPC] Curvature vector too small!" << std::endl;
        return 0.0f;
    }

    Eigen::VectorXd x0(4);
    x0(0) = state.lateral_deviation;
    x0(1) = 0.0;
    x0(2) = state.yaw_angle;
    x0(3) = 0.0;

    Eigen::VectorXd v_k(N_);
    for (int i = 0; i < N_; ++i) {
        v_k(i) = state.curvature[i] * velocity;
    }

    return solveQP(x0, v_k);
}

// ==================== MPC COMPUTATION METHODS ====================

cv::Vec3f MpcController::fitCenterlinePoly(const std::vector<cv::Point>& centerline) {
    if (centerline.size() < 3) {
        return cv::Vec3f(0, 0, 0);
    }
    
    std::vector<float> x_vals, y_vals;
    for (const auto& pt : centerline) {
        x_vals.push_back(static_cast<float>(pt.x));
        y_vals.push_back(static_cast<float>(pt.y));
    }
    
    cv::Mat Y(y_vals.size(), 1, CV_32F, y_vals.data());
    cv::Mat X(x_vals.size(), 1, CV_32F, x_vals.data());
    
    cv::Mat A(Y.rows, 3, CV_32F);
    for (int i = 0; i < Y.rows; ++i) {
        float y = Y.at<float>(i, 0);
        A.at<float>(i, 0) = y * y;
        A.at<float>(i, 1) = y;
        A.at<float>(i, 2) = 1.0f;
    }
    
    cv::Mat coeffs;
    bool ok = cv::solve(A, X, coeffs, cv::DECOMP_SVD);
    
    if (ok) {
        return cv::Vec3f(coeffs.at<float>(0), 
                        coeffs.at<float>(1), 
                        coeffs.at<float>(2));
    }
    
    return cv::Vec3f(0, 0, 0);
}

std::vector<float> MpcController::computeMultipleCurvatures(const cv::Vec3f& coeffs, int N) {
    std::vector<float> curvatures;
    float a = coeffs[0];
    float b = coeffs[1];

    if(vehicle_y_ == 0.0f){
        vehicle_y_ = DEFAULT_BIRD_EYE_HEIGHT - 1.0f;
    }
    
    for (int i = 0; i < N; ++i) {
        float y = vehicle_y_ - i * 26.0f;  // 26 pixels ≈ 3cm
        
        float dx_dy = 2.0f * a * y + b;
        float d2x_dy2 = 2.0f * a;
        
        float numerator = std::abs(d2x_dy2);
        float denominator = std::pow(1.0f + dx_dy * dx_dy, 1.5f);
        
        float kappa_pixel = (denominator > 1e-6f) ? (numerator / denominator) : 0.0f;
        float kappa_meter = kappa_pixel / pixel_per_meter_;
        
        curvatures.push_back(kappa_meter);
    }
    
    return curvatures;
}

float MpcController::computeLateralDeviation(const cv::Vec3f& coeffs, 
                                          const cv::Mat& birdEyeView) {

    if (vehicle_x_ == 0.0f && vehicle_y_ == 0.0f) {
        vehicle_x_ = birdEyeView.cols / 2.0f;
        vehicle_y_ = birdEyeView.rows - 1.0f;
    }
    
    float centerline_x = coeffs[0] * vehicle_y_ * vehicle_y_ + 
                         coeffs[1] * vehicle_y_ + 
                         coeffs[2];
    
    float lateral_deviation_pixel = vehicle_x_ - centerline_x;
    float lateral_deviation_meter = lateral_deviation_pixel * pixel_per_meter_;
    
    return lateral_deviation_meter;
}

float MpcController::computeYawAngle(const cv::Vec3f& coeffs, float y) {
    float dx_dy = 2.0f * coeffs[0] * y + coeffs[1];
    float yaw_angle_rad = std::atan(dx_dy);
    
    return yaw_angle_rad;
}

MpcState MpcController::computeMpcParameters(const std::vector<cv::Point>& centerline,
                                          const cv::Mat& birdEyeView) {
    MpcState result;
    
    if (centerline.size() < 3) {
        result.is_valid = false;
        return result;
    }
    
    cv::Vec3f coeffs = fitCenterlinePoly(centerline);
    
    if (vehicle_x_ == 0.0f && vehicle_y_ == 0.0f) {
        vehicle_x_ = birdEyeView.cols / 2.0f;
        vehicle_y_ = birdEyeView.rows - 1.0f;
    }
    
    // 1. Compute curvature vector (10 elements)
    result.curvature = computeMultipleCurvatures(coeffs, 10);
    
    // 2. Compute yaw angle
    result.yaw_angle = computeYawAngle(coeffs, vehicle_y_);
    
    // 3. Compute lateral deviation (with steering angle compensation)
    float raw_deviation = computeLateralDeviation(coeffs, birdEyeView);
    result.lateral_deviation = raw_deviation - 
                               DISTANCE_TO_AXLE * std::sin(result.yaw_angle);
    
    result.is_valid = true;
    return result;
}

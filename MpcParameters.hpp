#ifndef MPCPARAMETERS_HPP
#define MPCPARAMETERS_HPP

#include <mutex>
#include <chrono>

struct MpcParams {
    float curvature;
    float offset;
    float angle_y;
    bool is_valid;
    uint64_t timestamp;
    
    MpcParams() : curvature(0), offset(0), angle_y(0), 
                  is_valid(false), timestamp(0) {}
};

class MpcParametersManager {
public:
    MpcParametersManager() { reset(); }
    
    void updateParameters(float curv, float lat_dev, float yaw) {
        std::lock_guard<std::mutex> lock(mutex_);
        params_.curvature = curv;
        params_.offset = lat_dev;
        params_.angle_y = yaw;
        params_.is_valid = true;
        params_.timestamp = getCurrentTimeMs();
    }
    
    MpcParams getParameters() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return params_;
    }
    
    bool isValid() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return params_.is_valid;
    }
    
    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        params_ = MpcParams();
    }
    
    float getCurvature() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return params_.curvature;
    }
    
    float getLateralDeviation() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return params_.offset;
    }
    
    float getYawAngle() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return params_.angle_y;
    }

private:
    mutable std::mutex mutex_;
    MpcParams params_;
    
    uint64_t getCurrentTimeMs() const {
        using namespace std::chrono;
        return duration_cast<milliseconds>(
            system_clock::now().time_since_epoch()
        ).count();
    }
};

#endif
#ifndef MPCSTATE_HPP
#define MPCSTATE_HPP

#include <vector>
#include <chrono>
#include <mutex>

// Struct chứa 3 tham số MPC
struct MpcState {
    std::vector<float> curvature;  // 10 phần tử (1/m)
    float lateral_deviation;        // offset (m)
    float yaw_angle;               // rad
    bool is_valid;
    uint64_t timestamp;
    
    MpcState() 
        : curvature(10, 0.0f),
          lateral_deviation(0.0f),
          yaw_angle(0.0f),
          is_valid(false),
          timestamp(0) {}
};

// Thread-safe manager
class MpcStateManager {
public:
    MpcStateManager() { reset(); }
    
    void updateState(const MpcState& state) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = state;
        state_.is_valid = true;
        state_.timestamp = getCurrentTimeMs();
    }
    
    MpcState getState() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }
    
    bool isValid() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_.is_valid;
    }
    
    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = MpcState();
    }
    
private:
    mutable std::mutex mutex_;
    MpcState state_;
    
    uint64_t getCurrentTimeMs() const {
        using namespace std::chrono;
        return duration_cast<milliseconds>(
            system_clock::now().time_since_epoch()
        ).count();
    }
};

#endif
#ifndef MPCSTATE_HPP
#define MPCSTATE_HPP

#include <vector>

struct MpcState {
    std::vector<float> curvature;
    float lateral_deviation;
    float yaw_angle;
    bool is_valid;

    MpcState()
        : lateral_deviation(0.0f),
          yaw_angle(0.0f),
          is_valid(false) {}
};

#endif

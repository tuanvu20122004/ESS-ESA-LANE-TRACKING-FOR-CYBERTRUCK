#include "LaneDetector.hpp"

int main() {
    std::string videoPath = "D:/Downloads/demo.mp4";
    LaneDetector detector(videoPath);
    detector.processFrame();
    return 0;
}

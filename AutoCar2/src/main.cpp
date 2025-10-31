#include <iostream>
#include <chrono>
#include "LaneDetector.hpp"
#include "MpcController.hpp"

int main(int argc, char* argv[]) {
    std::cout << "=== Lane Keeping System with MPC Controller ===" << std::endl;
    
    std::string videoPath = (argc > 1) ? argv[1] : "Demo.mp4";
    LaneDetector detector(videoPath, 640, 480);
    
    if (!detector.isOpened()) {
        std::cerr << "[ERROR] Cannot open video: " << videoPath << std::endl;
        return -1;
    }
    std::cout << "[MAIN] Lane Detector initialized." << std::endl;
    
    MpcController mpc;
    mpc.init(1000.0f, 50.0f, 5.0f);
    mpc.debugMatrices();  
    mpc.setVehicleParams(0.2515f, 2.3f, 0.132f, 0.12f, 0.04f, 0.02f, 0.04f);
    std::cout << "[MAIN] MPC Controller initialized." << std::endl;
    
    float desired_velocity = 0.3f;
    int frame_count = 0;
    
    cv::Mat frame;
    while (true) {
        auto start = std::chrono::high_resolution_clock::now();
        
        // Step 1: Get frame and process lane detection
        if (!detector.getFrame(frame)) {
            std::cout << "[MAIN] End of video." << std::endl;
            break;
        }
        
        detector.processFrame(frame);
        
        // Step 2: Check if lane is valid
        if (!detector.hasValidLane()) {
            std::cout << "[MAIN] Frame " << frame_count 
                      << ": No valid lane detected." << std::endl;
            frame_count++;
            
            if (cv::waitKey(30) == 27) break;
            continue;
        }
        
        // Step 3: Compute MPC parameters from centerline
        std::vector<cv::Point> centerline = detector.getCenterline();
        cv::Mat birdEyeView = detector.getBirdEyeView();
        
        MpcState state = mpc.computeMpcParameters(centerline, birdEyeView);
        
        if (!state.is_valid) {
            std::cout << "[MAIN] Frame " << frame_count 
                      << ": MPC computation failed." << std::endl;
            frame_count++;
            
            if (cv::waitKey(30) == 27) break;
            continue;
        }
        
        // Step 4: Compute steering angle from MPC
        float steering_angle = mpc.computeSteeringAngle(state, desired_velocity);
        int servo_angle = 93 + static_cast<int>(steering_angle);
        
        // Step 5: Update display information
        detector.setMpcDisplayData(state.curvature[0], state.lateral_deviation, state.yaw_angle);
        detector.setSteeringInfo(steering_angle, servo_angle);
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        // Print information
        std::cout << "\n========== Frame " << frame_count << " ==========" << std::endl;
        std::cout << "Curvature[0]: " << state.curvature[0] << " (1/m)" << std::endl;
        std::cout << "Lateral Dev:  " << state.lateral_deviation << " (m)" << std::endl;
        std::cout << "Yaw Angle:    " << state.yaw_angle * 180.0f / M_PI << " (deg)" << std::endl;
        std::cout << "Steering Cmd: " << steering_angle << " (deg)" << std::endl;
        std::cout << "Servo Angle:  " << servo_angle << std::endl;
        std::cout << "Process Time: " << duration.count() << " ms" << std::endl;
        
        frame_count++;
        
        if (cv::waitKey(30) == 27) break;
    }
    
    std::cout << "\n[MAIN] Total frames processed: " << frame_count << std::endl;
    std::cout << "[MAIN] System shutdown." << std::endl;
    
    return 0;
}

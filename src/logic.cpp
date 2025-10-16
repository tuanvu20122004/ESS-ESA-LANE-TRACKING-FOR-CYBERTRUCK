#include "logic.hpp"
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>

static std::mutex frame_mutex;
static cv::Mat latest_frame;
static std::atomic<bool> running(true);

Logic::Logic(const std::string& videoPath)
    : detector(videoPath, 640, 480),      
      comm("/dev/ttyACM0", 115200)
{
    mpc.init(1000.0f, 50.0f, 5.0f);
    mpc.setVehicleParams(0.2515f, 2.3f, 0.132f, 0.12f, 0.04f, 0.02f, 0.04f);
    std::cout << "[LOGIC] MPC initialized." << std::endl;
}

void Logic::run() {
    Logger logger("performance_log.txt");

    std::thread camera_thread([&]() {
        cv::Mat frame;
        while (running) {
            cap >> frame;
            if (frame.empty()) continue;

            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                latest_frame = frame.clone();
            }

            // Optional preview
            cv::imshow("Live Feed", frame);
            if (cv::waitKey(1) == 27) running = false; // ESC để thoát
        }
    });

    std::thread mpc_thread([&]() {
        cv::Mat frame;
        while (running) {
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                if (!latest_frame.empty())
                    frame = latest_frame.clone();
            }
            if (frame.empty()) continue;

            detector.processFrame(frame);
            MpcState state = detector.getMpcState();

            if (state.is_valid) {
                float steering = mpc.computeSteeringAngle(state, desired_velocity);
                int servo = 80 + static_cast<int>(steering);
                comm.sendCommands(desired_velocity, servo);
            }
        }
    });

    while (running) std::this_thread::sleep_for(std::chrono::milliseconds(10));

    cap.release();
    camera_thread.join();
    mpc_thread.join();
    cv::destroyAllWindows();
}

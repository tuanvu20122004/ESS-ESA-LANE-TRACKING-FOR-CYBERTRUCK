#include "logic.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <pthread.h>
#include <cmath>

void bindToCore(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    pthread_t current_thread = pthread_self();
    int result = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
    if (result != 0) {
        std::cerr << "[LOGIC] Error setting thread affinity." << std::endl;
    } else {
        std::cout << "[LOGIC] Thread bound to core " << core_id << "." << std::endl;
    }
}

Logic::Logic(const std::string& videoPath)
    : detector(videoPath, 640, 480),
      comm("/dev/ttyACM0", 115200),
      udp_send("192.168.1.102", 9996),
      //logger("Curvature.txt"),
      //logger1("steering.txt"),
      //logger2("Yaw.txt"),
      distance_detector("/home/tuandevvtx/yolov8n.onnx", 700.0f)
{
    mpc.init(1000.0f, 50.0f, 5.0f);
    mpc.debugMatrices();
    mpc.setVehicleParams(0.2515f, 2.3f, 0.132f, 0.12f, 0.04f, 0.02f, 0.04f);

    if (!detector.isOpened()) {
        std::cerr << "[LOGIC] LaneDetector/Camera không mở được." << std::endl;
        throw std::runtime_error("LaneDetector not opened");
    }

    std::cout << "[LOGIC] MPC initialized." << std::endl;
}

void Logic::run() {
    // ---- Camera thread ----
    std::thread camera_thread([&]() {
        bindToCore(0);
        cv::Mat frame;

        while (running.load()) {
            if (!detector.getFrame(frame)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                latest_frame = frame.clone();
            }

            int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q') {
                running.store(false);
                break;
            }
        }
    });

    // ---- Processing + MPC thread ----
    std::thread mpc_thread([&]() {
        bindToCore(1);

        cv::Mat frame_local;
        auto last_send = std::chrono::steady_clock::now();

        while (running.load()) {
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                if (!latest_frame.empty()) {
                    frame_local = latest_frame.clone();
                } else {
                    frame_local.release();
                }
            }

            if (frame_local.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // Tách 2 nhánh:
            // 1) frame_display: để detect vật thể + vẽ bbox + gửi UDP
            // 2) frame_lane: để xử lý lane, không bị nhiễu bởi bbox/text
            cv::Mat frame_display = frame_local.clone();
            cv::Mat frame_lane    = frame_local.clone();

            // Detect vật thể và vẽ khoảng cách lên frame gốc
            distance_detector.detectAndDraw(frame_display);

            // Lane detection dùng frame sạch
            detector.processFrame(frame_lane);

            std::vector<cv::Point> centerline = detector.getCenterline();
            cv::Mat birdEyeView = detector.getBirdEyeView();

            MpcState state = mpc.computeMpcParameters(centerline, birdEyeView);

            // Gửi ảnh detect vật thể về server
            if (!frame_display.empty()) {
                udp_send.sendFrame(frame_display, 60);
                std::this_thread::sleep_for(std::chrono::milliseconds(40));
            }

            auto now = std::chrono::steady_clock::now();

            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send).count() >= 100) {
                last_send = now;

                if (state.is_valid) {
                    float steering = mpc.computeSteeringAngle(state, desired_velocity);

                    std::cout << "Gia tri goc lai mpc: " << steering << std::endl;

                    steering = 0.02f * std::pow(steering, 3) + 1.15f * steering;

                    //logger1.log("Steering", steering);

                    if (steering <= -25.0f)
                        steering = -25.0f;
                    else if (steering >= 25.0f)
                        steering = 25.0f;

                    std::cout << "Gia tri goc lai qua noi suy: " << steering << std::endl;

                    int servo = static_cast<int>(std::lround(97.0f + steering));
                    comm.sendCommands(desired_velocity, servo);
                }
            }
        }
    });

    while (running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (camera_thread.joinable()) camera_thread.join();
    if (mpc_thread.joinable()) mpc_thread.join();

    std::cout << "[LOGIC] Stopped cleanly." << std::endl;
}
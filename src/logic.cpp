#include "logic.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <pthread.h>

void bindToCore(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    pthread_t current_thread = pthread_self();
    int result = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
    if (result != 0)
        std::cerr << "[LOGIC] Error setting thread affinity.\n";
    else
        std::cout << "[LOGIC] Thread bound to core " << core_id << ".\n";
}

Logic::Logic(const std::string& videoPath)
    : detector(videoPath, 640, 480),
      comm("/dev/ttyACM0", 115200),
      udp_send("192.168.1.108", 9996),
      logger("Curvature.txt"),
      logger1("steering.txt"),
      logger2("Yaw.txt") {

    mpc.init(1000.0f, 50.0f, 5.0f);
    mpc.debugMatrices();
    mpc.setVehicleParams(0.2515f, 2.3f, 0.132f, 0.12f, 0.04f, 0.02f, 0.04f);

    if (!detector.isOpened()) {
        std::cerr << "[LOGIC] LaneDetector/Camera không mở được.\n";
        throw std::runtime_error("LaneDetector not opened");
    }
    std::cout << "[LOGIC] MPC initialized.\n";
}

void Logic::run() {
    // ---- CAMERA THREAD ----
    std::thread camera_thread([&]() {
        bindToCore(0);
        cv::Mat frame;
        cv::namedWindow("Live Feed", cv::WINDOW_AUTOSIZE);

        while (running.load()) {
            if (!detector.getFrame(frame)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                latest_frame = detector.getFrameResize().clone();
            }

            if (!detector.getBirdEyeView().empty())
                cv::imshow("Bird_eye_view", detector.getBirdEyeView());

            int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q') {
                running.store(false);
                break;
            }
        }
    });

    // ---- MPC THREAD ----
    std::thread mpc_thread([&]() {
        bindToCore(0);
        cv::Mat frame_local;
        auto last_send = std::chrono::steady_clock::now();

        while (running.load()) {
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                if (!latest_frame.empty())
                    frame_local = latest_frame.clone();
                else
                    frame_local.release();
            }

            if (frame_local.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            detector.processFrame(frame_local);
            std::vector<cv::Point> centerline = detector.getCenterline();
            cv::Mat birdEyeView = detector.getBirdEyeView();

            if (!birdEyeView.empty()) {
                udp_send.sendFrame(birdEyeView, 70);
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
            }

            // ======= Gửi lệnh theo chu kỳ 10ms =======
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send).count() >= 150) {
                last_send = now;

                if (detector.hasValidLane()) {
                    MpcState state = mpc.computeMpcParameters(centerline, birdEyeView);
                    if (state.is_valid) {
                        float steering = mpc.computeSteeringAngle(state, desired_velocity);
                        int servo = static_cast<int>(std::lround(97.0f + steering));
                        comm.sendCommands(desired_velocity, servo);
                    }
                }
            }
        }
    });

    // ---- CHỜ THREAD KẾT THÚC ----
    while (running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (camera_thread.joinable()) camera_thread.join();
    if (mpc_thread.joinable()) mpc_thread.join();

    cv::destroyAllWindows();
    std::cout << "[LOGIC] Stopped cleanly.\n";
}

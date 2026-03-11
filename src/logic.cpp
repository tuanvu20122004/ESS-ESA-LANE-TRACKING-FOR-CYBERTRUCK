#include "logic.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <pthread.h>
#include <cmath>

void bindToCore(int core_id)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    pthread_t current_thread = pthread_self();

    int result = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);

    if (result != 0)
        std::cerr << "[LOGIC] Error setting thread affinity." << std::endl;
    else
        std::cout << "[LOGIC] Thread bound to core " << core_id << std::endl;
}

Logic::Logic(const std::string& videoPath)
    : detector(videoPath, 640, 480),
      comm("/dev/ttyACM0", 115200),
      udp_send("192.168.1.113", 9996)
{
    mpc.init(1000.0f, 50.0f, 5.0f);
    mpc.debugMatrices();

    mpc.setVehicleParams(0.2515f, 2.3f, 0.132f, 0.12f, 0.04f, 0.02f, 0.04f);

    if (!detector.isOpened())
    {
        std::cerr << "[LOGIC] Camera not opened." << std::endl;
        throw std::runtime_error("LaneDetector not opened");
    }

    std::cout << "[LOGIC] MPC initialized." << std::endl;
}

void Logic::run()
{
    std::thread camera_thread([&]()
    {
        bindToCore(0);

        cv::Mat frame;

        while (running.load())
        {
            if (!detector.getFrame(frame))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                latest_frame = detector.getFrameResize();
            }

            int key = cv::waitKey(1);

            if (key == 27 || key == 'q' || key == 'Q')
            {
                running.store(false);
                break;
            }
        }
    });

    std::thread mpc_thread([&]()
    {
        bindToCore(1);

        cv::Mat frame_local;
        auto last_send = std::chrono::steady_clock::now();

        while (running.load())
        {
            {
                std::lock_guard<std::mutex> lock(frame_mutex);

                if (!latest_frame.empty())
                    frame_local = latest_frame;
                else
                    frame_local.release();
            }

            if (frame_local.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            detector.processFrame(frame_local);
            std::vector<cv::Point> centerline = detector.getCenterline();
            cv::Mat birdEyeView = detector.getBirdEyeView();
            MpcState state = mpc.computeMpcParameters(centerline, birdEyeView);

            // Gửi ảnh BEV sang laptop
            if (!frame_local.empty())
            {
                udp_send.sendFrame(frame_local, 70);
                std::this_thread::sleep_for(std::chrono::milliseconds(40));
            }

            // Nhận khoảng cách từ laptop
            udp_send.receiveDistance();
            float distance = udp_send.getDistance();

            auto now = std::chrono::steady_clock::now();

            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send).count() >= 100)
            {
                last_send = now;

                if (state.is_valid)
                {
                    float steering = mpc.computeSteeringAngle(state, desired_velocity);

                    steering = 0.01f * std::pow(steering, 3) + 1.5f * steering;

                    if (steering <= -25.0f) steering = -25.0f;
                    else if (steering >= 25.0f) steering = 25.0f;

                    int servo = static_cast<int>(std::lround(97.0f + steering));

                    // Safety stop:
                    // chỉ dừng khi có distance hợp lệ và nhỏ hơn ngưỡng
                    float velocity_cmd = desired_velocity;

                    if (distance > 0.0f && distance < 12.0f)
                    {
                        velocity_cmd = 0.0f;
                    }
                    comm.sendCommands(velocity_cmd, servo);
                }
            }
        }
    });

    while (running.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if (camera_thread.joinable()) camera_thread.join();
    if (mpc_thread.joinable()) mpc_thread.join();

    std::cout << "[LOGIC] Stopped cleanly." << std::endl;
}
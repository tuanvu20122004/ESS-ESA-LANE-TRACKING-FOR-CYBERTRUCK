#include <iostream>
#include "logic.hpp"

int main(int argc, char* argv[]) {
    std::cout << "=== Lane Keeping System with MPC Controller ===" << std::endl;

    std::string videoPath = (argc > 1) ? argv[1] : "Demo.mp4";
    Logic logic(videoPath);
    logic.run();

    std::cout << "Program finished." << std::endl;
    return 0;
}

#include "camera/Realsense.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto camera = std::make_shared<Camera>("Realsense", 30, 1280, 720);
    rclcpp::spin(camera);

    rclcpp::shutdown();
    return 0;
}
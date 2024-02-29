#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "PurePursuit.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    const std::string pfp = "/home/sj/Project/SJ_CAR/path/sj_track_path.txt";
    const double xSpeed = 0.2;
    auto node = std::make_shared<PurePursuit>(pfp, xSpeed);
    rclcpp::spin(node);
    return 0;
}
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "kitti_odom/KittiOdom.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto kitti_odom = std::make_shared<KittiOdom>(10);
    rclcpp::spin(kitti_odom);
    rclcpp::shutdown();
    return 0;
}
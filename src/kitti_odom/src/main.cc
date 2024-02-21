#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "kitti_odom/KittiOdom.h"

int main(int argc, char **argv) {
    int fps;
    if (argc != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("KITTI_ODOM"), "参数不合法！");
        exit(1);
    } else {
        fps = atoi(argv[1]);
        RCLCPP_INFO(rclcpp::get_logger("KITTI_ODOM"), "设置fps为: %d", fps);
    }

    rclcpp::init(argc, argv);
    auto kitti_odom = std::make_shared<KittiOdom>(fps);
    rclcpp::spin(kitti_odom);
    rclcpp::shutdown();
    return 0;
}
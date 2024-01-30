#include <rclcpp/rclcpp.hpp>

#include "SystemRos2.h"

using namespace std;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto system = std::make_shared<SystemRos2>();
    rclcpp::spin(system);
    system->shutdown();

    RCLCPP_INFO(system->get_logger(), "正在保存地图到 /home/sj/Project/SJ_CAR/map/");
    system->saveMap("/home/sj/Project/SJ_CAR/map/");
    RCLCPP_INFO(system->get_logger(), "地图保存成功！");

    rclcpp::shutdown();
    system->trickStatistic();

    return 0;
}
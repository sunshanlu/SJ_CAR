#include <rclcpp/rclcpp.hpp>

#include "SystemRos2.h"

using namespace std;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto system = std::make_shared<SystemRos2>();
    system->run();

    rclcpp::shutdown();

    return 0;
}
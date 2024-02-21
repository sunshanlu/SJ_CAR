#include "BaseLinkPublisher.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BaseLinkPublisher>();
    rclcpp::spin(node);
    return 0;
}
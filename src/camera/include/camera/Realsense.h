#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sj_interfaces/msg/frame.hpp"

using namespace sj_interfaces::msg;
using namespace std::chrono_literals;

class Camera : public rclcpp::Node {
public:
    Camera(const std::string &node_name, float fps, unsigned width, unsigned height);

    void timerCallback();

private:
    rs2::config cfg;
    rs2::pipeline pipe;
    rclcpp::Publisher<Frame>::SharedPtr mp_publisher;
    rclcpp::TimerBase::SharedPtr mp_timer;
    unsigned m_width;
    unsigned m_height;
};

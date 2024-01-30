#include "camera/Realsense.h"

Camera::Camera(const std::string &node_name, float fps, unsigned width, unsigned height)
    : Node(node_name)
    , m_width(width)
    , m_height(height) {
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    auto selection = pipe.start(cfg);
    auto depthSensor = selection.get_device().first<rs2::depth_sensor>();
    depthSensor.set_option(RS2_OPTION_LASER_POWER, 0);

    mp_publisher = create_publisher<Frame>("camera/stereo", 10);
    mp_timer = create_wall_timer(0.1ms, std::bind(&Camera::timerCallback, this));
}

void Camera::timerCallback() {
    // RCLCPP_INFO(get_logger(), "定时器开始");
    Frame frameMsg;
    frameMsg.encoding = "mono8";
    frameMsg.channel = 1;
    frameMsg.width = m_width;
    frameMsg.height = m_height;

    rs2::frameset frame = pipe.wait_for_frames();
    frameMsg.time_stamp = frame.get_timestamp();
    auto leftFrame = frame.get_infrared_frame(1);
    auto rightFrame = frame.get_infrared_frame(2);

    auto left_start = (uint8_t *)leftFrame.get_data();
    auto right_start = (uint8_t *)rightFrame.get_data();

    for (int i = 0; i < m_width * m_height; ++i) {
        frameMsg.left_img.push_back(left_start[i]);
        frameMsg.right_img.push_back(right_start[i]);
    }
    mp_publisher->publish(frameMsg);
}

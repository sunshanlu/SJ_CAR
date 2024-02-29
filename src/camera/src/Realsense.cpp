#include "camera/Realsense.h"

Camera::Camera(const std::string &node_name, float fps, unsigned width, unsigned height)
    : Node(node_name)
    , m_width(width)
    , m_height(height) {
    m_cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
    m_cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    rs2::pipeline_profile selection;
    while (rclcpp::ok()) {
        try {
            selection = m_pipe.start(m_cfg);
            break;
        } catch (rs2::error &e) {
            RCLCPP_INFO(get_logger(), "无法启动相机，原因：%s", e.what());
            std::this_thread::sleep_for(1s);
        }
    }
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

    rs2::frameset frame = m_pipe.wait_for_frames();
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

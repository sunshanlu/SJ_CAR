#include "Camera.h"

void Camera::getFrame(cv::Mat &leftImg, cv::Mat &rightImg, double &timestamp) {
    rs2::frameset frame = pipe.wait_for_frames();
    timestamp = frame.get_timestamp();
    auto leftFrame = frame.get_infrared_frame(1);
    auto rightFrame = frame.get_infrared_frame(2);
    leftImg = cv::Mat(cv::Size(1280, 720), CV_8UC1, (void *)leftFrame.get_data(), cv::Mat::AUTO_STEP);
    rightImg = cv::Mat(cv::Size(1280, 720), CV_8UC1, (void *)rightFrame.get_data(), cv::Mat::AUTO_STEP);
}

Camera::Camera() {
    m_times += 1;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 1280, 720, RS2_FORMAT_Y8, 30);
    auto selection = pipe.start(cfg);
    auto depthSensor = selection.get_device().first<rs2::depth_sensor>();
    depthSensor.set_option(RS2_OPTION_LASER_POWER, 0);
}

Camera::SharedPtr Camera::getInstance() {
    static SharedPtr instance(new Camera());
    return instance;
}

unsigned Camera::m_times = 0;
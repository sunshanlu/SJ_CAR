#pragma once

#include <chrono>
#include <string>

#include <fmt/format.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sj_interfaces/msg/frame.hpp"

using sj_interfaces::msg::Frame;
using namespace std::chrono_literals;

const static std::string KITTI_ODOM_PATH = "/media/lu/489C4BAA9C4B917C/Dataset/00";

class KittiOdom : public rclcpp::Node {
public:
    KittiOdom(float fps)
        : Node("KITTI_ODOM") {
        mp_framePub = create_publisher<Frame>("camera/stereo", 10);
        mp_timer = create_wall_timer(1s / fps, std::bind(&KittiOdom::timerCallback, this));
    }

    void timerCallback();

    ~KittiOdom() { cv::destroyAllWindows(); }

private:
    // 根据kitti数据集路径创建帧数据
    void create_frame(Frame &frame);

    static size_t mi_idx;
    rclcpp::Publisher<Frame>::SharedPtr mp_framePub;
    rclcpp::TimerBase::SharedPtr mp_timer;
};
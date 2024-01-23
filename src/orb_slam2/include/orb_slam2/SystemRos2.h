#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "Camera.h"
#include "System.h"
#include "Map.h"

using namespace geometry_msgs::msg;

class SystemRos2 : public rclcpp::Node {
public:
    SystemRos2();

    // slam系统主逻辑
    bool run();

    // 保存地图
    bool saveMap(const std::string &filename);

    // 加载地图
    bool loadMap(const std::string &filename);

private:
    // 跟踪过程统计
    void trickStatistic();

    // 发布3D的位姿信息
    void publishPose3D(const cv::Mat &pose3d);

    // 发布2D的位姿信息
    void publishPose2D(const cv::Mat &pose2d);

    rclcpp::Publisher<Pose>::SharedPtr mp_pose3dPub;   ///< 发布3D位姿数据
    rclcpp::Publisher<Pose2D>::SharedPtr mp_pose2dPub; ///< 发布2D位姿数据
    ORB_SLAM2::System::SharedPtr mp_slamSystem;        ///< ORB-SLAM2系统
    std::vector<float> mv_trickTime;                   ///< 跟踪每一帧花费时间
    Camera::SharedPtr mp_camera;                       ///< 相机指针用于获取图像
};

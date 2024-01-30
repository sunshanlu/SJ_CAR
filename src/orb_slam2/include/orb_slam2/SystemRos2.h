#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "Map.h"
#include "System.h"
#include "sj_interfaces/msg/frame.hpp"

using namespace geometry_msgs::msg;

class SystemRos2 : public rclcpp::Node {
public:
    SystemRos2();

    void shutdown() { mp_slamSystem->Shutdown(); }

    // slam系统主逻辑
    // bool run();

    // 保存地图
    bool saveMap(const std::string &filename);

    // 加载地图
    bool loadMap(const std::string &filename, ORB_SLAM2::ORBVocabulary *voc, ORB_SLAM2::KeyFrameDatabase *kfdb);

    // 跟踪过程统计
    void trickStatistic();

private:
    // 收到帧数据时，进行回调
    void grabFrame(sj_interfaces::msg::Frame::SharedPtr frame);

    void convert2CvMat(sj_interfaces::msg::Frame::SharedPtr frame, cv::Mat &leftImg, cv::Mat &rightImg);

    // 发布3D的位姿信息
    void publishPose3D(const cv::Mat &pose3d);

    // 发布2D的位姿信息
    void publishPose2D(const cv::Mat &pose2d);

    rclcpp::Publisher<Pose>::SharedPtr mp_pose3dPub;                        ///< 发布3D位姿数据
    rclcpp::Publisher<Pose2D>::SharedPtr mp_pose2dPub;                      ///< 发布2D位姿数据
    ORB_SLAM2::System::SharedPtr mp_slamSystem;                             ///< ORB-SLAM2系统
    std::vector<float> mv_trickTime;                                        ///< 跟踪每一帧花费时间
    rclcpp::Subscription<sj_interfaces::msg::Frame>::SharedPtr mp_subFrame; ///< 订阅帧数据
};

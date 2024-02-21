#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "FramePublisher.h"
#include "Map.h"
#include "MapPublisher.h"
#include "System.h"
#include "sj_interfaces/msg/frame.hpp"

using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;

class SystemRos2 : public rclcpp::Node {
public:
    SystemRos2(const std::string &voc_fp, const std::string &setting_fp, const std::string &map_fp,
               ORB_SLAM2::System::eSensor sensor, bool buildMap, bool trackMap, bool bUseViewer);

    ~SystemRos2() {
        shutdown();
        trickStatistic();
        if (!mb_trackMap) {
            RCLCPP_INFO(get_logger(), "正在保存地图到指定路径");
            bool ret = saveMap();
            if (ret) {
                RCLCPP_INFO(get_logger(), "地图保存成功！");
            }
        } else {
            RCLCPP_INFO(get_logger(), "跟踪过程顺利完成！");
        }
    }

    void shutdown() { mp_slamSystem->Shutdown(); }

    // 保存地图
    bool saveMap();

    // 加载地图
    bool loadMap();

    // 跟踪过程统计
    void trickStatistic();

private:
    // 收到帧数据时，进行回调
    void grabFrame(sj_interfaces::msg::Frame::SharedPtr frame);

    void convert2CvMat(sj_interfaces::msg::Frame::SharedPtr frame, cv::Mat &leftImg, cv::Mat &rightImg);

    // 发布3D的位姿信息
    void publishPose3D(const cv::Mat &pose3d);

    // 可视化定时器回调函数
    void publishVisual() {
        mp_mapPublisher->Refresh(now());
        mp_framePublisher->Refresh(now());
    }

    rclcpp::Subscription<sj_interfaces::msg::Frame>::SharedPtr mp_subFrame; ///< 订阅帧数据
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr mp_pose3dPub;    ///< 发布3D位姿
    rclcpp::TimerBase::SharedPtr mp_pubMapTimer;                            ///< 可视化的定时器
    ORB_SLAM2::System::SharedPtr mp_slamSystem;                             ///< ORB-SLAM2系统
    ORB_SLAM2::MapPublisher::SharedPtr mp_mapPublisher;                     ///< 发布地图（可视化）
    ORB_SLAM2::FramePublisher::SharedPtr mp_framePublisher;                 ///< 发布帧数据（可视化）
    std::vector<float> mv_trickTime;                                        ///< 跟踪每一帧花费时间
    std::string ms_mapFp;                                                   ///< 地图文件加载和保存路径
    bool mb_trackMap;                                                       ///< 是否是跟踪地图模式
};

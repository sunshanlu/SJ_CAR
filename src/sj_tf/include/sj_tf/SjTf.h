#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace geometry_msgs::msg;
using namespace std::placeholders;

typedef std::shared_ptr<tf2_ros::TransformBroadcaster> TransformBroadcasterPtr;

class SjTf : public rclcpp::Node {
public:
    SjTf();

private:
    /// 相机位姿订阅回调函数（发布动态坐标变换）
    void pubDynamicTF(Pose::SharedPtr poseMsg);

    rclcpp::Subscription<Pose>::SharedPtr mp_cameraPose; ///< 相机位姿订阅方
    TransformBroadcasterPtr mp_dynamicTfPub;             ///< camera_link到map的动态坐标变换发布方
};
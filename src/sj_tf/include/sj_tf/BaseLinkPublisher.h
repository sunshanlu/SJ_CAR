#pragma once

#include <chrono>
#include <memory>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>

#include "sj_interfaces/msg/base_link.hpp"

typedef std::shared_ptr<tf2_ros::TransformListener> TransformListenerPtr;
typedef std::unique_ptr<tf2_ros::Buffer> BufferPtr;

using namespace std::chrono_literals;
using namespace geometry_msgs::msg;
using namespace std::placeholders;
using namespace sj_interfaces::msg;

class BaseLinkPublisher : public rclcpp::Node {
public:
    BaseLinkPublisher()
        : Node("BaseLink_Publisher") {
        mp_tfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        mp_tfListener = std::make_shared<tf2_ros::TransformListener>(*mp_tfBuffer, this, false);
        mp_camPose = create_subscription<Pose>("camera/pose", 10, std::bind(&BaseLinkPublisher::pubBaseLink, this, _1));
        mp_blPub = create_publisher<BaseLink>("base/massage", 10);
        RCLCPP_INFO(get_logger(), "正在等待速度生成...");
    }

private:
    void pubBaseLink(Pose::SharedPtr poseMsg) {
        (void)poseMsg;
        // 查看buffer中是否可以进行坐标转换
        if (!mp_tfBuffer->canTransform("world", "base_link", tf2::TimePointZero))
            return;
        auto tfMsg = mp_tfBuffer->lookupTransform("world", "base_link", tf2::TimePointZero);

        // 对mp_lastTf进行初始化
        if (!mp_lastTf) {
            mp_lastTf = std::make_shared<TransformStamped>(tfMsg);
            return;
        }

        // 如果是相同的转换，则不发布
        if (mp_lastTf->header.stamp == tfMsg.header.stamp)
            return;

        rclcpp::Time nowTime(tfMsg.header.stamp);
        rclcpp::Time lastTime(mp_lastTf->header.stamp);
        auto durTime = (nowTime - lastTime).seconds();

        double roll_1, roll_2, pitch_1, pitch_2, yaw_1, yaw_2;
        bool isReliable = true;
        if (durTime > 200 * 1e-3) {
            RCLCPP_WARN(get_logger(), "时间差为%.3f秒, 速度计算不准确！", durTime);
            isReliable = false;
        }
        tf2::Quaternion q1(tfMsg.transform.rotation.x, tfMsg.transform.rotation.y, tfMsg.transform.rotation.z,
                           tfMsg.transform.rotation.w);
        tf2::Quaternion q2(mp_lastTf->transform.rotation.x, mp_lastTf->transform.rotation.y,
                           mp_lastTf->transform.rotation.z, mp_lastTf->transform.rotation.w);
        tf2::Matrix3x3 m1(q1), m2(q2);
        m1.getRPY(roll_1, pitch_1, yaw_1);
        m2.getRPY(roll_2, pitch_2, yaw_2);

        double speedX = (tfMsg.transform.translation.x - mp_lastTf->transform.translation.x) / durTime;
        double speedY = (tfMsg.transform.translation.y - mp_lastTf->transform.translation.y) / durTime;
        double speedYaw = (yaw_1 - yaw_2) / durTime;
        double posX = tfMsg.transform.translation.x;
        double posY = tfMsg.transform.translation.y;

        BaseLink baseLinkMsg;
        baseLinkMsg.stamp = tfMsg.header.stamp;
        baseLinkMsg.is_reliable = isReliable;
        baseLinkMsg.position.x = posX;
        baseLinkMsg.position.y = posY;
        baseLinkMsg.position.yaw = yaw_1;
        baseLinkMsg.velocity.x_vel = speedX;
        baseLinkMsg.velocity.y_vel = speedY;
        baseLinkMsg.velocity.yaw_vel = speedYaw;
        mp_blPub->publish(baseLinkMsg);

        if (isReliable) {
            RCLCPP_INFO(get_logger(), "速度为: (%.2f m/s, %.2f m/s, %.2f °/s)", speedX, speedY, speedYaw * 180 / M_PI);
            RCLCPP_INFO(get_logger(), "位姿为: (%.2f m, %.2f m, %.2f °)", posX, posY, yaw_1 * 180 / M_PI);
        }

        mp_lastTf = std::make_shared<TransformStamped>(tfMsg);
    }

    TransformListenerPtr mp_tfListener;               ///< 坐标变换监听器
    BufferPtr mp_tfBuffer;                            ///< 坐标变换缓冲区
    TransformStamped::SharedPtr mp_lastTf;            ///< 上一次转换信息
    rclcpp::Subscription<Pose>::SharedPtr mp_camPose; ///< 相机位姿订阅方
    rclcpp::Publisher<BaseLink>::SharedPtr mp_blPub;  ///< 底盘信息发布方
};

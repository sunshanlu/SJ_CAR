#pragma once
#include <fstream>
#include <limits>
#include <sstream>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sj_interfaces/msg/base_link.hpp"

using namespace sj_interfaces::msg;
using namespace geometry_msgs::msg;
using namespace std::placeholders;

struct PointPose {
    double x, y, yaw;
};
std::istream &operator>>(std::istream &is, PointPose &point);

class PurePursuit : public rclcpp::Node {
public:
    PurePursuit(std::string pfp, double xSpeed);

private:
    void pursuit(BaseLink::SharedPtr msg);

    bool getGoalIdx(BaseLink::SharedPtr msg, std::size_t &idx, double &lf);

    bool getNearIdx(BaseLink::SharedPtr msg, std::size_t &itx);

    double computeWheelAngle(const std::size_t &idx, BaseLink::SharedPtr msg, double &lf);

    bool isComplete(BaseLink::SharedPtr msg);

    std::vector<PointPose> mv_points;                        ///< 路径点vector
    rclcpp::Subscription<BaseLink>::SharedPtr mp_basePosPub; ///< 订阅的底盘位姿
    rclcpp::Publisher<Twist>::SharedPtr mp_velPub;           ///< 发布的底盘速度
    double m_xSpeed;                                         ///< 目标跟踪速度
    double lambda = 0.5;                                     ///< 前视距离超参数1
    double c = 0.8;                                          ///< 前视距离超参数2
    double sacle = 1.0;                                      ///< 转角超参数
    const double L = 0.498;                                  ///< 轴距
    const double W = 1;                                      ///< 轮距
};

double calcDistance(const BaseLink &baseState, const PointPose &pointPose);

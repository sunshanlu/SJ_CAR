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

/**
 * @brief 路径点struct
 * @details 维护路径点的坐标和航向角
 */
struct PointPose {
    double x, y, yaw;
};

/// PointPose的输入操作符
std::istream &operator>>(std::istream &is, PointPose &point);

class PurePursuit : public rclcpp::Node {
public:
    PurePursuit(std::string pfp, double xSpeed);

private:
    /// 纯追踪框架函数 
    void pursuit(BaseLink::SharedPtr msg);

    /// 获取预瞄点idx
    bool getGoalIdx(BaseLink::SharedPtr msg, std::size_t &idx, double &lf, const bool &backFlag);

    /// 获取最近点idx
    bool getNearIdx(BaseLink::SharedPtr msg, std::size_t &itx);

    /// 根据前视距离计算轮子转角（阿卡曼转角）
    double computeWheelAngle(const std::size_t &idx, BaseLink::SharedPtr msg, double &lf, const bool &backFlag);

    /// 判断是否跟踪到终点
    bool isComplete(BaseLink::SharedPtr msg, const bool &backFlag);

    /// 判断是否是向后移动
    bool isBack();

    /// 用于转换前进或者后退的逻辑
    void turnBack();

    /// 读取文件中的跟踪路径消息
    bool readPath(const std::string &pfp);

    /// 发送停止指令（专门封装，减少代码量）
    void pubStop();

    std::vector<PointPose> mv_points;                        ///< 路径点vector
    rclcpp::Subscription<BaseLink>::SharedPtr mp_basePosPub; ///< 订阅的底盘位姿
    rclcpp::Publisher<Twist>::SharedPtr mp_velPub;           ///< 发布的底盘速度
    double m_xSpeed;                                         ///< 目标跟踪速度
    double lambda = 0.5;                                     ///< 前视距离超参数1
    double c = 0.8;                                          ///< 前视距离超参数2
    double sacle = 1.0;                                      ///< 转角超参数
    const double L = 0.245;                                  ///< 轴距
    const double W = 1;                                      ///< 轮距
    bool mb_back = true;                                     ///< 是否向后运行
    std::mutex m_mutex;                                      ///< 是否向后运行的互斥锁
};

/// 计算距离函数（工具）
double calcDistance(const BaseLink &baseState, const PointPose &pointPose);

/// 对称函数（工具）
PointPose symmetrizatePoint(BaseLink::SharedPtr msg, const PointPose &point, const double &lf);
#include "PurePursuit.h"
/**
 * @brief PointPose的输入操作符
 *
 * @param is    输入流
 * @param point 路径点
 * @return std::istream&
 */
std::istream &operator>>(std::istream &is, PointPose &point) {
    is >> point.x >> point.y >> point.yaw;
    return is;
}

/**
 * @brief 纯追踪节点构造，读取路径文件，初始化订阅和发布
 *
 * @param pfp       路径文件的存储路径
 * @param xSpeed    跟踪的速度信息
 */
PurePursuit::PurePursuit(std::string pfp, double xSpeed)
    : Node("Pure_Pursuit")
    , m_xSpeed(xSpeed) {
    if (!readPath(pfp)) {
        exit(1);
    }
    mp_basePosPub = create_subscription<BaseLink>("base/massage", 10, std::bind(&PurePursuit::pursuit, this, _1));
    mp_velPub = create_publisher<Twist>("cmd_vel", 10);
    RCLCPP_INFO(get_logger(), "纯追踪节点就绪！");
}

/**
 * @brief 读取路径信息
 *
 * @param pfp   文件的读取路径
 * @return true
 * @return false
 */
bool PurePursuit::readPath(const std::string &pfp) {
    std::ifstream ifs(pfp);
    if (!ifs.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "无法打开文件 %s", pfp.c_str());
        return false;
    }
    bool isTitle = true;
    std::string lineStr;
    while (std::getline(ifs, lineStr)) {
        if (isTitle) {
            isTitle = false;
            continue;
        }

        std::istringstream isstr(lineStr);
        PointPose point;
        isstr >> point;
        mv_points.push_back(point);
    }
    return true;
}

/**
 * @brief 发布停止指令
 *      condition1: 判断已经走完
 *      condition2: 未找到目标点
 */
void PurePursuit::pubStop() {
    Twist stop;
    stop.linear.x = 0;
    stop.linear.y = 0;
    stop.linear.z = 0;
    stop.angular.x = 0;
    stop.angular.y = 0;
    stop.angular.z = 0;
    mp_velPub->publish(stop);
}

/**
 * @brief 纯追踪主框架
 *
 * @param msg 接收的地盘的位姿信息
 */
void PurePursuit::pursuit(BaseLink::SharedPtr msg) {
    Twist ctlTwist;
    std::size_t idx;
    double lf;
    bool backFlag = isBack();
    if (!getGoalIdx(msg, idx, lf, backFlag)) {
        pubStop();
        return;
    }
    RCLCPP_INFO(get_logger(), "目标点索引为 %d", idx);
    double angle = computeWheelAngle(idx, msg, lf, backFlag);
    if (isComplete(msg, backFlag)) {
        pubStop();
        RCLCPP_INFO(get_logger(), "追踪结束！");
        return;
    }
    if (backFlag) {
        ctlTwist.linear.x = -m_xSpeed;
        // ctlTwist.angular.z = -angle * sacle;
        ctlTwist.angular.z = angle * sacle;
    } else {
        ctlTwist.linear.x = m_xSpeed;
        ctlTwist.angular.z = angle * sacle;
    }

    ctlTwist.linear.y = 0;
    ctlTwist.linear.z = 0;
    ctlTwist.angular.x = 0;
    ctlTwist.angular.y = 0;
    mp_velPub->publish(ctlTwist);
}

/**
 * @brief 找到一个预瞄点，返回其在mv_points中的索引
 *
 * @param msg       底盘位姿信息
 * @param idx       预瞄点的索引（输出）
 * @param lf        预设的前视距离
 * @param backFlag  后向标志位
 * @return true     找到了
 * @return false    未找到
 */
bool PurePursuit::getGoalIdx(BaseLink::SharedPtr msg, std::size_t &idx, double &lf, const bool &backFlag) {
    if (!getNearIdx(msg, idx)) {
        return false;
    }
    double speed = sqrt(pow(msg->velocity.x_vel, 2) + pow(msg->velocity.y_vel, 2));
    lf = lambda * speed + c; //! 前视距离
    double l = 0.0;
    while (l < lf) {
        if (backFlag) {
            double dx = mv_points[idx].x - mv_points[idx - 1].x;
            double dy = mv_points[idx].y - mv_points[idx - 1].y;
            l += sqrt(pow(dx, 2) + pow(dy, 2));
            idx--;
            if (idx < 1) {
                break;
            }
        } else {
            double dx = mv_points[idx + 1].x - mv_points[idx].x;
            double dy = mv_points[idx + 1].y - mv_points[idx].y;
            l += sqrt(pow(dx, 2) + pow(dy, 2));
            idx++;
            if (idx >= mv_points.size() - 1)
                break;
        }
    }
    auto goalpoint = mv_points[idx];
    RCLCPP_INFO(get_logger(), "路径起点 %.3f\t%.3f", mv_points[0].x, mv_points[0].y);
    RCLCPP_INFO(get_logger(), "目标点位姿 %.3f\t%.3f", goalpoint.x, goalpoint.y);
    return true;
}

/**
 * @brief 找到距离当前车辆位置的最近点
 *
 * @param msg       当前车辆位置
 * @param itx       输出的最近点的索引
 * @return true     找到最近点
 * @return false    路径中没有点信息
 */
bool PurePursuit::getNearIdx(BaseLink::SharedPtr msg, std::size_t &itx) {
    if (mv_points.size() == 0) {
        RCLCPP_WARN(get_logger(), "路径点为空！");
        return false;
    }
    std::vector<double> distances;
    for (const auto &point : mv_points) {
        distances.push_back(calcDistance(*msg, point));
    }
    double minDis = std::numeric_limits<double>::max();
    std::size_t minIdx = 0;
    for (std::size_t idx = 0; idx < mv_points.size(); ++idx) {
        if (distances[idx] < minDis) {
            minDis = distances[idx];
            minIdx = idx;
        }
    }
    itx = minIdx;
    return true;
}

/**
 * @brief 计算阿克曼模型转角
 *
 * @param idx       输入的预瞄点索引
 * @param msg       当前车辆位置
 * @param lf        待修正的前视距离
 * @param backFlag  后向标志位
 * @return double   转角
 */
double PurePursuit::computeWheelAngle(const std::size_t &idx, BaseLink::SharedPtr msg, double &lf,
                                      const bool &backFlag) {
    double dx = mv_points[idx].x - msg->position.x;
    double dy = mv_points[idx].y - msg->position.y;
    lf = sqrt(pow(dx, 2) + pow(dy, 2));
    PointPose goalPoint;
    if (backFlag) {
        goalPoint = symmetrizatePoint(msg, mv_points[idx], lf);
        RCLCPP_INFO(get_logger(), "对称后的点的坐标：x: %.3f, y: %.3f", goalPoint.x, goalPoint.y);
    } else {
        goalPoint = mv_points[idx];
    }
    double alpha = std::atan2(goalPoint.y - msg->position.y, goalPoint.x - msg->position.x) - msg->position.yaw;
    RCLCPP_INFO(get_logger(), "计算的alpha %.3f \t 前视距离lf %.3f", alpha * 180 / M_PI, lf);
    double delta = std::atan2(2 * L * std::sin(alpha) / lf, 1.0);
    return delta;
}

// TODO：这么操作可能会导致bug（速度过快或者延迟过高时，会导致冲出0.2m的范围）需要添加速度向量和追踪方向的角度判断
bool PurePursuit::isComplete(BaseLink::SharedPtr msg, const bool &backFlag) {
    if (!backFlag) {
        return calcDistance(*msg, mv_points[mv_points.size() - 1]) < 0.2 ? true : false;
    } else {
        return calcDistance(*msg, mv_points[0]) < 0.2 ? true : false;
    }
}

/**
 * @brief 计算距离工具函数
 *
 * @param baseState 车辆状态
 * @param pointPose 目标点状态
 * @return double   车辆与目标点距离
 */
double calcDistance(const BaseLink &baseState, const PointPose &pointPose) {
    double dx = baseState.position.x - pointPose.x;
    double dy = baseState.position.y - pointPose.y;
    return sqrt(pow(dx, 2) + pow(dy, 2));
}

/**
 * @brief 查看是否后退
 *
 * @return true     后退
 * @return false    前进
 */
bool PurePursuit::isBack() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return mb_back;
}

/**
 * @brief 转换后退或前进函数
 * 等待遥控器信息接收后调用
 */
void PurePursuit::turnBack() {
    std::unique_lock<std::mutex> lock(m_mutex);
    mb_back = !mb_back;
}

/**
 * @brief 工具函数，用于后退时，目标点的对称
 *
 * @param msg    底盘此时位置
 * @param point  待对称的目标点
 * @param lf     修正后的前视距离
 * @return PointPose 返回的对称点
 */
PointPose symmetrizatePoint(BaseLink::SharedPtr msg, const PointPose &point, const double &lf) {

    // double theta = std::atan2(point.y - msg->position.y, point.x - msg->position.x);
    // double beta = theta - msg->position.yaw - M_PI / 2;
    // double delta = theta - 2 * beta;
    // double dx = lf * std::cos(delta);
    // double dy = lf * std::sin(delta);
    // PointPose goalPoint;
    // goalPoint.x = msg->position.x + dx;
    // goalPoint.y = msg->position.y + dy;

    // // TODO：这里的目标点航向角没有计算，若要使用，记得计算！
    // goalPoint.yaw = -point.yaw;
    // std::cout << "theta: " << theta << "\t"
    //           << "beta: " << beta << "\t"
    //           << "delta: " << delta << "\t"
    //           << "dx: " << dx << "\t"
    //           << "dy: " << dy << std::endl;

    PointPose goalPoint;
    Eigen::Matrix2d Rbw, Rwb;
    Eigen::Vector2d twb, tbw(msg->position.x, msg->position.y);
    Rbw << std::cos(msg->position.yaw), -std::sin(msg->position.yaw), std::sin(msg->position.yaw),
        std::cos(msg->position.yaw);
    Rwb = Rbw.transpose();
    twb = -Rwb * tbw;

    Eigen::Vector2d Gw(point.x, point.y);
    Eigen::Vector2d Gb = Rbw * Gw + tbw;
    Eigen::Vector2d GbSym(-Gb.x(), Gb.y());
    Eigen::Vector2d GwSym = Rwb * GbSym + twb;

    /// 下面开始对称目标点偏航角
    Eigen::Vector2d GwYaw(std::tan(point.yaw), 1);
    Eigen::Vector2d GbYaw = Rbw * GwYaw;
    Eigen::Vector2d GbYawSym(-GbYaw.x(), GbYaw.y());
    Eigen::Vector2d GwYawSym = Rwb * GbYawSym;
    double yawSym = std::atan2(GwYawSym.y(), GwYawSym.x());

    goalPoint.x = GwSym.x();
    goalPoint.y = GwSym.y();
    goalPoint.yaw = yawSym;

    return goalPoint;
}
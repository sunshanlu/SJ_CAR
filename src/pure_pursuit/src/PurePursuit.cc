#include "PurePursuit.h"

std::istream &operator>>(std::istream &is, PointPose &point) {
    is >> point.x >> point.y >> point.yaw;
    return is;
}

PurePursuit::PurePursuit(std::string pfp, double xSpeed)
    : Node("Pure_Pursuit")
    , m_xSpeed(xSpeed) {
    std::ifstream ifs(pfp);
    if (!ifs.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "无法打开文件 %s", pfp.c_str());
        exit(1);
    }
    std::string lineStr;
    while (std::getline(ifs, lineStr)) {
        std::istringstream isstr(lineStr);
        PointPose point;
        isstr >> point;
        mv_points.push_back(point);
    }
    mp_basePosPub = create_subscription<BaseLink>("base/massage", 10, std::bind(&PurePursuit::pursuit, this, _1));
    mp_velPub = create_publisher<Twist>("cmd_vel", 10);
}

void PurePursuit::pursuit(BaseLink::SharedPtr msg) {
    Twist ctlTwist;
    std::size_t idx;
    double lf;
    if (!getGoalIdx(msg, idx, lf)) {
        return;
    }
    double angle = computeWheelAngle(idx, msg, lf);
    if (isComplete(msg)) {
        ctlTwist.linear.x = 0;
        ctlTwist.angular.z = 0;
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

bool PurePursuit::getGoalIdx(BaseLink::SharedPtr msg, std::size_t &idx, double &lf) {
    if (!getNearIdx(msg, idx)) {
        return false;
    }
    double speed = sqrt(pow(msg->velocity.x_vel, 2) + pow(msg->velocity.y_vel, 2));
    lf = lambda * speed + c; //! 前视距离
    double l = 0.0;
    while (l < lf && idx < mv_points.size() - 1) {
        double dx = mv_points[idx + 1].x - mv_points[idx].x;
        double dy = mv_points[idx + 1].y - mv_points[idx].y;
        l += sqrt(pow(dx, 2) + pow(dy, 2));
        idx++;
    }
    auto goalpoint = mv_points[idx];
    RCLCPP_INFO(get_logger(), "目标点位姿 %.3f\t%.3f", goalpoint.x, goalpoint.y);
    return true;
}

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

double PurePursuit::computeWheelAngle(const std::size_t &idx, BaseLink::SharedPtr msg, double &lf) {
    double dx = mv_points[idx].x - msg->position.x;
    double dy = mv_points[idx].y - msg->position.y;
    lf = sqrt(pow(dx, 2) + pow(dy, 2));
    const auto &goalPoint = mv_points[idx];
    double alpha = std::atan2(goalPoint.y - msg->position.y, goalPoint.x - msg->position.x) - msg->position.yaw;
    RCLCPP_INFO(get_logger(), "计算的alpha %.3f \t 前视距离lf %.3f", alpha * 180 / M_PI, lf);
    double delta = std::atan2(2 * L * std::sin(alpha) / lf, 1.0);
    return delta;
}

bool PurePursuit::isComplete(BaseLink::SharedPtr msg) {
    return calcDistance(*msg, mv_points[mv_points.size() - 1]) < 0.2 ? true : false;
}

double calcDistance(const BaseLink &baseState, const PointPose &pointPose) {
    double dx = baseState.position.x - pointPose.x;
    double dy = baseState.position.y - pointPose.y;
    return sqrt(pow(dx, 2) + pow(dy, 2));
}

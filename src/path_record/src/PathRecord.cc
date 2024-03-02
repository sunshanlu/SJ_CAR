#include "PathRecord.h"

/**
 * @brief 路径信息保存构造
 *
 * @param pfp 路径信息保存路径
 */
PathRecord::PathRecord(std::string pfp)
    : Node("Path_Record")
    , m_pfp(std::move(pfp)) {
    m_ofs.open(m_pfp, std::ios::out);
    m_ofs << "x\ty\tyaw\n";
    mp_basePos = create_subscription<BaseLink>("base/massage", 10, std::bind(&PathRecord::save2str, this, _1));
}

/**
 * @brief 析构时，进行路径的保存
 *
 */
PathRecord::~PathRecord() {
    RCLCPP_INFO(get_logger(), "路径信息保存成功，路径为%s", m_pfp.c_str());
    m_ofs.close();
}

/**
 * @brief 地盘订阅回调，将地盘消息保存到字符串中
 *
 * @param msg 输入的地盘位姿信息
 */
void PathRecord::save2str(BaseLink::SharedPtr msg) {
    m_ofs << msg->position.x << "\t";
    m_ofs << msg->position.y << "\t";
    m_ofs << msg->position.yaw << "\t";
    m_ofs << "\n";
}
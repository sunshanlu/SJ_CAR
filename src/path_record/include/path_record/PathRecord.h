#pragma once

#include <fstream>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sj_interfaces/msg/base_link.hpp>

using namespace sj_interfaces::msg;
using namespace std::placeholders;

class PathRecord : public rclcpp::Node {
public:
    PathRecord(std::string pfp);

    ~PathRecord();

private:
    void save2str(BaseLink::SharedPtr msg);

    rclcpp::Subscription<BaseLink>::SharedPtr mp_basePos; ///< 订阅的地盘位姿
    std::string m_pfp;                                    ///< 维护的路径信息的路径
    std::ofstream m_ofs;                                  ///< 维护输出文件留
};
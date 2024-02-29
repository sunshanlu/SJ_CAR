#include <rclcpp/rclcpp.hpp>

#include "PathRecord.h"

int main(int argc, char **argv){
    std::string path = "/home/sj/Project/SJ_CAR/path/sj_track_path.txt";
    rclcpp::init(argc, argv);
    auto path_record = std::make_shared<PathRecord>(path);
    rclcpp::spin(path_record);
    return 0;
}
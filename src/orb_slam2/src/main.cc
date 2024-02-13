#include <gflags/gflags.h>
#include <rclcpp/rclcpp.hpp>

#include "SystemRos2.h"

using namespace std;

DEFINE_bool(build_map, true, "是否以构建地图的方式启动，默认为 true");
DEFINE_bool(track_map, false, "是否以追踪地图的方式启动，默认为 false");
DEFINE_bool(show_map, true, "是否启用Viewer线程，默认为 true");
DEFINE_string(voc_fp, "/home/lu/Project/SJ_CAR/Vocabulary/ORBvoc.txt", "词袋文件的加载路径");
DEFINE_string(setting_fp, "/home/lu/Project/SJ_CAR/config/KITTI00-02.yaml", "配置文件的加载路径");
DEFINE_string(map_fp, "/home/lu/Project/SJ_CAR/map/", "地图文件的加载路径");

int main(int argc, char **argv) {

    // 用于处理ros-args
    int gflag_argc = argc;
    char **gflag_argv = argv;
    if (std::string(argv[argc - 1]) == "--ros-args") {
        gflag_argc = argc - 1;
    }

    gflags::ParseCommandLineFlags(&gflag_argc, &gflag_argv, false);
    rclcpp::init(argc, argv);

    auto system = std::make_shared<SystemRos2>(FLAGS_voc_fp, FLAGS_setting_fp, FLAGS_map_fp, ORB_SLAM2::System::STEREO,
                                               FLAGS_build_map, FLAGS_track_map, FLAGS_show_map);
    rclcpp::spin(system);
    rclcpp::shutdown();

    return 0;
}
#include "SystemRos2.h"

SystemRos2::SystemRos2()
    : Node("ORB_SLAM2") {
    std::string voc_fp = "/home/sj/Project/SJ_CAR/Vocabulary/ORBvoc.txt";
    std::string setting_fp = "/home/sj/Project/SJ_CAR/config/KITTI00-02.yaml";

    mp_pose3dPub = create_publisher<Pose>("camera/pose3d", 10);
    mp_pose2dPub = create_publisher<Pose2D>("camera/pose2d", 10);
    mp_slamSystem = std::make_shared<ORB_SLAM2::System>(voc_fp, setting_fp, ORB_SLAM2::System::STEREO, true);
    mp_camera = Camera::getInstance();
}

bool SystemRos2::run() {
    RCLCPP_INFO(get_logger(), "SLAM系统开始运行...");
    cv::Mat imLeft, imRight;
    double tframe;
    while (rclcpp::ok()) {
        // 读取realsense相机图像
        mp_camera->getFrame(imLeft, imRight, tframe);

        if (imLeft.empty()) {
            RCLCPP_ERROR(get_logger(), "Failed to load image");
            return false;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        auto framePose = mp_slamSystem->TrackStereo(imLeft, imRight, tframe);
        // std::cout << framePose << std::endl;
        if (framePose.empty())
            continue;
        publishPose3D(framePose);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        mv_trickTime.push_back(ttrack);
    }

    // 停止SLAM系统的所有线程
    mp_slamSystem->Shutdown();

    // 统计跟踪信息
    trickStatistic();
    return true;
}

/**
 * @brief 用于统计跟踪信息，单位为秒
 *
 */
void SystemRos2::trickStatistic() {
    sort(mv_trickTime.begin(), mv_trickTime.end());
    float totaltime = 0;
    std::size_t nend = mv_trickTime.size();
    for (std::size_t ni = 0; ni < nend; ni++) {
        totaltime += mv_trickTime[ni];
    }
    RCLCPP_INFO(get_logger(), "-------");
    RCLCPP_INFO(get_logger(), "total time: %f", totaltime);
    RCLCPP_INFO(get_logger(), "mean tracking time: %f", mv_trickTime[mv_trickTime.size() / 2]);
}

/**
 * @brief 发布相机的3D位姿Tcw
 * 使用Eigen库进行相机位姿转换到geometry::msg::Pose上
 * @param pose3d 类型为cv::Mat的3D位姿Tcw
 */
void SystemRos2::publishPose3D(const cv::Mat &pose3d) {
    Eigen::Matrix3f rotationMatrix;
    rotationMatrix << pose3d.at<float>(0, 0), pose3d.at<float>(0, 1), pose3d.at<float>(0, 2), pose3d.at<float>(1, 0),
        pose3d.at<float>(1, 1), pose3d.at<float>(1, 2), pose3d.at<float>(2, 0), pose3d.at<float>(2, 1),
        pose3d.at<float>(2, 2);
    Eigen::Quaternionf quat(rotationMatrix);
    quat.normalize(); // 注意四元数的标准化
    Pose pose;
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    pose.position.x = pose3d.at<float>(0, 3);
    pose.position.y = pose3d.at<float>(1, 3);
    pose.position.y = pose3d.at<float>(2, 3);
    mp_pose3dPub->publish(pose);
}
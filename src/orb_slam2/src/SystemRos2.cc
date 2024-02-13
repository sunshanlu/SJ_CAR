#include "SystemRos2.h"

using namespace std::placeholders;

SystemRos2::SystemRos2(const std::string &voc_fp, const std::string &setting_fp, const std::string &map_fp,
                       ORB_SLAM2::System::eSensor sensor, bool buildMap, bool trackMap, bool bUseViewer)
    : Node("ORB_SLAM2")
    , ms_mapFp(map_fp) {
    mp_slamSystem =
        std::make_shared<ORB_SLAM2::System>(voc_fp, setting_fp, map_fp, sensor, buildMap, trackMap, bUseViewer);
    if (trackMap && !buildMap) {
        mb_trackMap = true;
    } else {
        mb_trackMap = false;
    }

    mp_subFrame = create_subscription<sj_interfaces::msg::Frame>("camera/stereo", 10,
                                                                 std::bind(&SystemRos2::grabFrame, this, _1));
}

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

// 保存地图
// bool SystemRos2::saveMap(const std::string &filename) {
//     ORB_SLAM2::Map *map = mp_slamSystem->GetMap();
//     bool ret = map->saveMap(filename);
//     delete map;
//     map = nullptr;
//     return ret;
// }
bool SystemRos2::saveMap() {
    bool ret = mp_slamSystem->SaveMap(ms_mapFp);
    return ret;
}

// 加载地图
// bool SystemRos2::loadMap(const std::string &filename, ORB_SLAM2::ORBVocabulary *voc,
//                          ORB_SLAM2::KeyFrameDatabase *kfdb) {
//     ORB_SLAM2::Map *map = mp_slamSystem->GetMap();
//     return map->loadMap(filename, voc, kfdb);
// }
bool SystemRos2::loadMap() {
    bool ret = mp_slamSystem->LoadMap(ms_mapFp);
    return ret;
}

void SystemRos2::convert2CvMat(sj_interfaces::msg::Frame::SharedPtr frame, cv::Mat &leftImg, cv::Mat &rightImg) {
    cv::Mat leftTmp(frame->height, frame->width, CV_8UC1, frame->left_img.data());
    cv::Mat rightTmp(frame->height, frame->width, CV_8UC1, frame->right_img.data());

    // cv::imshow("image", leftTmp);
    // cv::waitKey(1);

    leftImg = std::move(leftTmp);
    rightImg = std::move(rightTmp);
}

void SystemRos2::grabFrame(sj_interfaces::msg::Frame::SharedPtr frame) {
    cv::Mat imLeft, imRight;
    convert2CvMat(frame, imLeft, imRight);

    if (imLeft.empty()) {
        RCLCPP_ERROR(get_logger(), "加载图片失败！");
        return;
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    auto framePose = mp_slamSystem->TrackStereo(imLeft, imRight, frame->time_stamp);
    // std::cout << framePose << std::endl;
    if (framePose.empty())
        RCLCPP_WARN(get_logger(), "跟踪丢失！");

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    mv_trickTime.push_back(ttrack);
}
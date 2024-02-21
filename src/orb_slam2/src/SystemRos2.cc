#include <cmath>

#include "SystemRos2.h"

using namespace std::placeholders;

/**
 * @brief SJ的SLAM系统的构造函数
 * @details
 *      1. buildMap模式下，读取voc_fp、setting_fp和map_fp
 *      2. trackMap模式下，读取voc_fp、setting_fp和map_fp，并将地图加载到ORB-SLAM2中（仅使用定位模式）
 * @param voc_fp        ORB-SLAM2使用的词典文件路径
 * @param setting_fp    ORB-SLAM2使用的设置文件路径
 * @param map_fp        ORB-SLAM2使用的地图文件路径
 * @param sensor        ORB-SLAM2使用的相机类型
 * @param buildMap      是否构建地图
 * @param trackMap      是否追踪地图
 * @param bUseViewer    是否使用Viewer（pangolin）
 */
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
    mp_pose3dPub = create_publisher<geometry_msgs::msg::Pose>("camera/pose", 10);

    ORB_SLAM2::MapPublisherRcl rclpub;
    rclpub.mpPub = create_publisher<visualization_msgs::msg::Marker>("visual/map/map_points", 10);
    rclpub.refMpPub = create_publisher<visualization_msgs::msg::Marker>("visual/map/ref_map_points", 10);
    rclpub.curFPub = create_publisher<visualization_msgs::msg::Marker>("visual/map/curr_frame", 10);
    rclpub.kfPub = create_publisher<visualization_msgs::msg::Marker>("visual/map/keyframes", 10);
    rclpub.civiGraphPub = create_publisher<visualization_msgs::msg::Marker>("visual/map/civi_graph", 10);
    rclpub.spTreePub = create_publisher<visualization_msgs::msg::Marker>("visual/map/span_tree", 10);

    auto framePublisher = create_publisher<sensor_msgs::msg::Image>("visual/frame/left_img", 10);
    auto framePublisherRight = create_publisher<sensor_msgs::msg::Image>("visual/frame/right_img", 10);

    mp_mapPublisher = std::make_shared<ORB_SLAM2::MapPublisher>(mp_slamSystem->GetMap(), rclpub, mb_trackMap);
    mp_framePublisher = std::make_shared<ORB_SLAM2::FramePublisher>(framePublisher, framePublisherRight);
    mp_framePublisher->SetMap(mp_slamSystem->GetMap());

    mp_pubMapTimer = create_wall_timer(50ms, std::bind(&SystemRos2::publishVisual, this));
}

void SystemRos2::trickStatistic() {
    sort(mv_trickTime.begin(), mv_trickTime.end());
    float totaltime = 0;
    std::size_t nend = mv_trickTime.size();
    for (std::size_t ni = 0; ni < nend; ni++) {
        totaltime += mv_trickTime[ni];
    }
    RCLCPP_INFO(get_logger(), "-------");
    RCLCPP_INFO(get_logger(), "系统跟踪总时间: %f", totaltime);
    RCLCPP_INFO(get_logger(), "平均跟踪时间为: %f", mv_trickTime[mv_trickTime.size() / 2]);
}

/**
 * @brief 发布相机的3D位姿Tcw
 * 使用Eigen库进行相机位姿转换到geometry::msg::Pose上
 * @param pose3d 类型为cv::Mat的3D位姿Tcw
 */
void SystemRos2::publishPose3D(const cv::Mat &pose3d) {
    Eigen::Matrix3f rotM;
    rotM << pose3d.at<float>(0, 0), pose3d.at<float>(0, 1), pose3d.at<float>(0, 2), pose3d.at<float>(1, 0),
        pose3d.at<float>(1, 1), pose3d.at<float>(1, 2), pose3d.at<float>(2, 0), pose3d.at<float>(2, 1),
        pose3d.at<float>(2, 2);
    Eigen::Quaternionf quat(rotM);
    quat.normalize();

    Pose poseMsg;
    poseMsg.position.x = pose3d.at<float>(0, 3);
    poseMsg.position.y = pose3d.at<float>(1, 3);
    poseMsg.position.z = pose3d.at<float>(2, 3);
    poseMsg.orientation.x = quat.x();
    poseMsg.orientation.y = quat.y();
    poseMsg.orientation.z = quat.z();
    poseMsg.orientation.w = quat.w();

    mp_pose3dPub->publish(poseMsg);
}

bool SystemRos2::saveMap() {
    bool ret = mp_slamSystem->SaveMap(ms_mapFp);
    return ret;
}

bool SystemRos2::loadMap() {
    bool ret = mp_slamSystem->LoadMap(ms_mapFp);
    return ret;
}

/**
 * @brief 将自定义的传感器信息转换为cv::Mat
 *
 * @param frame     输入的自定义图像帧
 * @param leftImg   输出的左图（cv::Mat）
 * @param rightImg  输出的右图（cv::Mat）
 */
void SystemRos2::convert2CvMat(sj_interfaces::msg::Frame::SharedPtr frame, cv::Mat &leftImg, cv::Mat &rightImg) {
    cv::Mat leftTmp(frame->height, frame->width, CV_8UC1, frame->left_img.data());
    cv::Mat rightTmp(frame->height, frame->width, CV_8UC1, frame->right_img.data());

    leftImg = std::move(leftTmp);
    rightImg = std::move(rightTmp);
}

/**
 * @brief 用于接收相机图像帧的回调
 *
 * @param frame 输入的图像帧
 */
void SystemRos2::grabFrame(sj_interfaces::msg::Frame::SharedPtr frame) {
    cv::Mat imLeft, imRight;
    convert2CvMat(frame, imLeft, imRight);

    if (imLeft.empty()) {
        RCLCPP_ERROR(get_logger(), "加载图片失败！");
        return;
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    auto framePose = mp_slamSystem->TrackStereo(imLeft, imRight, frame->time_stamp);

    mp_framePublisher->Update(mp_slamSystem->GetTracker());

    if (!framePose.empty()) {
        mp_mapPublisher->SetCurrentCameraPose(framePose);
    }
    if (framePose.empty())
        RCLCPP_WARN(get_logger(), "跟踪丢失！");
    else {
        publishPose3D(framePose);
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    mv_trickTime.push_back(ttrack);
}
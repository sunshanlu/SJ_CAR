#include "kitti_odom/KittiOdom.h"

size_t KittiOdom::mi_idx = 0;

/**
 * @brief KittiOdom创建帧数据的api
 *
 * @param frame 输出的帧数据
 */
void KittiOdom::create_frame(Frame &frame) {
    if (mi_idx == 4541) {
        mp_timer->cancel();
        return;
    }
    auto leftImgPath = fmt::format("{}/image_0/{:06}.png", KITTI_ODOM_PATH, mi_idx);
    auto rightImgPath = fmt::format("{}/image_1/{:06}.png", KITTI_ODOM_PATH, mi_idx++);

    auto leftCVImg = cv::imread(leftImgPath, cv::IMREAD_GRAYSCALE);
    auto righCVtImg = cv::imread(rightImgPath, cv::IMREAD_GRAYSCALE);

    for (int row = 0; row < 376; ++row) {
        for (int col = 0; col < 1241; ++col) {
            frame.left_img.push_back(leftCVImg.at<uchar>(row, col));
            frame.right_img.push_back(righCVtImg.at<uchar>(row, col));
        }
    }

    frame.channel = 1;
    frame.encoding = "mono8";
    frame.height = 376;
    frame.width = 1241;
    frame.time_stamp = now().seconds();
}

/**
 * @brief 定时器的回调函数，发送帧信息到camera/stereo
 *
 */
void KittiOdom::timerCallback() {
    Frame frame;
    create_frame(frame);
    mp_framePub->publish(frame);

    RCLCPP_INFO(get_logger(), "Publish frame %d", mi_idx);
}
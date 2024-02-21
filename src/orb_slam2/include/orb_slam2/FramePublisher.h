#ifndef FRAMEPUBLISHER_H
#define FRAMEPUBLISHER_H
#include <mutex>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "Map.h"
#include "MapPoint.h"
#include "Tracking.h"

#include "rclcpp/rclcpp.hpp"

using namespace sensor_msgs::msg;

namespace ORB_SLAM2 {

class Tracking;

class FramePublisher {

public:
    typedef std::shared_ptr<FramePublisher> SharedPtr;
    FramePublisher(rclcpp::Publisher<Image>::SharedPtr imagePub, rclcpp::Publisher<Image>::SharedPtr imagePubRight);

    void Update(Tracking *pTracker);

    void Refresh(const rclcpp::Time &stamp);

    void SetMap(Map *pMap);

protected:
    cv::Mat DrawFrame();

    void PublishFrame(const rclcpp::Time &stamp);

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    cv::Mat mIm;
    cv::Mat mImRight;
    vector<cv::KeyPoint> mvCurrentKeys;

    vector<bool> mvbOutliers;

    vector<MapPoint *> mvpMatchedMapPoints;
    int mnTracked;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mImagePub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mImagePubRight;

    int mState;

    bool mbUpdated;

    Map *mpMap;

    std::mutex mMutex;

    bool mbOnlyTracking;
};

} // namespace ORB_SLAM2

#endif // FRAMEPUBLISHER_H

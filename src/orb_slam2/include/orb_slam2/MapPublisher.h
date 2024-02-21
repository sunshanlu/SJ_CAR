#ifndef MAPPUBLISHER_H
#define MAPPUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"

using namespace visualization_msgs::msg;

namespace ORB_SLAM2 {

struct MapPublisherRcl {
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mpPub;        ///< 发布地图点
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr refMpPub;     ///< 发布参考地图点
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kfPub;        ///< 发布关键帧
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr civiGraphPub; ///< 发布共视图
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spTreePub;    ///< 发布生成树
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr curFPub;      ///< 发布当前帧
};

class MapPublisher {
public:
    typedef std::shared_ptr<MapPublisher> SharedPtr;

    MapPublisher(Map *pMap, const MapPublisherRcl &publisher, bool onlyTracking);

    Map *mpMap;

    void Refresh(const rclcpp::Time &timeStamp);
    void PublishMapPoints(const std::vector<MapPoint *> &vpMPs, const std::vector<MapPoint *> &vpRefMPs,
                          std::future<void> &mpFut, std::future<void> &refMpFut);
    void PublishKeyFrames(const std::vector<KeyFrame *> &vpKFs, std::future<void> &kfFut,
                          std::future<void> &civGraphFut, std::future<void> &mstFut);
    void PublishCurrentCamera(const cv::Mat &Tcw);
    void SetCurrentCameraPose(const cv::Mat &Tcw);

private:
    cv::Mat GetCurrentCameraPose();
    bool isCamUpdated();
    void ResetCamFlag();

    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mpPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr refMpPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kfPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr civiGraphPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spTreePub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr curFPub;
    rclcpp::Time m_timeStamp;

    Marker mPoints;
    Marker mReferencePoints;
    Marker mKeyFrames;
    Marker mReferenceKeyFrames;
    Marker mCovisibilityGraph;
    Marker mMST;
    Marker mCurrentCamera;

    float fCameraSize;
    float fPointSize;

    cv::Mat mCameraPose;
    bool mbCameraUpdated;
    bool mbOnlyTracking;

    std::mutex mMutexCamera;
};

} // namespace ORB_SLAM2

#endif // MAPPUBLISHER_H

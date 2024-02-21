#include "MapPublisher.h"
#include "KeyFrame.h"
#include "MapPoint.h"

namespace ORB_SLAM2 {

MapPublisher::MapPublisher(Map *pMap, const MapPublisherRcl &publisher, bool onlyTracking)
    : mpPub(publisher.mpPub)
    , refMpPub(publisher.refMpPub)
    , kfPub(publisher.kfPub)
    , civiGraphPub(publisher.civiGraphPub)
    , spTreePub(publisher.spTreePub)
    , curFPub(publisher.curFPub)
    , mpMap(pMap)
    , mbCameraUpdated(false)
    , mbOnlyTracking(onlyTracking) {
    const char *MAP_FRAME_ID = "map";
    const char *POINTS_NAMESPACE = "MapPoints";
    const char *KEYFRAMES_NAMESPACE = "KeyFrames";
    const char *GRAPH_NAMESPACE = "Graph";
    const char *CAMERA_NAMESPACE = "Camera";

    // Configure MapPoints
    fPointSize = 0.03;
    mPoints.header.frame_id = MAP_FRAME_ID;
    mPoints.ns = POINTS_NAMESPACE;
    mPoints.id = 0;
    mPoints.type = Marker::POINTS;
    mPoints.scale.x = fPointSize;
    mPoints.scale.y = fPointSize;
    mPoints.pose.orientation.w = 1.0;
    mPoints.action = Marker::ADD;
    mPoints.color.a = 1.0;

    // Configure KeyFrames
    fCameraSize = 1;
    mKeyFrames.header.frame_id = MAP_FRAME_ID;
    mKeyFrames.ns = KEYFRAMES_NAMESPACE;
    mKeyFrames.id = 1;
    mKeyFrames.type = Marker::LINE_LIST;
    mKeyFrames.scale.x = 0.1;
    mKeyFrames.pose.orientation.w = 1.0;
    mKeyFrames.action = Marker::ADD;

    mKeyFrames.color.b = 1.0f;
    mKeyFrames.color.a = 1.0;

    // Configure Covisibility Graph
    mCovisibilityGraph.header.frame_id = MAP_FRAME_ID;
    mCovisibilityGraph.ns = GRAPH_NAMESPACE;
    mCovisibilityGraph.id = 2;
    mCovisibilityGraph.type = Marker::LINE_LIST;
    mCovisibilityGraph.scale.x = 0.002;
    mCovisibilityGraph.pose.orientation.w = 1.0;
    mCovisibilityGraph.action = Marker::ADD;
    mCovisibilityGraph.color.b = 0.7f;
    mCovisibilityGraph.color.g = 0.7f;
    mCovisibilityGraph.color.a = 0.3;

    // Configure KeyFrames Spanning Tree
    mMST.header.frame_id = MAP_FRAME_ID;
    mMST.ns = GRAPH_NAMESPACE;
    mMST.id = 3;
    mMST.type = Marker::LINE_LIST;
    mMST.scale.x = 0.1;
    mMST.pose.orientation.w = 1.0;
    mMST.action = Marker::ADD;
    mMST.color.b = 0.0f;
    mMST.color.g = 1.0f;
    mMST.color.a = 1.0f;

    // Configure Current Camera
    mCurrentCamera.header.frame_id = MAP_FRAME_ID;
    mCurrentCamera.ns = CAMERA_NAMESPACE;
    mCurrentCamera.id = 4;
    mCurrentCamera.type = Marker::LINE_LIST;
    mCurrentCamera.scale.x = 0.1; // 0.2; 0.03
    mCurrentCamera.pose.orientation.w = 1.0;
    mCurrentCamera.action = Marker::ADD;
    mCurrentCamera.color.g = 1.0f;
    mCurrentCamera.color.a = 1.0;

    // Configure Reference MapPoints
    mReferencePoints.header.frame_id = MAP_FRAME_ID;
    mReferencePoints.ns = POINTS_NAMESPACE;
    mReferencePoints.id = 6;
    mReferencePoints.type = Marker::POINTS;
    mReferencePoints.scale.x = fPointSize;
    mReferencePoints.scale.y = fPointSize;
    mReferencePoints.pose.orientation.w = 1.0;
    mReferencePoints.action = Marker::ADD;
    mReferencePoints.color.r = 1.0f;
    mReferencePoints.color.a = 1.0;

    // Configure Publisher
    mpPub->publish(mPoints);
    refMpPub->publish(mReferencePoints);
    civiGraphPub->publish(mCovisibilityGraph);
    kfPub->publish(mKeyFrames);
    curFPub->publish(mCurrentCamera);
}

void MapPublisher::Refresh(const rclcpp::Time &timeStamp) {
    m_timeStamp = timeStamp;
    std::future<void> curFFut, mpFut, refMpFut, kfFut, civGraphFut, mstFut;
    if (isCamUpdated()) {
        cv::Mat Tcw = GetCurrentCameraPose();
        curFFut = std::async(std::bind(&MapPublisher::PublishCurrentCamera, this, Tcw));
        ResetCamFlag();
    }
    if (mpMap->IsUpdated() || mbOnlyTracking) {
        vector<KeyFrame *> vKeyFrames = mpMap->GetAllKeyFrames();
        // vector<MapPoint *> vMapPoints = mpMap->GetAllMapPoints();
        // vector<MapPoint *> vRefMapPoints = mpMap->GetReferenceMapPoints();

        // PublishMapPoints(vMapPoints, vRefMapPoints, mpFut, refMpFut);
        PublishKeyFrames(vKeyFrames, kfFut, civGraphFut, mstFut);

        if (curFFut.valid()) {
            curFFut.wait();
        }
        // mpFut.wait();
        // refMpFut.wait();
        kfFut.wait();
        // civGraphFut.wait();
        // mstFut.wait();

        mpMap->ResetUpdated();
        // RCLCPP_INFO(rclcpp::get_logger("ORB_SLAM2"), "地图发送成功！");
    }
}

void MapPublisher::PublishMapPoints(const vector<MapPoint *> &vpMPs, const vector<MapPoint *> &vpRefMPs,
                                    std::future<void> &mpFut, std::future<void> &refMpFut) {
    // mPoints.points.clear();
    mReferencePoints.points.clear();

    set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
        if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        geometry_msgs::msg::Point p;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        p.x = pos.at<float>(0);
        p.y = pos.at<float>(1);
        p.z = pos.at<float>(2);

        mPoints.points.push_back(p);
    }

    for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++) {
        if ((*sit)->isBad())
            continue;
        geometry_msgs::msg::Point p;
        cv::Mat pos = (*sit)->GetWorldPos();
        p.x = pos.at<float>(0);
        p.y = pos.at<float>(1);
        p.z = pos.at<float>(2);

        mReferencePoints.points.push_back(p);
    }

    mPoints.header.stamp = m_timeStamp;
    mReferencePoints.header.stamp = m_timeStamp;

    mpFut = std::async([this]() { mpPub->publish(mPoints); });
    refMpFut = std::async([this]() { refMpPub->publish(mReferencePoints); });

    // mpPub->publish(mPoints);
    // refMpPub->publish(mReferencePoints);
}

void MapPublisher::PublishKeyFrames(const vector<KeyFrame *> &vpKFs, std::future<void> &kfFut,
                                    std::future<void> &civGraphFut, std::future<void> &mstFut) {
    mKeyFrames.points.clear();
    // mCovisibilityGraph.points.clear();
    // mMST.points.clear();

    float d = fCameraSize;

    // Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4, 1) << d, d * 0.8, d * 0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4, 1) << d, -d * 0.8, d * 0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4, 1) << -d, -d * 0.8, d * 0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4, 1) << -d, d * 0.8, d * 0.5, 1);

    for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
        cv::Mat Tcw = vpKFs[i]->GetPose();
        cv::Mat Twc = Tcw.inv();
        cv::Mat ow = vpKFs[i]->GetCameraCenter();
        cv::Mat p1w = Twc * p1;
        cv::Mat p2w = Twc * p2;
        cv::Mat p3w = Twc * p3;
        cv::Mat p4w = Twc * p4;

        geometry_msgs::msg::Point msgs_o, msgs_p1, msgs_p2, msgs_p3, msgs_p4;
        msgs_o.x = ow.at<float>(0);
        msgs_o.y = ow.at<float>(1);
        msgs_o.z = ow.at<float>(2);
        msgs_p1.x = p1w.at<float>(0);
        msgs_p1.y = p1w.at<float>(1);
        msgs_p1.z = p1w.at<float>(2);
        msgs_p2.x = p2w.at<float>(0);
        msgs_p2.y = p2w.at<float>(1);
        msgs_p2.z = p2w.at<float>(2);
        msgs_p3.x = p3w.at<float>(0);
        msgs_p3.y = p3w.at<float>(1);
        msgs_p3.z = p3w.at<float>(2);
        msgs_p4.x = p4w.at<float>(0);
        msgs_p4.y = p4w.at<float>(1);
        msgs_p4.z = p4w.at<float>(2);

        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p1);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p1);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p1);

        //! Covisibility Graph（共视图不进行rviz可视化）
        // vector<KeyFrame *> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(5);
        // if (!vCovKFs.empty()) {
        //     for (vector<KeyFrame *>::iterator vit = vCovKFs.begin(), vend = vCovKFs.end(); vit != vend; vit++) {
        //         if ((*vit)->mnId < vpKFs[i]->mnId)
        //             continue;
        //         cv::Mat Ow2 = (*vit)->GetCameraCenter();
        //         geometry_msgs::msg::Point msgs_o2;
        //         msgs_o2.x = Ow2.at<float>(0);
        //         msgs_o2.y = Ow2.at<float>(1);
        //         msgs_o2.z = Ow2.at<float>(2);
        //         mCovisibilityGraph.points.push_back(msgs_o);
        //         mCovisibilityGraph.points.push_back(msgs_o2);
        //     }
        // }

        //! MST （生成树不进行rviz可视化）
        // KeyFrame *pParent = vpKFs[i]->GetParent();
        // if (pParent) {
        //     cv::Mat Owp = pParent->GetCameraCenter();
        //     geometry_msgs::msg::Point msgs_op;
        //     msgs_op.x = Owp.at<float>(0);
        //     msgs_op.y = Owp.at<float>(1);
        //     msgs_op.z = Owp.at<float>(2);
        //     mMST.points.push_back(msgs_o);
        //     mMST.points.push_back(msgs_op);
        // }
        // set<KeyFrame *> sLoopKFs = vpKFs[i]->GetLoopEdges();
        // for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++) {
        //     if ((*sit)->mnId < vpKFs[i]->mnId)
        //         continue;
        //     cv::Mat Owl = (*sit)->GetCameraCenter();
        //     geometry_msgs::msg::Point msgs_ol;
        //     msgs_ol.x = Owl.at<float>(0);
        //     msgs_ol.y = Owl.at<float>(1);
        //     msgs_ol.z = Owl.at<float>(2);
        //     mMST.points.push_back(msgs_o);
        //     mMST.points.push_back(msgs_ol);
        // }
    }

    mKeyFrames.header.stamp = m_timeStamp;
    // mCovisibilityGraph.header.stamp = m_timeStamp;
    // mMST.header.stamp = m_timeStamp;

    kfFut = std::async([this]() { kfPub->publish(mKeyFrames); });
    // civGraphFut = std::async([this]() { civiGraphPub->publish(mCovisibilityGraph); });
    // mstFut = std::async([this]() { spTreePub->publish(mMST); });

    // kfPub->publish(mKeyFrames);
    // civiGraphPub->publish(mCovisibilityGraph);
    // spTreePub->publish(mMST);
}

void MapPublisher::PublishCurrentCamera(const cv::Mat &Tcw) {
    mCurrentCamera.points.clear();

    float d = fCameraSize;

    // Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4, 1) << d, d * 0.8, d * 0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4, 1) << d, -d * 0.8, d * 0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4, 1) << -d, -d * 0.8, d * 0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4, 1) << -d, d * 0.8, d * 0.5, 1);

    cv::Mat Twc = Tcw.inv();
    cv::Mat ow = Twc * o;
    cv::Mat p1w = Twc * p1;
    cv::Mat p2w = Twc * p2;
    cv::Mat p3w = Twc * p3;
    cv::Mat p4w = Twc * p4;

    geometry_msgs::msg::Point msgs_o, msgs_p1, msgs_p2, msgs_p3, msgs_p4;
    msgs_o.x = ow.at<float>(0);
    msgs_o.y = ow.at<float>(1);
    msgs_o.z = ow.at<float>(2);
    msgs_p1.x = p1w.at<float>(0);
    msgs_p1.y = p1w.at<float>(1);
    msgs_p1.z = p1w.at<float>(2);
    msgs_p2.x = p2w.at<float>(0);
    msgs_p2.y = p2w.at<float>(1);
    msgs_p2.z = p2w.at<float>(2);
    msgs_p3.x = p3w.at<float>(0);
    msgs_p3.y = p3w.at<float>(1);
    msgs_p3.z = p3w.at<float>(2);
    msgs_p4.x = p4w.at<float>(0);
    msgs_p4.y = p4w.at<float>(1);
    msgs_p4.z = p4w.at<float>(2);

    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);

    mCurrentCamera.header.stamp = m_timeStamp;

    curFPub->publish(mCurrentCamera);
}

void MapPublisher::SetCurrentCameraPose(const cv::Mat &Tcw) {
    std::unique_lock<std::mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
    mbCameraUpdated = true;
}

cv::Mat MapPublisher::GetCurrentCameraPose() {
    std::unique_lock<std::mutex> lock(mMutexCamera);
    return mCameraPose.clone();
}

bool MapPublisher::isCamUpdated() {
    std::unique_lock<std::mutex> lock(mMutexCamera);
    return mbCameraUpdated;
}

void MapPublisher::ResetCamFlag() {
    std::unique_lock<std::mutex> lock(mMutexCamera);
    mbCameraUpdated = false;
}

} // namespace ORB_SLAM2

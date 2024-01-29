#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <jsoncpp/json/json.h>
#include <mutex>

#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "MapPoint.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"

namespace ORB_SLAM2 {

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame {
public:
    KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB);

    //! 添加Json对象到关键帧对象的转换
    KeyFrame(const Json::Value &kfJson, const Json::Value &commonData);

    //! 添加设置parent的api
    void setParent(KeyFrame *pParent) { mpParent = pParent; }

    //! 添加loop边
    void insertLoopEdge(KeyFrame *kf) { mspLoopEdges.insert(kf); }

    //! 添加orderedConnectKF
    void insertOrderedConnectKF(KeyFrame *kf) { mvpOrderedConnectedKeyFrames.push_back(kf); }

    //! 更新updateConnectKFandWeight
    void updateConnectKFandWeight() {
        mConnectedKeyFrameWeights.clear();
        for (std::size_t idx = 0, iend = mvpOrderedConnectedKeyFrames.size(); idx < iend; ++idx) {
            KeyFrame *connectedKF = mvpOrderedConnectedKeyFrames[idx];
            int &weight = mvOrderedWeights[idx];
            mConnectedKeyFrameWeights.insert(std::make_pair(connectedKF, weight));
        }
    }

    //! 添加获取mapPoints的api
    std::vector<MapPoint *> &getMapPoints() { return mvpMapPoints; }

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame *pKF, const int &weight);
    void EraseConnection(KeyFrame *pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame *> GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame *> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame *> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame *pKF);

    // Spanning tree functions
    void AddChild(KeyFrame *pKF);
    void EraseChild(KeyFrame *pKF);
    void ChangeParent(KeyFrame *pKF);
    std::set<KeyFrame *> GetChilds();
    KeyFrame *GetParent();
    bool hasChild(KeyFrame *pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame *pKF);
    std::set<KeyFrame *> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(MapPoint *pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint *pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP);
    std::set<MapPoint *> GetMapPoints();
    std::vector<MapPoint *> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPoint *GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    //! 添加获取要被删除的flag
    bool GetTobeErased() { return mbToBeErased; }

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp(int a, int b) { return a > b; }

    static bool lId(KeyFrame *pKF1, KeyFrame *pKF2) { return pKF1->mnId < pKF2->mnId; }

    //! 添加将关键帧数据保存为json的api
    void saveKeyFrame2Json(Json::Value &keyFrameJson);
    void saveCommonData2Json(Json::Value &commonJson);

    //! 添加地图点的api
    void setMapPoints(const std::vector<MapPoint *> &vpMPs) { mvpMapPoints = vpMPs; }

    //! 添加连接关键帧api
    void setConnectKeyFrames(const std::vector<KeyFrame *> &vpKFs) { mvpOrderedConnectedKeyFrames = vpKFs; }

    //! 生成mConnectedKeyFrameWeights
    void generateConnectKW() {
        for (int i = 0; i < mvpOrderedConnectedKeyFrames.size(); i++) {
            mConnectedKeyFrameWeights.insert(std::make_pair(mvpOrderedConnectedKeyFrames[i], mvOrderedWeights[i]));
        }
    }

    //! 添加地图api
    void setMap(Map *map) { mpMap = map; }

    //! 添加词袋api
    void setVoc(ORBVocabulary *voc) { mpORBvocabulary = voc; }

    //! 添加关键帧数据库
    void setKFDB(KeyFrameDatabase *kfdb) { mpKeyFrameDB = kfdb; }

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:
    static long unsigned int nNextId;
    long unsigned int mnId;
    long unsigned int mnFrameId;

    double mTimeStamp;

    // Grid (to speed up feature matching)
    int mnGridCols;
    int mnGridRows;
    float mfGridElementWidthInv;
    float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;
    std::vector<float> mvuRight; // negative value for monocular points
    std::vector<float> mvDepth;  // negative value for monocular points
    cv::Mat mDescriptors;

    // BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    std::vector<float> mvScaleFactors;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    int mnMinX;
    int mnMinY;
    int mnMaxX;
    int mnMaxY;
    cv::Mat mK;

    float mHalfBaseline; // Only for visualization

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:
    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<MapPoint *> mvpMapPoints;

    // BoW
    KeyFrameDatabase *mpKeyFrameDB;
    ORBVocabulary *mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector<std::vector<std::vector<size_t>>> mGrid;

    std::map<KeyFrame *, int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame *> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame *mpParent;
    std::set<KeyFrame *> mspChildrens;
    std::set<KeyFrame *> mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;

    Map *mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

} // namespace ORB_SLAM2

void pose2Json(const cv::Mat &pose, Json::Value &jsonPose);

void json2Pose(const Json::Value &jsonPose, cv::Mat &pose);
#endif // KEYFRAME_H

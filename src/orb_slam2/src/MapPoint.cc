#include <mutex>

#include "MapPoint.h"
#include "ORBmatcher.h"

namespace ORB_SLAM2 {

long unsigned int MapPoint::nNextId = 0;
mutex MapPoint::mGlobalMutex;

//! 添加Json::Value类型向MapPoint的类型转换
MapPoint::MapPoint(const Json::Value &mpJson, const Json::Value &commonData) {
    // 需要类型转换的数据
    mnId = mpJson["mnId"].asUInt64();
    mnFirstKFid = mpJson["mnFirstKFid"].asInt64();
    mnFirstFrame = mpJson["mnFirstFrame"].asInt64();
    mnTrackReferenceForFrame = mpJson["mnTrackReferenceForFrame"].asUInt64();
    mnLastFrameSeen = mpJson["mnLastFrameSeen"].asUInt64();
    mnBALocalForKF = mpJson["mnBALocalForKF"].asUInt64();
    mnFuseCandidateForKF = mpJson["mnFuseCandidateForKF"].asUInt64();
    mnLoopPointForKF = mpJson["mnLoopPointForKF"].asUInt64();
    mnCorrectedByKF = mpJson["mnCorrectedByKF"].asUInt64();
    mnCorrectedReference = mpJson["mnCorrectedReference"].asUInt64();
    mnBAGlobalForKF = mpJson["mnBAGlobalForKF"].asUInt64();

    // int类型的数据
    //! 这里不使用这个方式判断的原因是在后面添加连续的时候，会进行观测数据量的更新
    // nObs = mpJson["nObs"].asInt();
    mnFound = mpJson["mnFound"].asInt();
    mnVisible = mpJson["mnVisible"].asInt();

    // bool类型的数据
    mbBad = mpJson["mbBad"].asBool();
    mbTrackInView = mpJson["mbTrackInView"].asBool();

    // float类型的数据
    mTrackProjX = mpJson["mTrackProjX"].asFloat();
    mTrackProjY = mpJson["mTrackProjY"].asFloat();
    mTrackProjXR = mpJson["mTrackProjXR"].asFloat();
    mTrackViewCos = mpJson["mTrackViewCos"].asFloat();
    mfMinDistance = mpJson["mfMinDistance"].asFloat();
    mfMaxDistance = mpJson["mfMaxDistance"].asFloat();
    mnTrackScaleLevel = mpJson["mnTrackScaleLevel"].asFloat();

    // 容器类型的数据
    cv::Mat jmWorldPos(3, 1, CV_32F), jmNormalVector(3, 1, CV_32F), jmDescriptor(1, 32, CV_8U);
    int cntPose = 0;

    json2Pose(mpJson["mPosGBA"], mPosGBA);

    for (int i = 0; i < 3; ++i) {
        jmWorldPos.at<float>(i, 0) = mpJson["mWorldPos"][i].asFloat();
        jmNormalVector.at<float>(i, 0) = mpJson["mNormalVector"][i].asFloat();
    }
    for (int i = 0; i < 32; ++i) {
        jmDescriptor.at<uchar>(0, i) = (uchar)mpJson["mDescriptor"][i].asUInt();
    }
    jmWorldPos.copyTo(mWorldPos);
    jmNormalVector.copyTo(mNormalVector);
    jmDescriptor.copyTo(mDescriptor);
}

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map *pMap)
    : mnFirstKFid(pRefKF->mnId)
    , mnFirstFrame(pRefKF->mnFrameId)
    , nObs(0)
    , mnTrackReferenceForFrame(0)
    , mnLastFrameSeen(0)
    , mnBALocalForKF(0)
    , mnFuseCandidateForKF(0)
    , mnLoopPointForKF(0)
    , mnCorrectedByKF(0)
    , mnCorrectedReference(0)
    , mnBAGlobalForKF(0)
    , mpRefKF(pRefKF)
    , mnVisible(1)
    , mnFound(1)
    , mbBad(false)
    , mpReplaced(static_cast<MapPoint *>(NULL))
    , mfMinDistance(0)
    , mfMaxDistance(0)
    , mpMap(pMap) {
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3, 1, CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId = nNextId++;
}

MapPoint::MapPoint(const cv::Mat &Pos, Map *pMap, Frame *pFrame, const int &idxF)
    : mnFirstKFid(-1)
    , mnFirstFrame(pFrame->mnId)
    , nObs(0)
    , mnTrackReferenceForFrame(0)
    , mnLastFrameSeen(0)
    , mnBALocalForKF(0)
    , mnFuseCandidateForKF(0)
    , mnLoopPointForKF(0)
    , mnCorrectedByKF(0)
    , mnCorrectedReference(0)
    , mnBAGlobalForKF(0)
    , mpRefKF(static_cast<KeyFrame *>(NULL))
    , mnVisible(1)
    , mnFound(1)
    , mbBad(false)
    , mpReplaced(NULL)
    , mpMap(pMap) {
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector / cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor = pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist * levelScaleFactor;
    mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId = nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos) {
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos() {
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal() {
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

KeyFrame *MapPoint::GetReferenceKeyFrame() {
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame *pKF, size_t idx) {
    unique_lock<mutex> lock(mMutexFeatures);
    if (mObservations.count(pKF))
        return;
    mObservations[pKF] = idx;

    if (pKF->mvuRight[idx] >= 0)
        nObs += 2;
    else
        nObs++;
}

void MapPoint::EraseObservation(KeyFrame *pKF) {
    bool bBad = false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF)) {
            int idx = mObservations[pKF];
            if (pKF->mvuRight[idx] >= 0)
                nObs -= 2;
            else
                nObs--;

            mObservations.erase(pKF);

            if (mpRefKF == pKF)
                mpRefKF = mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if (nObs <= 2)
                bBad = true;
        }
    }

    if (bBad)
        SetBadFlag();
}

map<KeyFrame *, size_t> MapPoint::GetObservations() {
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations() {
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag() {
    map<KeyFrame *, size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad = true;
        obs = mObservations;
        mObservations.clear();
    }
    for (map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
        KeyFrame *pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }

    mpMap->EraseMapPoint(this);
}

MapPoint *MapPoint::GetReplaced() {
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPoint *pMP) {
    if (pMP->mnId == this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame *, size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs = mObservations;
        mObservations.clear();
        mbBad = true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for (map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
        // Replace measurement in keyframe
        KeyFrame *pKF = mit->first;

        if (!pMP->IsInKeyFrame(pKF)) {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF, mit->second);
        } else {
            pKF->EraseMapPointMatch(mit->second);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad() {
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n) {
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible += n;
}

void MapPoint::IncreaseFound(int n) {
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound += n;
}

float MapPoint::GetFoundRatio() {
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound) / mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors() {
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame *, size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if (mbBad)
            return;
        observations = mObservations;
    }

    if (observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
        KeyFrame *pKF = mit->first;

        if (!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if (vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for (size_t i = 0; i < N; i++) {
        Distances[i][i] = 0;
        for (size_t j = i + 1; j < N; j++) {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
            Distances[i][j] = distij;
            Distances[j][i] = distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for (size_t i = 0; i < N; i++) {
        vector<int> vDists(Distances[i], Distances[i] + N);
        sort(vDists.begin(), vDists.end());
        int median = vDists[0.5 * (N - 1)];

        if (median < BestMedian) {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor() {
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF) {
    unique_lock<mutex> lock(mMutexFeatures);
    if (mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF) {
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth() {
    map<KeyFrame *, size_t> observations;
    KeyFrame *pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if (mbBad)
            return;
        observations = mObservations;
        pRefKF = mpRefKF;
        Pos = mWorldPos.clone();
    }

    if (observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
    int n = 0;
    for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
        KeyFrame *pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali / cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor = pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist * levelScaleFactor;
        mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
        mNormalVector = normal / n;
    }
}

float MapPoint::GetMinDistanceInvariance() {
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f * mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance() {
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f * mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, KeyFrame *pKF) {
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance / currentDist;
    }

    int nScale = ceil(log(ratio) / pKF->mfLogScaleFactor);
    if (nScale < 0)
        nScale = 0;
    else if (nScale >= pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels - 1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame *pF) {
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance / currentDist;
    }

    int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);
    if (nScale < 0)
        nScale = 0;
    else if (nScale >= pF->mnScaleLevels)
        nScale = pF->mnScaleLevels - 1;

    return nScale;
}

//! 保存地图点到JSON
void MapPoint::saveMapPoint2json(Json::Value &mapPointJson) {
    mapPointJson["mnId"] = (unsigned long long)mnId;
    mapPointJson["mnFirstKFid"] = (unsigned long long)mnFirstKFid;
    mapPointJson["mnFirstFrame"] = (unsigned long long)mnFirstFrame;
    mapPointJson["mnTrackReferenceForFrame"] = (unsigned long long)mnTrackReferenceForFrame;
    mapPointJson["mnLastFrameSeen"] = (unsigned long long)mnLastFrameSeen;
    mapPointJson["mnBALocalForKF"] = (unsigned long long)mnBALocalForKF;
    mapPointJson["mnFuseCandidateForKF"] = (unsigned long long)mnFuseCandidateForKF;
    mapPointJson["mnLoopPointForKF"] = (unsigned long long)mnLoopPointForKF;
    mapPointJson["mnCorrectedByKF"] = (unsigned long long)mnCorrectedByKF;
    mapPointJson["mnCorrectedReference"] = (unsigned long long)mnCorrectedReference;
    mapPointJson["mnBAGlobalForKF"] = (unsigned long long)mnBAGlobalForKF;

    mapPointJson["nObs"] = nObs;
    mapPointJson["mTrackProjX"] = mTrackProjX;
    mapPointJson["mTrackProjY"] = mTrackProjY;
    mapPointJson["mTrackProjXR"] = mTrackProjXR;
    mapPointJson["mbTrackInView"] = mbTrackInView;
    mapPointJson["mnTrackScaleLevel"] = mnTrackScaleLevel;
    mapPointJson["mTrackViewCos"] = mTrackViewCos;
    mapPointJson["mnFound"] = mnFound;
    mapPointJson["mbBad"] = mbBad;
    mapPointJson["mnVisible"] = mnVisible;
    mapPointJson["mfMinDistance"] = mfMinDistance;
    mapPointJson["mfMaxDistance"] = mfMaxDistance;

    // 指针类型
    if (mpRefKF && mpRefKF->isBad() == false)
        mapPointJson["mpRefKF"] = (unsigned long long)mpRefKF->mnId;
    else
        mapPointJson["mpRefKF"] = Json::Value(Json::nullValue);

    if (mpReplaced && mpReplaced->isBad() == false)
        mapPointJson["mpReplaced"] = (unsigned long long)mpReplaced->mnId;
    else
        mapPointJson["mpReplaced"] = Json::Value(Json::nullValue);

    // 容器类型(mObservations包含指针)
    // Json::Value mObservations_json(Json::arrayValue);
    Json::Value mPosGBA_json(Json::arrayValue);
    Json::Value mWorldPos_json(Json::arrayValue);
    Json::Value mNormalVector_json(Json::arrayValue);
    Json::Value mDescriptor_json(Json::arrayValue);

    // for (auto &item : mObservations) {
    //     if (item.first && item.first->isBad() == false) {
    //         Json::Value observation;
    //         observation["keyframe_id"] = (unsigned long long)item.first->mnId;
    //         observation["feature_id"] = (unsigned long long)item.second;
    //         mObservations_json.append(observation);
    //     }
    // }

    if (!mPosGBA.empty()) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                mPosGBA_json.append(mPosGBA.at<float>(i, j));
            }
        }
    }

    if (!mWorldPos.empty()) {
        for (int i = 0; i < 3; ++i) {
            mWorldPos_json.append(mWorldPos.at<float>(i, 0));
        }
    }

    if (!mNormalVector.empty()) {
        for (int i = 0; i < 3; ++i) {
            mNormalVector_json.append(mNormalVector.at<float>(i, 0));
        }
    }

    if (!mDescriptor.empty()) {
        for (int i = 0; i < 32; ++i) {
            mDescriptor_json.append(mDescriptor.at<uchar>(0, i));
        }
    }

    // mapPointJson["mObservations"] = mObservations_json;
    mapPointJson["mPosGBA"] = mPosGBA_json;
    mapPointJson["mWorldPos"] = mWorldPos_json;
    mapPointJson["mNormalVector"] = mNormalVector_json;
    mapPointJson["mDescriptor"] = mDescriptor_json;
}

//! 保存地图点公共数据到JSON
void MapPoint::saveCommonData2Json(Json::Value &commonJson) { commonJson["nNextId"] = (unsigned long long)nNextId; }

} // namespace ORB_SLAM2

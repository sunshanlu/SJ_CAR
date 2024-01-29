#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "Converter.h"
#include "KeyFrame.h"
#include "ORBmatcher.h"

namespace ORB_SLAM2 {

long unsigned int KeyFrame::nNextId = 0;

//! 添加关键帧的构造函数
KeyFrame::KeyFrame(const Json::Value &kfJson, const Json::Value &commonData) {

    // common data
    const Json::Value &commonKfData = commonData["kfCommon"];
    cx = commonKfData["cx"].asFloat();
    cy = commonKfData["cy"].asFloat();
    fx = commonKfData["fx"].asFloat();
    fy = commonKfData["fy"].asFloat();

    invfx = commonKfData["invfx"].asFloat();
    invfy = commonKfData["invfy"].asFloat();
    mHalfBaseline = commonKfData["mHalfBaseline"].asFloat();
    mThDepth = commonKfData["mThDepth"].asFloat();
    mb = commonKfData["mb"].asFloat();
    mbf = commonKfData["mbf"].asFloat();
    mfGridElementHeightInv = commonKfData["mfGridElementHeightInv"].asFloat();
    mfGridElementWidthInv = commonKfData["mfGridElementWidthInv"].asFloat();
    mfLogScaleFactor = commonKfData["mfLogScaleFactor"].asFloat();
    mfScaleFactor = commonKfData["mfScaleFactor"].asFloat();
    mnGridCols = commonKfData["mnGridCols"].asInt();
    mnGridRows = commonKfData["mnGridRows"].asInt();
    mnMaxX = commonKfData["mnMaxX"].asInt();
    mnMaxY = commonKfData["mnMaxY"].asInt();
    mnMinX = commonKfData["mnMinX"].asInt();
    mnMinY = commonKfData["mnMinY"].asInt();
    mnScaleLevels = commonKfData["mnScaleLevels"].asInt();

    cv::Mat matrixK(3, 3, CV_32F);
    int s = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            matrixK.at<float>(i, j) = commonKfData["mK"][s].asFloat();
            ++s;
        }
    }
    matrixK.copyTo(mK);

    for (auto begin = commonKfData["mvInvLevelSigma2"].begin(); begin != commonKfData["mvInvLevelSigma2"].end();
         ++begin) {
        mvInvLevelSigma2.push_back(begin->asFloat());
    }

    for (auto begin = commonKfData["mvLevelSigma2"].begin(); begin != commonKfData["mvLevelSigma2"].end(); ++begin) {
        mvLevelSigma2.push_back(begin->asFloat());
    }

    for (auto begin = commonKfData["mvScaleFactors"].begin(); begin != commonKfData["mvScaleFactors"].end(); ++begin) {
        mvScaleFactors.push_back(begin->asFloat());
    }

    // 需要将unsigned long long转换为long unsigned int
    mnId = kfJson["mnId"].asUInt64();
    mnFrameId = kfJson["mnFrameId"].asUInt64();
    mnTrackReferenceForFrame = kfJson["mnTrackReferenceForFrame"].asUInt64();
    mnFuseTargetForKF = kfJson["mnFuseTargetForKF"].asUInt64();
    mnBALocalForKF = kfJson["mnBALocalForKF"].asUInt64();
    mnBAFixedForKF = kfJson["mnBAFixedForKF"].asUInt64();
    mnLoopQuery = kfJson["mnLoopQuery"].asUInt64();
    mnRelocQuery = kfJson["mnRelocQuery"].asUInt64();
    mnBAGlobalForKF = kfJson["mnBAGlobalForKF"].asUInt64();

    // 可直接赋值型
    mnLoopWords = kfJson["mnLoopWords"].asInt();
    mTimeStamp = kfJson["mTimeStamp"].asDouble();
    mLoopScore = kfJson["mLoopScore"].asFloat();
    mnRelocWords = kfJson["mnRelocWords"].asInt();
    mRelocScore = kfJson["mRelocScore"].asFloat();
    mbFirstConnection = kfJson["mbFirstConnection"].asBool();
    mbNotErase = kfJson["mbNotErase"].asBool();
    mbToBeErased = kfJson["mbToBeErased"].asBool();
    mbBad = kfJson["mbBad"].asBool();
    N = kfJson["N"].asInt();

    // cv::Mat部分，矩阵和向量
    json2Pose(kfJson["mTcwGBA"], mTcwGBA);
    json2Pose(kfJson["mTcwBefGBA"], mTcwBefGBA);
    json2Pose(kfJson["mTcp"], mTcp);
    json2Pose(kfJson["Tcw"], Tcw);
    json2Pose(kfJson["Twc"], Twc);

    cv::Mat tempOw(4, 4, CV_32F), tempCw(4, 4, CV_32F);
    for (int i = 0; i < 3; ++i) {
        tempOw.at<float>(i, 0) = kfJson["Ow"][i].asFloat();
        tempCw.at<float>(i, 0) = kfJson["Cw"][i].asFloat();
    }
    Ow = std::move(tempOw);
    Cw = std::move(tempCw);

    // 容器部分
    cv::Mat desc(kfJson["mvKeysUn"].size(), 32, CV_8U);
    for (int ibeg = 0, iend = kfJson["mvKeysUn"].size(); ibeg < iend; ++ibeg) {
        auto octave = kfJson["mvKeysUn"][ibeg]["octave"].asInt();
        auto x = kfJson["mvKeysUn"][ibeg]["x"].asFloat();
        auto y = kfJson["mvKeysUn"][ibeg]["y"].asFloat();
        auto angle = kfJson["mvKeysUn"][ibeg]["angle"].asFloat();
        mvKeys.emplace_back(x, y, 1.0, angle, 1.0, octave);
        mvKeysUn.emplace_back(x, y, 1.0, angle, 1.0, octave);

        for (int i = 0; i < 32; ++i) {
            desc.at<uchar>(ibeg, i) = (uchar)kfJson["mDescriptors"][ibeg][i].asUInt();
        }
        mBowVec.insert(std::make_pair<DBoW2::WordId, DBoW2::WordValue>(
            kfJson["mBowVec"][ibeg]["WordId"].asUInt(), kfJson["mBowVec"][ibeg]["WordValue"].asDouble()));
    }
    desc.copyTo(mDescriptors);

    for (int ibeg = 0, iend = kfJson["mFeatVec"].size(); ibeg < iend; ++ibeg) {
        auto &featJson = kfJson["mFeatVec"][ibeg];
        auto nodeid = featJson["NodeId"].asUInt();
        std::vector<unsigned> feats;
        for (int i = 0; i < featJson["FeatureIds"].size(); ++i) {
            feats.push_back(featJson["FeatureIds"][i].asUInt());
        }
        mFeatVec.insert(std::make_pair(nodeid, feats));
    }
    for (int ibeg = 0, iend = kfJson["mvOrderedWeights"].size(); ibeg < iend; ++ibeg)
        mvOrderedWeights.push_back(kfJson["mvOrderedWeights"][ibeg].asInt());

    for (auto begin = kfJson["mvuRight"].begin(), end = kfJson["mvuRight"].end(); begin != end; ++begin) {
        mvuRight.push_back(begin->asFloat());
    }

    for (auto begin = kfJson["mvDepth"].begin(), end = kfJson["mvDepth"].end(); begin != end; ++begin) {
        mvDepth.push_back(begin->asFloat());
    }
}

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB)
    : mnFrameId(F.mnId)
    , mTimeStamp(F.mTimeStamp)
    , mnGridCols(FRAME_GRID_COLS)
    , mnGridRows(FRAME_GRID_ROWS)
    , mfGridElementWidthInv(F.mfGridElementWidthInv)
    , mfGridElementHeightInv(F.mfGridElementHeightInv)
    , mnTrackReferenceForFrame(0)
    , mnFuseTargetForKF(0)
    , mnBALocalForKF(0)
    , mnBAFixedForKF(0)
    , mnLoopQuery(0)
    , mnLoopWords(0)
    , mnRelocQuery(0)
    , mnRelocWords(0)
    , mnBAGlobalForKF(0)
    , fx(F.fx)
    , fy(F.fy)
    , cx(F.cx)
    , cy(F.cy)
    , invfx(F.invfx)
    , invfy(F.invfy)
    , mbf(F.mbf)
    , mb(F.mb)
    , mThDepth(F.mThDepth)
    , N(F.N)
    , mvKeys(F.mvKeys)
    , mvKeysUn(F.mvKeysUn)
    , mvuRight(F.mvuRight)
    , mvDepth(F.mvDepth)
    , mDescriptors(F.mDescriptors.clone())
    , mBowVec(F.mBowVec)
    , mFeatVec(F.mFeatVec)
    , mnScaleLevels(F.mnScaleLevels)
    , mfScaleFactor(F.mfScaleFactor)
    , mfLogScaleFactor(F.mfLogScaleFactor)
    , mvScaleFactors(F.mvScaleFactors)
    , mvLevelSigma2(F.mvLevelSigma2)
    , mvInvLevelSigma2(F.mvInvLevelSigma2)
    , mnMinX(F.mnMinX)
    , mnMinY(F.mnMinY)
    , mnMaxX(F.mnMaxX)
    , mnMaxY(F.mnMaxY)
    , mK(F.mK)
    , mvpMapPoints(F.mvpMapPoints)
    , mpKeyFrameDB(pKFDB)
    , mpORBvocabulary(F.mpORBvocabulary)
    , mbFirstConnection(true)
    , mpParent(NULL)
    , mbNotErase(false)
    , mbToBeErased(false)
    , mbBad(false)
    , mHalfBaseline(F.mb / 2)
    , mpMap(pMap) {
    mnId = nNextId++;

    mGrid.resize(mnGridCols);
    for (int i = 0; i < mnGridCols; i++) {
        mGrid[i].resize(mnGridRows);
        for (int j = 0; j < mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);
}

void KeyFrame::ComputeBoW() {
    if (mBowVec.empty() || mFeatVec.empty()) {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
    }
}

void KeyFrame::SetPose(const cv::Mat &Tcw_) {
    unique_lock<mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc * tcw;

    Twc = cv::Mat::eye(4, 4, Tcw.type());
    Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
    Ow.copyTo(Twc.rowRange(0, 3).col(3));
    cv::Mat center = (cv::Mat_<float>(4, 1) << mHalfBaseline, 0, 0, 1);
    Cw = Twc * center;
}

cv::Mat KeyFrame::GetPose() {
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse() {
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter() {
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter() {
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}

cv::Mat KeyFrame::GetRotation() {
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0, 3).colRange(0, 3).clone();
}

cv::Mat KeyFrame::GetTranslation() {
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0, 3).col(3).clone();
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight) {
    {
        unique_lock<mutex> lock(mMutexConnections);
        if (!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF] = weight;
        else if (mConnectedKeyFrameWeights[pKF] != weight)
            mConnectedKeyFrameWeights[pKF] = weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles() {
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int, KeyFrame *>> vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end();
         mit != mend; mit++)
        vPairs.push_back(make_pair(mit->second, mit->first));

    sort(vPairs.begin(), vPairs.end());
    list<KeyFrame *> lKFs;
    list<int> lWs;
    for (size_t i = 0, iend = vPairs.size(); i < iend; i++) {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

set<KeyFrame *> KeyFrame::GetConnectedKeyFrames() {
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame *> s;
    for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(); mit != mConnectedKeyFrameWeights.end();
         mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame *> KeyFrame::GetVectorCovisibleKeyFrames() {
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame *> KeyFrame::GetBestCovisibilityKeyFrames(const int &N) {
    unique_lock<mutex> lock(mMutexConnections);
    if ((int)mvpOrderedConnectedKeyFrames.size() < N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + N);
}

vector<KeyFrame *> KeyFrame::GetCovisiblesByWeight(const int &w) {
    unique_lock<mutex> lock(mMutexConnections);

    if (mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame *>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w, KeyFrame::weightComp);
    if (it == mvOrderedWeights.end())
        return vector<KeyFrame *>();
    else {
        int n = it - mvOrderedWeights.begin();
        return vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + n);
    }
}

int KeyFrame::GetWeight(KeyFrame *pKF) {
    unique_lock<mutex> lock(mMutexConnections);
    if (mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx) {
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx] = pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx) {
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint *pMP) {
    int idx = pMP->GetIndexInKeyFrame(this);
    if (idx >= 0)
        mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
}

void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP) { mvpMapPoints[idx] = pMP; }

set<MapPoint *> KeyFrame::GetMapPoints() {
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint *> s;
    for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++) {
        if (!mvpMapPoints[i])
            continue;
        MapPoint *pMP = mvpMapPoints[i];
        if (!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs) {
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints = 0;
    const bool bCheckObs = minObs > 0;
    for (int i = 0; i < N; i++) {
        MapPoint *pMP = mvpMapPoints[i];
        if (pMP) {
            if (!pMP->isBad()) {
                if (bCheckObs) {
                    if (mvpMapPoints[i]->Observations() >= minObs)
                        nPoints++;
                } else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint *> KeyFrame::GetMapPointMatches() {
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPoint *KeyFrame::GetMapPoint(const size_t &idx) {
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections() {
    map<KeyFrame *, int> KFcounter;

    vector<MapPoint *> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    // For all map points in keyframe check in which other keyframes are they seen
    // Increase counter for those keyframes
    for (vector<MapPoint *>::iterator vit = vpMP.begin(), vend = vpMP.end(); vit != vend; vit++) {
        MapPoint *pMP = *vit;

        if (!pMP)
            continue;

        if (pMP->isBad())
            continue;

        map<KeyFrame *, size_t> observations = pMP->GetObservations();

        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend;
             mit++) {
            if (mit->first->mnId == mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if (KFcounter.empty())
        return;

    // If the counter is greater than threshold add connection
    // In case no keyframe counter is over threshold add the one with maximum counter
    int nmax = 0;
    KeyFrame *pKFmax = NULL;
    int th = 15;

    vector<pair<int, KeyFrame *>> vPairs;
    vPairs.reserve(KFcounter.size());
    for (map<KeyFrame *, int>::iterator mit = KFcounter.begin(), mend = KFcounter.end(); mit != mend; mit++) {
        if (mit->second > nmax) {
            nmax = mit->second;
            pKFmax = mit->first;
        }
        if (mit->second >= th) {
            vPairs.push_back(make_pair(mit->second, mit->first));
            (mit->first)->AddConnection(this, mit->second);
        }
    }

    if (vPairs.empty()) {
        vPairs.push_back(make_pair(nmax, pKFmax));
        pKFmax->AddConnection(this, nmax);
    }

    sort(vPairs.begin(), vPairs.end());
    list<KeyFrame *> lKFs;
    list<int> lWs;
    for (size_t i = 0; i < vPairs.size(); i++) {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if (mbFirstConnection && mnId != 0) {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }
    }
}

void KeyFrame::AddChild(KeyFrame *pKF) {
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF) {
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF) {
    unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(this);
}

set<KeyFrame *> KeyFrame::GetChilds() {
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame *KeyFrame::GetParent() {
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF) {
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF) {
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame *> KeyFrame::GetLoopEdges() {
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase() {
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase() {
    {
        unique_lock<mutex> lock(mMutexConnections);
        if (mspLoopEdges.empty()) {
            mbNotErase = false;
        }
    }

    if (mbToBeErased) {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag() {
    {
        unique_lock<mutex> lock(mMutexConnections);
        if (mnId == 0)
            return;
        else if (mbNotErase) {
            mbToBeErased = true;
            return;
        }
    }

    for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end();
         mit != mend; mit++)
        mit->first->EraseConnection(this);

    for (size_t i = 0; i < mvpMapPoints.size(); i++)
        if (mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(this);
    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<KeyFrame *> sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while (!mspChildrens.empty()) {
            bool bContinue = false;

            int max = -1;
            KeyFrame *pC;
            KeyFrame *pP;

            for (set<KeyFrame *>::iterator sit = mspChildrens.begin(), send = mspChildrens.end(); sit != send; sit++) {
                KeyFrame *pKF = *sit;
                if (pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<KeyFrame *> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for (size_t i = 0, iend = vpConnected.size(); i < iend; i++) {
                    for (set<KeyFrame *>::iterator spcit = sParentCandidates.begin(), spcend = sParentCandidates.end();
                         spcit != spcend; spcit++) {
                        if (vpConnected[i]->mnId == (*spcit)->mnId) {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if (w > max) {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if (bContinue) {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            } else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if (!mspChildrens.empty())
            for (set<KeyFrame *>::iterator sit = mspChildrens.begin(); sit != mspChildrens.end(); sit++) {
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(this);
        mTcp = Tcw * mpParent->GetPoseInverse();
        mbBad = true;
    }

    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
}

bool KeyFrame::isBad() {
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame *pKF) {
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if (mConnectedKeyFrameWeights.count(pKF)) {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate = true;
        }
    }

    if (bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const {
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0, (int)floor((x - mnMinX - r) * mfGridElementWidthInv));
    if (nMinCellX >= mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols - 1, (int)ceil((x - mnMinX + r) * mfGridElementWidthInv));
    if (nMaxCellX < 0)
        return vIndices;

    const int nMinCellY = max(0, (int)floor((y - mnMinY - r) * mfGridElementHeightInv));
    if (nMinCellY >= mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows - 1, (int)ceil((y - mnMinY + r) * mfGridElementHeightInv));
    if (nMaxCellY < 0)
        return vIndices;

    for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
        for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
            const vector<size_t> vCell = mGrid[ix][iy];
            for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x - x;
                const float disty = kpUn.pt.y - y;

                if (fabs(distx) < r && fabs(disty) < r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const {
    return (x >= mnMinX && x < mnMaxX && y >= mnMinY && y < mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(int i) {
    const float z = mvDepth[i];
    if (z > 0) {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u - cx) * z * invfx;
        const float y = (v - cy) * z * invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        return Twc.rowRange(0, 3).colRange(0, 3) * x3Dc + Twc.rowRange(0, 3).col(3);
    } else
        return cv::Mat();
}

float KeyFrame::ComputeSceneMedianDepth(const int q) {
    vector<MapPoint *> vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0, 3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2, 3);
    for (int i = 0; i < N; i++) {
        if (mvpMapPoints[i]) {
            MapPoint *pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw) + zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(), vDepths.end());

    return vDepths[(vDepths.size() - 1) / q];
}

//! 添加将关键帧数据保存为json的api
void KeyFrame::saveKeyFrame2Json(Json::Value &keyFrameJson) {
    keyFrameJson["mnId"] = (unsigned long long)mnId;
    keyFrameJson["mnFrameId"] = (unsigned long long)mnFrameId;
    keyFrameJson["mTimeStamp"] = mTimeStamp;
    keyFrameJson["mnTrackReferenceForFrame"] = (unsigned long long)mnTrackReferenceForFrame;
    keyFrameJson["mnFuseTargetForKF"] = (unsigned long long)mnFuseTargetForKF;
    keyFrameJson["mnBALocalForKF"] = (unsigned long long)mnBALocalForKF;
    keyFrameJson["mnBAFixedForKF"] = (unsigned long long)mnBAFixedForKF;
    keyFrameJson["mnLoopQuery"] = (unsigned long long)mnLoopQuery;
    keyFrameJson["mnLoopWords"] = mnLoopWords;
    keyFrameJson["mLoopScore"] = mLoopScore;
    keyFrameJson["mnRelocQuery"] = (unsigned long long)mnRelocQuery;
    keyFrameJson["mnRelocWords"] = mnRelocWords;
    keyFrameJson["mRelocScore"] = mRelocScore;
    keyFrameJson["mnBAGlobalForKF"] = (unsigned long long)mnBAGlobalForKF;
    keyFrameJson["N"] = N;
    keyFrameJson["mbBad"] = mbBad;
    keyFrameJson["mbToBeErased"] = mbToBeErased;
    keyFrameJson["mbNotErase"] = mbNotErase;
    keyFrameJson["mbFirstConnection"] = mbFirstConnection;

    Json::Value mTcwGBA_json, mTcwBefGBA_json, mTcp_json, Tcw_json, Twc_json, Ow_json, Cw_json;

    pose2Json(mTcwGBA, mTcwGBA_json);
    pose2Json(mTcwBefGBA, mTcwBefGBA_json);
    pose2Json(mTcp, mTcp_json);
    pose2Json(Tcw, Tcw_json);
    pose2Json(Twc, Twc_json);

    for (int i = 0; i < 3; ++i) {
        Ow_json.append(Ow.at<float>(i, 0));
        Cw_json.append(Cw.at<float>(i, 0));
    }

    keyFrameJson["mTcwGBA"] = mTcwGBA_json;
    keyFrameJson["mTcwBefGBA"] = mTcwBefGBA_json;
    keyFrameJson["mTcp"] = mTcp_json;
    keyFrameJson["Tcw"] = Tcw_json;
    keyFrameJson["Twc"] = Twc_json;
    keyFrameJson["Ow"] = Ow_json;
    keyFrameJson["Cw"] = Cw_json;

    // Json::Value mvKeys_json, mvKeysUn_json, mvpMapPoints_json, mDescriptors_json, mvuRight_json, mvDepth_json;
    Json::Value mvKeysUn_json, mvpMapPoints_json, mDescriptors_json, mvuRight_json, mvDepth_json;
    for (int ibegin = 0, iend = mvKeys.size(); ibegin < iend; ++ibegin) {
        auto keypoint = mvKeys[ibegin];
        auto keypointUn = mvKeysUn[ibegin];
        auto mapPoint = mvpMapPoints[ibegin];

        // Json::Value keypoint_json, keypointUn_json, desc_json;
        Json::Value keypointUn_json, desc_json;
        // keypoint_json["x"] = keypoint.pt.x;
        // keypoint_json["y"] = keypoint.pt.y;
        // keypoint_json["octave"] = keypoint.octave;
        // keypoint_json["angle"] = keypoint.angle;

        keypointUn_json["x"] = keypointUn.pt.x;
        keypointUn_json["y"] = keypointUn.pt.y;
        keypointUn_json["octave"] = keypointUn.octave;
        keypointUn_json["angle"] = keypointUn.angle;
        for (int i = 0; i < 32; ++i) {
            desc_json.append(mDescriptors.at<uchar>(ibegin, i));
        }

        // mvKeys_json.append(keypoint_json);
        mvKeysUn_json.append(keypointUn_json);
        mDescriptors_json.append(desc_json);
        mvuRight_json.append(mvuRight[ibegin]);
        mvDepth_json.append(mvDepth[ibegin]);
        if (mapPoint)
            mvpMapPoints_json.append((long long)mapPoint->mnId);
        else
            mvpMapPoints_json.append(-1);
    }

    // keyFrameJson["mvKeys"] = mvKeys_json;
    keyFrameJson["mvKeysUn"] = mvKeysUn_json;
    keyFrameJson["mvpMapPoints"] = mvpMapPoints_json;
    keyFrameJson["mDescriptors"] = mDescriptors_json;
    keyFrameJson["mvuRight"] = mvuRight_json;
    keyFrameJson["mvDepth"] = mvDepth_json;

    Json::Value mBowVec_json;
    for (auto keyFrameJson : mBowVec) {
        Json::Value mBowVec_item;
        mBowVec_item["WordId"] = keyFrameJson.first;
        mBowVec_item["WordValue"] = keyFrameJson.second;
        mBowVec_json.append(mBowVec_item);
    }

    // NodeId, std::vector<unsigned int>
    Json::Value mFeatVec_json;
    for (auto keyFrameJson : mFeatVec) {
        Json::Value mFeatVec_item, featureIds;
        mFeatVec_item["NodeId"] = keyFrameJson.first;
        for (auto featureId : keyFrameJson.second) {
            featureIds.append(featureId);
        }
        mFeatVec_item["FeatureIds"] = featureIds;
        mFeatVec_json.append(mFeatVec_item);
    }
    keyFrameJson["mFeatVec"] = mFeatVec_json;
    keyFrameJson["mBowVec"] = mBowVec_json;

    // mConnectedKeyFrameWeights可以由排序好的mvpOrderedConnectedKeyFrames和mvOrderedWeights得出
    Json::Value mvpOrderedConnectedKeyFrames_json, mvOrderedWeights_json;
    mvpOrderedConnectedKeyFrames_json.resize(0);
    mvOrderedWeights_json.resize(0);
    for (int ibegin = 0, iend = mvpOrderedConnectedKeyFrames.size(); ibegin < iend; ++ibegin) {
        auto kf = mvpOrderedConnectedKeyFrames[ibegin];
        auto weight = mvOrderedWeights[ibegin];
        mvpOrderedConnectedKeyFrames_json.append((unsigned long long)kf->mnId);
        mvOrderedWeights_json.append(weight);
    }
    keyFrameJson["mvpOrderedConnectedKeyFrames"] = mvpOrderedConnectedKeyFrames_json;
    keyFrameJson["mvOrderedWeights"] = mvOrderedWeights_json;

    if (mpParent && mpParent->isBad() == false)
        // 保证父关键帧存在
        keyFrameJson["mpParent"] = (unsigned long long)mpParent->mnId;
    else {
        keyFrameJson["mpParent"] = Json::nullValue;
    }
    Json::Value mspChildrens_json, mspLoopEdges_json;
    mspChildrens_json.resize(0);
    mspLoopEdges_json.resize(0);
    for (auto childern : mspChildrens) {
        if (childern && childern->isBad() == false)
            mspChildrens_json.append((unsigned long long)childern->mnId);
    }
    keyFrameJson["mspChildrens"] = mspChildrens_json;

    for (auto edge : mspLoopEdges) {
        if (edge && edge->isBad() == false)
            mspLoopEdges_json.append((unsigned long long)edge->mnId);
    }
    keyFrameJson["mspLoopEdges"] = mspLoopEdges_json;

    // Json::Value mGrid_json;
    // mGrid_json.resize(0);
    // for (auto i : mGrid) {
    //     Json::Value mGrid_1;
    //     mGrid_1.resize(0);
    //     for (auto j : i) {
    //         Json::Value mGrid_2;
    //         mGrid_2.resize(0);
    //         for (auto k : j) {
    //             mGrid_2.append((unsigned long long)k);
    //         }
    //         mGrid_1.append(mGrid_2);
    //     }
    //     mGrid_json.append(mGrid_1);
    // }
    // keyFrameJson["mGrid"] = mGrid_json;
}

/**
 * @brief 将关键帧中重复的信息转换为JSON格式，用于地图的保存
 *
 * @param commonJson 输出的JSON格式
 */
void KeyFrame::saveCommonData2Json(Json::Value &commonJson) {
    // 寻找一个能用的关键帧的指针来提取那些相同的信息
    commonJson["nNextId"] = (unsigned long long)nNextId;
    commonJson["mnGridCols"] = mnGridCols;
    commonJson["mnGridRows"] = mnGridRows;
    commonJson["mfGridElementWidthInv"] = mfGridElementWidthInv;
    commonJson["mfGridElementHeightInv"] = mfGridElementHeightInv;
    commonJson["fx"] = fx;
    commonJson["fy"] = fy;
    commonJson["cx"] = cx;
    commonJson["cy"] = cy;
    commonJson["invfx"] = invfx;
    commonJson["invfy"] = invfy;
    commonJson["mbf"] = mbf;
    commonJson["mb"] = mb;
    commonJson["mThDepth"] = mThDepth;
    commonJson["mnScaleLevels"] = mnScaleLevels;
    commonJson["mfScaleFactor"] = mfScaleFactor;
    commonJson["mfLogScaleFactor"] = mfLogScaleFactor;
    commonJson["mnMinX"] = mnMinX;
    commonJson["mnMinY"] = mnMinY;
    commonJson["mnMaxX"] = mnMaxX;
    commonJson["mnMaxY"] = mnMaxY;
    commonJson["mHalfBaseline"] = mHalfBaseline;

    Json::Value mvScaleFactors_json, mvLevelSigma2_json, mvInvLevelSigma2_json, mK_json;
    for (int i = 0, iend = mvScaleFactors.size(); i < iend; ++i) {
        mvScaleFactors_json.append(mvScaleFactors[i]);
        mvLevelSigma2_json.append(mvLevelSigma2[i]);
        mvInvLevelSigma2_json.append(mvInvLevelSigma2[i]);
    }
    commonJson["mvScaleFactors"] = mvScaleFactors_json;
    commonJson["mvLevelSigma2"] = mvLevelSigma2_json;
    commonJson["mvInvLevelSigma2"] = mvInvLevelSigma2_json;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mK_json.append(mK.at<float>(i, j));
        }
    }
    commonJson["mK"] = mK_json;
}

} // namespace ORB_SLAM2

void pose2Json(const cv::Mat &pose, Json::Value &jsonPose) {
    if (pose.empty()) {
        // 非法位姿，构建空数组
        jsonPose.resize(0);
        return;
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            jsonPose.append(pose.at<float>(i, j));
        }
    }
}

void json2Pose(const Json::Value &jsonPose, cv::Mat &pose) {
    cv::Mat tempPose(4, 4, CV_32F);
    if (jsonPose.empty()) {
        return;
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            tempPose.at<float>(i, j) = jsonPose[i * 4 + j].asFloat();
        }
    }
    pose = std::move(tempPose);
}

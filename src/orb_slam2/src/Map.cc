#include <fstream>
#include <jsoncpp/json/json.h>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "Map.h"

namespace ORB_SLAM2 {

Map::Map()
    : mnMaxKFid(0)
    , mnBigChangeIdx(0) {}

void Map::AddKeyFrame(KeyFrame *pKF) {
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    SetUpdated();

    // -----------------------------------
    // cv::Mat pose = pKF->GetPose();
    // for (int i = 0; i < 4; ++i) {
    //     for (int j = 0; j < 4; ++j) {
    //         std::cout << pose.at<float>(i, j) << std::endl;
    //     }
    // }
    // -----------------------------------

    if (pKF->mnId > mnMaxKFid)
        mnMaxKFid = pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP) {
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
    SetUpdated();
}

void Map::EraseMapPoint(MapPoint *pMP) {
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
    // delete pMP;
    // pMP = nullptr;
}

void Map::EraseKeyFrame(KeyFrame *pKF) {
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the Keyframe
    // delete pKF;
    // pKF = nullptr;
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs) {
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange() {
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx() {
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame *> Map::GetAllKeyFrames() {
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
}

vector<MapPoint *> Map::GetAllMapPoints() {
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap() {
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap() {
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint *> Map::GetReferenceMapPoints() {
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid() {
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear() {
    for (set<MapPoint *>::iterator sit = mspMapPoints.begin(), send = mspMapPoints.end(); sit != send; sit++)
        delete *sit;

    for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

//! 保存地图中的关键帧
void Map::saveKeyframesInMap(const std::string &fp) {
    Json::Value root(Json::arrayValue);
    Json::FastWriter writer;
    for (auto keyframe : mspKeyFrames) {
        if (!keyframe) {
            continue;
        }
        if (keyframe->isBad()) {
            continue;
        }
        Json::Value item;

        keyframe->saveKeyFrame2Json(item);
        root.append(item);
    }
    auto json_str = writer.write(root);
    std::ofstream mapOfs(fp);
    mapOfs << json_str << std::endl;

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // std::cout << "关键帧保存成功！" << std::endl;
}

//! 保存地图中的地图点
void Map::saveMappointsInMap(const std::string &fp) {

    Json::Value root(Json::arrayValue);
    Json::FastWriter writer;
    for (auto mappoint : mspMapPoints) {
        if (!mappoint) {
            continue;
        }
        if (mappoint->isBad()) {
            continue;
        }
        Json::Value item;

        mappoint->saveMapPoint2json(item);
        root.append(item);
    }
    auto json_str = writer.write(root);
    std::ofstream mapOfs(fp);
    mapOfs << json_str << std::endl;

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // std::cout << "地图点保存成功！" << std::endl;
}

//! 保存公共数据的api
bool Map::saveCommonData(const std::string &filename) {
    KeyFrame *refkf = nullptr;
    MapPoint *refmp = nullptr;
    RCLCPP_INFO(rclcpp::get_logger("Map"), "地图中目前有关键帧数目为%d", mspKeyFrames.size());
    RCLCPP_INFO(rclcpp::get_logger("Map"), "地图中目前有地图点的数目为%d", mspMapPoints.size());

    // 寻找第一个可用的关键帧和地图点
    for (auto keyframe : mspKeyFrames) {
        if (keyframe != nullptr && keyframe->isBad() == false) {
            refkf = keyframe;
            break;
        }
    }
    for (auto point : mspMapPoints) {
        if (point && point->isBad() == false) {
            refmp = point;
            break;
        }
    }
    if (refmp == nullptr) {
        RCLCPP_WARN(rclcpp::get_logger("Map"), "地图中没有可用地图点！");
        return false;
    }
    if (refkf == nullptr) {
        RCLCPP_WARN(rclcpp::get_logger("Map"), "地图中没有可用关键帧！");
        return false;
    }
    Json::Value root, kfCommon, mpCommon;
    Json::FastWriter writer;
    refkf->saveCommonData2Json(kfCommon);
    refmp->saveCommonData2Json(mpCommon);

    root["kfCommon"] = kfCommon;
    root["mpCommon"] = mpCommon;
    std::ofstream ofs(filename);
    ofs << writer.write(root) << std::endl;
    return true;
}

bool Map::checkJsonFile(const std::string &filename, Json::Reader &reader, Json::Value &jsonValue) {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("Map"), "无法打开文件：%s", filename.c_str());
        return false;
    }
    if (!reader.parse(ifs, jsonValue)) {
        RCLCPP_ERROR(rclcpp::get_logger("Map"), "解析文件失败：%s", filename.c_str());
        return false;
    }
    return true;
}

//! 添加地图的保存api
bool Map::saveMap(const std::string &fileDir) {
    update();
    std::string commonFp = fileDir + "common_data.json";
    std::string kfsFp = fileDir + "keyframes.json";
    std::string mpsFp = fileDir + "mappoints.json";

    if (!saveCommonData(commonFp)) {
        return false;
    }

    // 多线程写入
    std::thread saveKfThread(std::bind(&Map::saveKeyframesInMap, this, kfsFp));
    std::thread saveMpThread(std::bind(&Map::saveMappointsInMap, this, mpsFp));
    saveKfThread.join();
    saveMpThread.join();

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // std::cout << "地图保存成功！" << std::endl;
    return true;
}

/**
 * @brief 用于维护关键帧中和指针相关的数据，用于后续的数据连接
 *
 * @param kFConnectedKfs    关键帧id与一阶相连关键帧id的映射
 * @param kFChildrenKfs     关键帧id与子级关键帧id映射
 * @param kFParentKf        关键帧id与父级关键帧id映射
 * @param kFLoopKfs         关键帧id与所有产生回环帧id映射
 * @param kFConnectMps      关键帧id与所有关联地图点id映射
 * @param kfs               关键帧id与指针映射
 */
void Map::getKFPointer(KeyFrame *kf, Json::Value &kfJson, KFid2LongVID &kFConnectedKfs, KFid2LongVID &kFChildrenKfs,
                       KFid2LongID &kFParentKf, KFid2LongVID &kFLoopKfs, KFid2LongVID &kFConnectMps, KFid2KF &kfs) {
    kfs.insert(std::make_pair(kf->mnId, kf));

    std::vector<long long> mapPointIDs;
    std::vector<long long> orderedKfs;
    std::vector<long long> childrens;
    std::vector<long long> loopEdges;
    long long parentID;

    for (auto ibeg = kfJson["mvpMapPoints"].begin(); ibeg != kfJson["mvpMapPoints"].end(); ++ibeg) {
        mapPointIDs.push_back(ibeg->asInt64());
    }
    for (auto ibeg = kfJson["mvpOrderedConnectedKeyFrames"].begin();
         ibeg != kfJson["mvpOrderedConnectedKeyFrames"].end(); ++ibeg) {
        orderedKfs.push_back(ibeg->asInt64());
    }
    for (auto ibeg = kfJson["mspChildrens"].begin(); ibeg != kfJson["mspChildrens"].end(); ++ibeg) {
        childrens.push_back(ibeg->asInt64());
    }
    for (auto ibeg = kfJson["mspLoopEdges"].begin(); ibeg != kfJson["mspLoopEdges"].end(); ++ibeg) {
        loopEdges.push_back(ibeg->asInt64());
    }
    if (kfJson["mpParent"].isNull()) {
        parentID = -1;
    } else
        parentID = kfJson["mpParent"].asUInt64();
    kFConnectMps.insert(std::make_pair(kf->mnId, mapPointIDs));
    kFConnectedKfs.insert(std::make_pair(kf->mnId, orderedKfs));
    kFChildrenKfs.insert(std::make_pair(kf->mnId, childrens));
    kFLoopKfs.insert(std::make_pair(kf->mnId, loopEdges));
    kFParentKf.insert(std::make_pair(kf->mnId, parentID));
}

/**
 * @brief 用于维护地图中点与指针相关的数据信息，用于后续的数据关联
 *
 * @param mp            地图点指针
 * @param mpJson        地图点的Json格式数据
 * @param mps           地图点的id与指针映射
 * @param mp2RfKF       地图点id与参考关键帧的映射
 * @param mp2RpMp       地图点id与替换地图点id的映射
 * @param mp2Obs        地图点id与观测数据的映射
 */
void Map::getMPPointer(MapPoint *mp, const Json::Value &mpJson, MPid2MP &mps, MPid2RfKF &mp2RfKF, MPid2RpMP &mp2RpMp,
                       MPid2Obs &mp2Obs) {
    (void)mp2Obs;
    // 带有指针需要后续处理的数据
    // std::map<unsigned long int, int> obs;
    long long rfKFId = -1, rpMpId = -1;
    // for (auto it = mpJson["mObservations"].begin(), iend = mpJson["mObservations"].end(); it != iend; ++it) {
    //     auto feaID = it->operator[]("feature_id").asInt();
    //     auto kfID = it->operator[]("keyframe_id").asUInt64();
    //     obs.insert(std::make_pair(kfID, feaID));
    // }
    if (!mpJson["mpRefKF"].isNull()) {
        rfKFId = mpJson["mpRefKF"].asInt64();
    }
    if (!mpJson["mpReplaced"].isNull()) {
        rpMpId = mpJson["mpReplaced"].asInt64();
    }

    mp2RfKF.insert(std::make_pair(mp->mnId, rfKFId));
    mps.insert(std::make_pair(mp->mnId, mp));
    // mp2Obs.insert(std::make_pair(mp->mnId, obs));
    mp2RpMp.insert(std::make_pair(mp->mnId, rpMpId));
}

//! 加载关键帧的api
bool Map::loadKeyframesInFile(const std::string &filename, const Json::Value &commonData, ORBVocabulary *voc,
                              KeyFrameDatabase *kfdb, KFPointer &kfpointer) {

    KFid2LongVID kFConnectedKfs; //< 维护关键帧和与其一阶相连关键帧的id信息
    KFid2LongVID kFChildrenKfs;  //< 维护关键帧和其所有子关键帧的id信息
    KFid2LongID kFParentKf;      //< 维护关键帧和其父关键帧的id信息
    KFid2LongVID kFLoopKfs;      //< 维护回环连接信息
    KFid2LongVID kFConnectMps;   //< 维护关键帧和与之匹配的地图点id信息
    KFid2KF kfs;                 //< 维护关键帧id和关键帧对象的映射

    Json::Value kfsJson;
    Json::Reader reader;
    if (!checkJsonFile(filename, reader, kfsJson)) {
        return false;
    }
    KeyFrame::nNextId = commonData["kfCommon"]["nNextId"].asUInt64();
    for (int ibeg = 0, iend = kfsJson.size(); ibeg < iend; ++ibeg) {
        Json::Value &kfJson = kfsJson[ibeg];
        KeyFrame *kf = new KeyFrame(kfJson, commonData);
        kf->setMap(this);
        mspKeyFrames.insert(kf);

        kf->setKFDB(kfdb);
        kf->setVoc(voc);

        // 维护指针信息
        getKFPointer(kf, kfJson, kFConnectedKfs, kFChildrenKfs, kFParentKf, kFLoopKfs, kFConnectMps, kfs);
    }
    kfpointer.kFConnectedKfs = std::move(kFConnectedKfs);
    kfpointer.kFChildrenKfs = std::move(kFChildrenKfs);
    kfpointer.kFParentKf = std::move(kFParentKf);
    kfpointer.kFLoopKfs = std::move(kFLoopKfs);
    kfpointer.kFConnectMps = std::move(kFConnectMps);
    kfpointer.kfs = std::move(kfs);
    return true;
}

//! 加载地图点的api
bool Map::loadMappointsInFile(const std::string &fp, const Json::Value &commonData, MPPointer &mppointer) {
    MPid2MP &mps = mppointer.mps;           ///< 维护地图点id和地图点对象的映射
    MPid2RfKF &mp2RfKF = mppointer.mp2RfKF; ///< 维护地图点id和参考关键帧id的映射
    MPid2RpMP &mp2RpMp = mppointer.mp2RpMp; ///< 维护地图点id和代替地图点id的映射
    MPid2Obs &mp2Obs = mppointer.mp2Obs;    ///< 维护地图点id和观测信息的映射（!!!!不使用了!!!!）

    Json::Value mpsJson;
    Json::Reader reader;
    if (!checkJsonFile(fp, reader, mpsJson)) {
        return false;
    }
    MapPoint::nNextId = commonData["mpCommon"]["nNextId"].asUInt64();
    for (auto begin = mpsJson.begin(), end = mpsJson.end(); begin != end; ++begin) {
        const Json::Value &mpJson = *begin;
        MapPoint *mp = new MapPoint(mpJson, commonData);
        mp->setMap(this);
        mspMapPoints.insert(mp);

        // 带有指针需要后续处理的数据
        getMPPointer(mp, mpJson, mps, mp2RfKF, mp2RpMp, mp2Obs);
    }
    return true;
}

//! 加载公共数据的api
bool Map::loadCommonData(const std::string &filename, Json::Value &commonData) {
    Json::Reader reader;
    return checkJsonFile(filename, reader, commonData);
}

//! 添加地图的加载api
bool Map::loadMap(const std::string &fileDir, ORBVocabulary *voc, KeyFrameDatabase *kfdb) {
    MPPointer mppointer;
    KFPointer kfpointer;
    Json::Value commonData;
    if (loadCommonData(fileDir + "common_data.json", commonData) == false) {
        return false;
    }

    //! jsoncpp 不支持大数据量的异步加载
    // std::future<bool> retKF = std::async(std::bind(&Map::loadKeyframesInFile, this, fileDir + "keyframes.json",
    //                                                std::ref(commonData), voc, kfdb, std::ref(kfpointer)));
    // std::future<bool> retMP = std::async(std::bind(&Map::loadMappointsInFile, this, fileDir + "mappoints.json",
    //                                                std::ref(commonData), std::ref(mppointer)));

    // retKF.wait();
    // retMP.wait();

    // if (retKF.get() == false || retMP.get() == false) {
    //     return false;
    // }

    bool ret_kf = loadKeyframesInFile(fileDir + "keyframes.json", commonData, voc, kfdb, kfpointer);
    bool ret_mp = loadMappointsInFile(fileDir + "mappoints.json", std::ref(commonData), mppointer);
    if (!ret_kf || !ret_mp) {
        return false;
    }

    procKFandMPPointer(kfpointer, mppointer);

    return true;
}

//! 只处理关键帧对应的连接数据
void Map::procKF(KFPointer &kfPointer) {
    // 关键帧相关的指针类型
    auto &kfs = kfPointer.kfs;
    auto &kFParentKf = kfPointer.kFParentKf;
    auto &kFLoopKfs = kfPointer.kFLoopKfs;
    auto &kFConnectedKfs = kfPointer.kFConnectedKfs;
    auto &kFChildrenKfs = kfPointer.kFChildrenKfs;
    auto &kFConnectMps = kfPointer.kFConnectMps;

    // 更新了mpParent
    for (auto &item : kFParentKf) {
        KeyFrame *parent, *child;
        auto &childID = item.first;
        auto &parentID = item.second;

        if (!kfPointer.findKF(childID, child)) {
            continue;
        }
        if (parentID == -1) {
            child->setParent(nullptr);
        }
        if (kfPointer.findKF(parentID, parent)) {
            child->setParent(parent);
        }
    }

    // 更新了mspLoopEdges
    for (auto &item : kFLoopKfs) {
        KeyFrame *kf, *loopKF;
        const unsigned long int &kfID = item.first;
        if (!kfPointer.findKF(kfID, kf)) {
            continue;
        }
        std::vector<long long> &loopIDs = item.second;
        for (long long &loopID : loopIDs) {
            if (loopID == -1 || !kfPointer.findKF(loopID, loopKF)) {
                continue;
            }
            kf->insertLoopEdge(loopKF);
        }
    }

    // 更新了mConnectedKeyFrameWeights、mvpOrderedConnectedKeyFrames
    for (auto &item : kFConnectedKfs) {
        KeyFrame *kf, *connKF;
        const unsigned long int &kfID = item.first;
        if (!kfPointer.findKF(kfID, kf)) {
            continue;
        }
        std::vector<long long> &connectIDs = item.second;
        for (long long &connectID : connectIDs) {
            if (connectID == -1 || !kfPointer.findKF(connectID, connKF)) {
                continue;
            }
            kf->insertOrderedConnectKF(connKF);
        }
        kf->updateConnectKFandWeight();
    }

    // 更新了mspChildrens
    for (auto &item : kFChildrenKfs) {
        KeyFrame *kf, *childKF;
        const unsigned long int &kfID = item.first;
        if (!kfPointer.findKF(kfID, kf)) {
            continue;
        }
        std::vector<long long> &childIDs = item.second;
        for (long long &childID : childIDs) {
            if (childID == -1 || !kfPointer.findKF(childID, childKF)) {
                continue;
            }
            kf->AddChild(childKF);
        }
    }
}

//! 只处理地图点和地图点连接的数据
void Map::procMP(MPPointer &mpPointer) {
    // 地图点相关的指针类型
    auto &mps = mpPointer.mps;
    auto &mp2RpMp = mpPointer.mp2RpMp;

    // 更新了replace
    for (auto &item : mp2RpMp) {
        MapPoint *mp, *replaceMP;
        const unsigned long int &mpID = item.first;
        long long &replaceID = item.second;

        if (!mpPointer.findMP(mpID, mp)) {
            continue;
        }

        if (replaceID == -1 || !mpPointer.findMP(replaceID, replaceMP))
            mp->setReplace(nullptr);
        else {
            mp->setReplace(replaceMP);
        }
    }
}

//! 添加处理关键帧和地图点指针数据的api
void Map::procKFandMPPointer(KFPointer &kfPointer, MPPointer &mpPointer) {
    procKF(kfPointer);
    procMP(mpPointer);

    auto &mp2RfKF = mpPointer.mp2RfKF;
    auto &kFConnectMps = kfPointer.kFConnectMps;

    // 更新了地图点和关键帧之间的双向观测信息
    for (auto &item : kFConnectMps) {
        KeyFrame *kf;
        MapPoint *mp;
        const unsigned long int &kfID = item.first;
        std::vector<long long> mpIDs = item.second;

        if (!kfPointer.findKF(kfID, kf)) {
            kf->SetBadFlag();
            continue;
        }

        //! 统计一下每个地图点被观测几次
        // std::unordered_map<MapPoint *, int> mpCount;
        std::size_t idx = 0;
        for (long long &mpID : mpIDs) {
            if (mpID == -1 || !mpPointer.findMP(mpID, mp)) {
                mp = nullptr;
                kf->getMapPoints().push_back(mp);
            } else {
                kf->getMapPoints().push_back(mp);
                mp->AddObservation(kf, idx);
                // ++mpCount[mp];
            }
            ++idx;
        }
        // std::cout << "统计结束" << std::endl;
    }

    // 更新了地图点中的mpRefKF属性
    for (auto &item : mp2RfKF) {
        MapPoint *mp;
        KeyFrame *refKF;
        const unsigned long int &mpID = item.first;
        long long &refKFID = item.second;

        if (!mpPointer.findMP(mpID, mp)) {
            continue;
        }

        if (refKFID == -1 || !kfPointer.findKF(refKFID, refKF)) {
            if (!mp->GetObservations().size()) {
                mp->SetBadFlag();
            } else {
                refKF = mp->GetObservations().begin()->first;
                mp->setRefKF(refKF);
            }
        } else
            mp->setRefKF(refKF);
    }
}

void Map::update() {
    for (auto &kf : mspKeyFrames) {
        if (kf && !kf->isBad()) {
            kf->UpdateConnections();
        }
    }
    for (auto &mp : mspMapPoints) {
        if (mp && !mp->isBad() == false) {
            mp->ComputeDistinctiveDescriptors();
            mp->UpdateNormalAndDepth();
        }
    }
}

} // namespace ORB_SLAM2

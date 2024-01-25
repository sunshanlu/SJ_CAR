#include <fstream>
#include <jsoncpp/json/json.h>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "Map.h"

namespace ORB_SLAM2 {

Map::Map()
    : mnMaxKFid(0)
    , mnBigChangeIdx(0) {}

void Map::AddKeyFrame(KeyFrame *pKF) {
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if (pKF->mnId > mnMaxKFid)
        mnMaxKFid = pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP) {
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP) {
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF) {
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the Keyframe
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

//! 添加地图的保存api
bool Map::saveMap(const std::string &fileDir) {
    std::string commonFp = fileDir + "common_data.json";
    std::string kfsFp = fileDir + "keyframes.json";
    std::string mpsFp = fileDir + "mappoints.json";

    if (!saveCommonData(commonFp)) {
        return false;
    }
    saveKeyframesInMap(kfsFp);
    saveMappointsInMap(mpsFp);
    return true;
}

//! 添加地图的加载api
bool Map::loadMap(const std::string &filename) { return true; }

} // namespace ORB_SLAM2

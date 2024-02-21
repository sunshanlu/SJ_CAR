#ifndef MAP_H
#define MAP_H

#include <mutex>
#include <set>
#include <unordered_map>

#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "MapPoint.h"
#include "ORBVocabulary.h"

namespace ORB_SLAM2 {

class MapPoint;
class KeyFrame;
class KeyFrameDatabase;

//! 关键帧加载数据类型定义
using KFid2LongVID = std::unordered_map<unsigned long int, std::vector<long long>>;
using KFid2LongID = std::unordered_map<unsigned long int, long long>;
using KFid2KF = std::unordered_map<unsigned long int, KeyFrame *>;

//! 地图点加载数据类型定义
using MPid2MP = std::unordered_map<unsigned long int, MapPoint *>;
using MPid2RfKF = std::unordered_map<unsigned long int, long long>;
using MPid2RpMP = std::unordered_map<unsigned long int, long long>;
using MPid2Obs = std::unordered_map<unsigned long int, std::map<unsigned long int, int>>;

//! 添加地图点指针类型struct，用于参数接收和加载
struct MPPointer {
    MPid2MP mps;       ///< 维护地图点id和地图点对象的映射
    MPid2RfKF mp2RfKF; ///< 维护地图点id和参考关键帧id的映射
    MPid2RpMP mp2RpMp; ///< 维护地图点id和代替地图点id的映射
    MPid2Obs mp2Obs;   ///< 维护地图点id和观测信息的映射

    /**
     * @brief 给定id，寻找是否有对应的地图点
     *
     * @param id        给定地图点的id
     * @param mp        返回找到的地图点
     * @return true     找到了对应id的地图点
     * @return false    未找到对听id的地图点
     */
    bool findMP(const unsigned long int &id, MapPoint *&mp) {
        auto iter = mps.find(id);
        if (iter == mps.end()) {
            mp = nullptr;
            return false;
        }
        mp = iter->second;
        return true;
    }
};

//! 添加关键帧指针类型的struct，用于参数的接收和加载
struct KFPointer {
    KFid2LongVID kFConnectedKfs; //< 维护关键帧和与其一阶相连关键帧的id信息
    KFid2LongVID kFChildrenKfs;  //< 维护关键帧和其所有子关键帧的id信息
    KFid2LongID kFParentKf;      //< 维护关键帧和其父关键帧的id信息
    KFid2LongVID kFLoopKfs;      //< 维护回环连接信息
    KFid2LongVID kFConnectMps;   //< 维护关键帧和与之匹配的地图点id信息
    KFid2KF kfs;                 //< 维护关键帧id和关键帧对象的映射

    /**
     * @brief 给定关键帧id，在kfs中寻找是否有该关键帧
     *
     * @param id        给定的关键帧id
     * @param kf        返回找到id的关键帧对象，没找到返回nullptr
     * @return true     找到了对应id的关键帧
     * @return false    未找到对应id的关键帧
     */
    bool findKF(const unsigned long int &id, KeyFrame *&kf) {
        auto iter = kfs.find(id);
        if (iter == kfs.end()) {
            kf = nullptr;
            return false;
        }
        kf = iter->second;
        return true;
    }
};

class Map {
public:
    Map();

    void AddKeyFrame(KeyFrame *pKF);
    void AddMapPoint(MapPoint *pMP);
    void EraseMapPoint(MapPoint *pMP);
    void EraseKeyFrame(KeyFrame *pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame *> GetAllKeyFrames();
    std::vector<MapPoint *> GetAllMapPoints();
    std::vector<MapPoint *> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    //! 设置更新
    void SetUpdated() {
        unique_lock<mutex> lock(m_mutexUpdated);
        mb_isUpdated = true;
    }

    //! 重置更新
    void ResetUpdated() {
        unique_lock<mutex> lock(m_mutexUpdated);
        mb_isUpdated = false;
    }

    //! 查看更新
    bool IsUpdated() {
        if (!mb_isUpdated) {
            return false;
        }
        unique_lock<mutex> lock(m_mutexUpdated);
        return mb_isUpdated;
    }

    //! 添加地图的保存api
    bool saveMap(const std::string &fileDir);

    //! 添加地图的加载api
    bool loadMap(const std::string &fileDir, ORBVocabulary *voc, KeyFrameDatabase *kfdb);

    //! 在保存地图之前需要更新地图
    void update();

    std::vector<KeyFrame *> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;
    //! 增加地图析构函数
    ~Map() {
        // 析构关键帧
        for (KeyFrame *kf : mspKeyFrames) {
            if (!kf)
                continue;
            delete kf;
            kf = nullptr;
        }
        // 析构地图点
        for (MapPoint *mp : mspMapPoints) {
            if (!mp)
                continue;
            delete mp;
            mp = nullptr;
        }
    }

protected:
    //! 保存关键帧的api
    void saveKeyframesInMap(const std::string &fp);

    //! 保存地图点的api
    void saveMappointsInMap(const std::string &fp);

    //! 保存关键帧和地图点公共数据的api
    bool saveCommonData(const std::string &filename);

    //! 加载关键帧的api
    bool loadKeyframesInFile(const std::string &fp, const Json::Value &commonData, ORBVocabulary *voc,
                             KeyFrameDatabase *kfdb, KFPointer &kfpointer);

    //! 加载地图点的api
    bool loadMappointsInFile(const std::string &fp, const Json::Value &commonData, MPPointer &mppointer);

    //! 加载公共数据的api（包括地图点和关键帧）
    bool loadCommonData(const std::string &filename, Json::Value &commonData); ///< 加载公共数据的api

    //! 检测json文件是否合法
    bool checkJsonFile(const std::string &filename, Json::Reader &reader, Json::Value &jsonValue);

    //! 维护关键帧中的指针信息，等关键帧和地图点创建完毕后进行关联
    void getKFPointer(KeyFrame *kf, Json::Value &kfJson, KFid2LongVID &kFConnectedKfs, KFid2LongVID &kFChildrenKfs,
                      KFid2LongID &kFParentKf, KFid2LongVID &kFLoopKfs, KFid2LongVID &kFConnectMps, KFid2KF &kfs);

    //! 维护地图点中的指针信息，等关键帧和地图点创建完成后进行关联
    void getMPPointer(MapPoint *mp, const Json::Value &mpJson, MPid2MP &mps, MPid2RfKF &mp2RfKF, MPid2RpMP &mp2RpMp,
                      MPid2Obs &mp2Obs);

    //! 处理关键帧和地图点中指针类型的数据
    void procKFandMPPointer(KFPointer &kfPointer, MPPointer &mpPointer);

    //! 只处理关键帧和关键帧连接的数据
    void procKF(KFPointer &kfPointer);

    //! 只处理地图点和地图点连接的数据
    void procMP(MPPointer &mppointer);

    //! 添加地图是否更新标识
    bool mb_isUpdated;
    std::mutex m_mutexUpdated;

    std::set<MapPoint *> mspMapPoints;
    std::set<KeyFrame *> mspKeyFrames;

    std::vector<MapPoint *> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;
};
} // namespace ORB_SLAM2

#endif // MAP_H

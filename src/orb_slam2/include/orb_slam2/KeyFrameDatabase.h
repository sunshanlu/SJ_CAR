#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <list>
#include <mutex>
#include <set>
#include <vector>

#include "Frame.h"
#include "KeyFrame.h"
#include "ORBVocabulary.h"

namespace ORB_SLAM2 {

class Map;
class KeyFrame;
class Frame;

class KeyFrameDatabase {
public:
    KeyFrameDatabase(const ORBVocabulary &voc);

    void add(KeyFrame *pKF);

    void erase(KeyFrame *pKF);

    void clear();

    // Loop Detection
    std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame *pKF, float minScore);

    // Relocalization
    std::vector<KeyFrame *> DetectRelocalizationCandidates(Frame *F);

    //! 加载整个地图，只有在创建好地图后使用
    void loadMap(Map *pMap);

protected:
    // Associated vocabulary
    const ORBVocabulary *mpVoc;

    // Inverted file
    std::vector<list<KeyFrame *>> mvInvertedFile;

    // Mutex
    std::mutex mMutex;
};

} // namespace ORB_SLAM2

#endif

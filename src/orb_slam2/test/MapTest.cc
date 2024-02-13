#include <gtest/gtest.h>
#include <string>

#include "KeyFrameDatabase.h"
#include "Map.h"
#include "ORBVocabulary.h"

TEST(MAP_TEST, load_map) {
    auto voc = new ORB_SLAM2::ORBVocabulary();
    auto kfdb = new ORB_SLAM2::KeyFrameDatabase(*voc);
    auto map = new ORB_SLAM2::Map();
    std::string fpDir = "/home/lu/Project/SJ_CAR/map/";

    GTEST_ASSERT_EQ(map->loadMap(fpDir, voc, kfdb), true);
}

// TEST(MAP_TEST, save_map) {
//     auto voc = new ORB_SLAM2::ORBVocabulary();
//     auto kfdb = new ORB_SLAM2::KeyFrameDatabase(*voc);
//     auto map = new ORB_SLAM2::Map();
//     std::string fpDir = "/home/lu/Project/SJ_CAR/map/";

//     GTEST_ASSERT_EQ(map->loadMap(fpDir, voc, kfdb), true);
//     GTEST_ASSERT_EQ(map->saveMap("/home/lu/Project/SJ_CAR/src/orb_slam2/map_test/"), true);
// }

int main(int argc, char **argv) {
    // testing::InitGoogleTest(&argc, argv);
    // return RUN_ALL_TESTS();
    auto voc = new ORB_SLAM2::ORBVocabulary();
    auto kfdb = new ORB_SLAM2::KeyFrameDatabase(*voc);
    auto map = new ORB_SLAM2::Map();
    std::string fpDir = "/home/lu/Project/SJ_CAR/map/";
    map->loadMap(fpDir, voc, kfdb);
    for (auto kf : map->GetAllKeyFrames()) {
        kf->GetCameraCenter();
    }
    return 0;
}

include_directories("/home/sj/Project/SJ_CAR/src/orb_slam2/include/orb_slam2")

if(BUILD_TESTING)
    # 静态测试内容
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    # 动态测试内容
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(map_test MapTest.cc TIMEOUT 20)
    target_link_libraries(map_test orb_slam2_lib)

    ament_add_gtest(eigen_test EigenTest.cc TIMEOUT 20)
    ament_add_gtest(future_test FutureTest.cc TIMEOUT 20)
endif()
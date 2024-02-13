cmake_minimum_required(VERSION 3.5)
project(orb_slam2)
set(CMAKE_CXX_STANDARD 14)

# 根据不同的构建类型设置不同的路径
set(ORB_SLAM2_EXEC_RPATH
    ${CMAKE_CURRENT_SOURCE_DIR}/../../install/${CMAKE_BUILD_TYPE}/orb_slam2/lib/orb_slam2)
set(ORB_SLAM2_LIB_RPATH
    ${CMAKE_CURRENT_SOURCE_DIR}/../../install/${CMAKE_BUILD_TYPE}/orb_slam2/share/orb_slam2/Thridparty/lib
)
set(DBoW2_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/Thridparty/lib/libDBoW2.so)
set(G2O_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/Thridparty/lib/libg2o.so)

# 寻找ros2相关依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sj_interfaces REQUIRED)

# 寻找非ros2相关依赖包
find_package(OpenCV 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(gflags REQUIRED)
find_package(PkgConfig)
pkg_check_modules(JSONCPP jsoncpp)

# ORB-SLAM2 编译成lib
file(GLOB ORB_SLAM2_SRC ${PROJECT_SOURCE_DIR}/src/*.cc)
list(FILTER ORB_SLAM2_SRC EXCLUDE REGEX ".*main.*")
add_library(orb_slam2_lib SHARED ${ORB_SLAM2_SRC})
ament_target_dependencies(orb_slam2_lib "rclcpp" "sensor_msgs" "sj_interfaces")
target_link_libraries(
    orb_slam2_lib
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${DBoW2_LIBRARY}
    ${G2O_LIBRARY}
    ${realsense2_LIBRARY}
    ${JSONCPP_LIBRARIES})
target_include_directories(
    orb_slam2_lib
    PUBLIC ${PROJECT_SOURCE_DIR}/Thridparty/include
           ${PROJECT_SOURCE_DIR}/include/orb_slam2 ${EIGEN3_INCLUDE_DIR})

# ORB-SLAM2 的可执行文件
add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cc)
ament_target_dependencies(${PROJECT_NAME} "rclcpp" "sensor_msgs")
target_link_libraries(${PROJECT_NAME} orb_slam2_lib ${GFLAGS_LIBRARIES})

# 安装target和directory到install目录中
install(TARGETS orb_slam2_lib ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY Thridparty DESTINATION share/${PROJECT_NAME})

# 设置target的运行时查找动态库的路径
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH
                                                ${ORB_SLAM2_EXEC_RPATH})
set_target_properties(orb_slam2_lib PROPERTIES INSTALL_RPATH
                                               ${ORB_SLAM2_LIB_RPATH})
                                               

# 设置测试文件夹
add_subdirectory(test)

ament_package()

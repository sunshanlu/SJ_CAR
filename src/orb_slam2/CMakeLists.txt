cmake_minimum_required(VERSION 3.5)
project(orb_slam2)

set(CMAKE_BUILD_TYPE Release)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DBoW2_LIBRARY
    /home/sj/Project/sj_ws/src/orb_slam2/Thridparty/lib/libDBoW2.so)
set(G2O_LIBRARY /home/sj/Project/sj_ws/src/orb_slam2/Thridparty/lib/libg2o.so)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig)
pkg_check_modules(JSONCPP jsoncpp)

file(GLOB ORB_SLAM2_SRC ${PROJECT_SOURCE_DIR}/src/*.cc)
list(FILTER ORB_SLAM2_SRC EXCLUDE REGEX ".*main.*")

# ORB-SLAM2 编译成lib
add_library(orb_slam2_lib SHARED ${ORB_SLAM2_SRC})
ament_target_dependencies(orb_slam2_lib "rclcpp" "sensor_msgs")
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
add_executable(orb_slam2_exec ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cc)
ament_target_dependencies(orb_slam2_exec "rclcpp" "sensor_msgs")
target_link_libraries(orb_slam2_exec orb_slam2_lib)
# target_include_directories( orb_slam2_exec PRIVATE
# ${PROJECT_SOURCE_DIR}/Thridparty/include
# ${PROJECT_SOURCE_DIR}/include/orb_slam2 ${EIGEN3_INCLUDE_DIR})

install(TARGETS orb_slam2_lib orb_slam2_exec DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY Thridparty DESTINATION share/${PROJECT_NAME})

add_subdirectory(test)

ament_package()

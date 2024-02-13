cmake_minimum_required(VERSION 3.5)
project(kitti_odom)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sj_interfaces REQUIRED)

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)

file(GLOB SOURCE_DIR ${PROJECT_SOURCE_DIR}/src/*.cc)
add_executable(${PROJECT_NAME} ${SOURCE_DIR})
ament_target_dependencies(${PROJECT_NAME} rclcpp sj_interfaces)
target_link_libraries(${PROJECT_NAME} fmt::fmt ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()

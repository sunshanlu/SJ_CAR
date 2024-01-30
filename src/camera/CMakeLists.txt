cmake_minimum_required(VERSION 3.5)
project(camera)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
# add_compile_options(-Wall -Wextra -Wpedantic) endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realsense2 REQUIRED)
find_package(sj_interfaces REQUIRED)

file(GLOB SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_executable(camera ${SOURCE_FILE})
target_link_libraries(camera ${realsense2_LIBRARY})
target_include_directories(camera PUBLIC "include")
ament_target_dependencies(camera "rclcpp" "sensor_msgs" "sj_interfaces")
install(TARGETS camera DESTINATION lib/${PROJECT_NAME})

ament_package()

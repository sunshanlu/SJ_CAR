cmake_minimum_required(VERSION 3.5)
project(path_record)

# Default to C99

set(CMAKE_C_STANDARD 99)
# Default to C++14
set(CMAKE_CXX_STANDARD 14)

include_directories(include/path_record)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sj_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
file(GLOB SOURCE_FILE src/*.cc)
add_executable(${PROJECT_NAME} ${SOURCE_FILE})
ament_target_dependencies(${PROJECT_NAME} "rclcpp" "sj_interfaces")
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()

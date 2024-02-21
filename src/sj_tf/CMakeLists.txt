cmake_minimum_required(VERSION 3.5)
project(sj_tf)

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

include_directories(${PROJECT_SOURCE_DIR}/include/sj_tf)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(sj_interfaces REQUIRED)


add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/SjTf.cc)
ament_target_dependencies(${PROJECT_NAME} "rclcpp" "tf2" "tf2_ros")

add_executable(BaseLinkPublisher ${PROJECT_SOURCE_DIR}/src/BaseLinkPublisher.cc)
ament_target_dependencies(BaseLinkPublisher "rclcpp" "tf2" "tf2_ros" "geometry_msgs" "sj_interfaces")

install(TARGETS ${PROJECT_NAME} BaseLinkPublisher DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

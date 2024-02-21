cmake_minimum_required(VERSION 3.5)
project(sj_interfaces)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)

set(MSG_REL_SOURCES "")
file(GLOB MSG_ABS_SOURCES "msg/*.msg")
foreach(FILE_ABS ${MSG_ABS_SOURCES})
    file(RELATIVE_PATH FILE_REL "${CMAKE_CURRENT_SOURCE_DIR}" "${FILE_ABS}")
    list(APPEND MSG_REL_SOURCES "${FILE_REL}")
endforeach(FILE_ABS ${MSG_ABS_SOURCES})

rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_REL_SOURCES} DEPENDENCIES "builtin_interfaces")

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights uncomment
    # the line when a copyright and license is not present in all source files
    # set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint
    # (only works in a git repo) uncomment the line when this package is not in
    # a git repo set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()

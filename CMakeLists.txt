cmake_minimum_required(VERSION 3.8)
project(mosaic-ros2-husky-a200-example)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mosaic-ros2-base REQUIRED)
find_package(mosaic-ros2-geometry REQUIRED)
find_package(mosaic-ros2-sensor REQUIRED)
find_package(clearpath_platform_msgs REQUIRED)

add_executable(mosaic-ros2-husky-a200-example
        src/mosaic_ros2_husky_a200_example.cpp
        src/register.cpp
        src/platform_power_connector.cpp
)
target_include_directories(mosaic-ros2-husky-a200-example PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mosaic-ros2-husky-a200-example
        rclcpp mosaic-ros2-base mosaic-ros2-geometry mosaic-ros2-sensor clearpath_platform_msgs
)

install(TARGETS mosaic-ros2-husky-a200-example
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include
        DESTINATION include/
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(nturt_rpi_deployer)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Boost REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nturt_ros_interface REQUIRED)

# configure wifi-scan
include(cmake/wifi-scan.cmake)

include_directories(
        ${Boost_INCLUDE_DIRS}
        include
)

# libraries
# library: system_stats_monitor
add_library(system_stats_monitor SHARED
        src/system_stats.cpp
        src/system_stats_monitor.cpp
)
target_link_libraries(system_stats_monitor
        wifi-scan
)
ament_target_dependencies(system_stats_monitor
        rclcpp
        rclcpp_components
        nturt_ros_interface
)
# register to node components
# node component: SystemStatsMonitor
rclcpp_components_register_nodes(system_stats_monitor "SystemStatsMonitor")
set(node_plugins "${node_plugins}SystemStatsMonitor;$<TARGET_FILE:system_stats_monitor>\n")

# applications
# executable: system_stats_monitor_node
add_executable(system_stats_monitor_node
        app/system_stats_monitor_node.cpp
)
target_link_libraries(system_stats_monitor_node
        system_stats_monitor
)
ament_target_dependencies(system_stats_monitor_node
        rclcpp
)

# install headers
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)

# install libraries
install(TARGETS system_stats_monitor
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

# install applications
install(TARGETS system_stats_monitor_node
        DESTINATION lib/${PROJECT_NAME}
)

# install launch scripts
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        # the following line skips the linter which checks for copyrights
        # comment the line when a copyright and license is added to all source files
        set(ament_cmake_copyright_FOUND TRUE)
        # the following line skips cpplint (only works in a git repo)
        # comment the line when this package is in a git repo and when
        # a copyright and license is added to all source files
        set(ament_cmake_cpplint_FOUND TRUE)
        ament_lint_auto_find_test_dependencies()
endif()

# export package dependencies
ament_export_include_directories(
        include
)
ament_export_libraries(
        system_stats_monitor
)
ament_export_dependencies(
        adment_cmake
        nturt_ros_interface
        rclcpp
        rclcpp_components
)

ament_package()

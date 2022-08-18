cmake_minimum_required(VERSION 3.0.2)
project(nturt_deploy_to_rpi)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
        can_msgs
        roscpp
        rospy
        std_msgs
)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES test
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(can_test
        test/can_test.cpp
)

add_executable(stm32_test
        test/stm32_test.cpp
)

target_link_libraries(can_test
        ${catkin_LIBRARIES}
)

target_link_libraries(stm32_test
        ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
        scripts/ros_test.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

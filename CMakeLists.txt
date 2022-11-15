cmake_minimum_required(VERSION 3.0.2)
project(nturt_rpi_deployer)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
        can_msgs
        roscpp
        rospy
        std_msgs
)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS can_msgs roscpp rospy std_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# libraries
# library: nturt_led_controller
add_library(nturt_led_controller
        src/nturt_led_controller.cpp
)
target_link_libraries(nturt_led_controller
        ${catkin_LIBRARIES}
        wiringPi
)

# applications
# executable: nturt_led_controller_node
add_executable(nturt_led_controller_node
        app/nturt_led_controller_node.cpp
)
target_link_libraries(nturt_led_controller_node
        nturt_led_controller
)

#tests
# executables: can_test
add_executable(can_test
        test/can_test.cpp
)
target_link_libraries(can_test
        ${catkin_LIBRARIES}
)

# executables: can_test
add_executable(stm32_test
        test/stm32_test.cpp
)
target_link_libraries(stm32_test
        ${catkin_LIBRARIES}
)

# executables: can_test
add_executable(cmd_test
        test/cmd_test
)
target_link_libraries(cmd_test
        ${catkin_LIBRARIES}
)

# python scripts
catkin_install_python(PROGRAMS
        scripts/ros_test.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

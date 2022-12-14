// std include
#include <iostream>

// ros include
#include <ros/ros.h>

int main(int argc, char **argv) {
    // register as a ros node
    ros::init(argc, argv, "cmd_test_node");

    // create a node handle
    ros::NodeHandle nh;

    // sleep for 30 seconds
    ros::Duration(30).sleep();

    // kill ros
    if(system("pkill -f roslaunch") != 0) {
        ROS_FATAL("Fail to kill roslaunch.");
    };

    return 0;
}

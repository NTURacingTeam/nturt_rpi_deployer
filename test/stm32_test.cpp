// ROS include
#include <ros/ros.h>

// ROS message include
#include "can_msgs/Frame.h"

int main(int argc, char **argv) {
    // Register as a ros node
    ros::init(argc, argv, "stm32_test_node");

    // Create a node handle
    ros::NodeHandle nh;

    // Create a publisher to topic "sent_messages"
    ros::Publisher pub = nh.advertise<can_msgs::Frame>("sent_messages", 10);
    
    // Frequancy 10 Hz
    ros::Rate loop_rate(10);

    // Main loop
    while (ros::ok()) {
        // Create test message
        can_msgs::Frame msg;
        msg.id = 0x0008a7d0;
        msg.header.stamp = ros::Time::now();
        msg.dlc = 8;
        msg.is_extended = true;
        msg.data = {1, 0, 0, 0, 0, 0, 0, 0};
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

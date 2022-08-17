// ROS include
#include <ros/ros.h>

// ROS message include
#include "can_msgs/Frame.h"

int main(int argc, char **argv) {
    // Register as a ros node
    ros::init(argc, argv, "can_test_node");

    // Create a node handle
    ros::NodeHandle nh;

    // Create a publisher to topic "sent_messages"
    ros::Publisher pub = nh.advertise<can_msgs::Frame>("sent_messages", 10);
    
    // Frequancy 1 Hz
    ros::Rate loop_rate(1);

    // Main loop
    while (ros::ok()) {
        // Create test message
        can_msgs::Frame msg;
        msg.id = 0x010;
        msg.header.stamp = ros::Time::now();
        msg.dlc = 8;
        msg.data = {0, 1, 2, 3, 4, 5, 6, 7};
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

#include "nturt_led_controller.hpp"

int main(int argc, char **argv) {
    // register as a ros node
    ros::init(argc, argv, "nturt_led_controller_node");

    // create a node handle
    auto nh = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());

    // initialize led controller
    LedController led_controller(nh);
    ros::spin();
    led_controller.cleanup();

    return 0;
}

#include "nturt_led_controller.hpp"

LedController::LedController(std::shared_ptr<ros::NodeHandle> &_nh) :
    nh_(_nh),
    can_sub_(_nh->subscribe("/received_messages", 10, &LedController::onCan, this)),
    rosout_sub_(_nh->subscribe("/rosout", 10, &LedController::onRosout, this)),
    led_timer_(_nh->createTimer(ros::Duration(0.5), &LedController::led_callback, this, false)) {
    
    // initiate wiringpi gpio
    wiringPiSetup();
    pinMode(LED_ROS_PIN, OUTPUT);
    pinMode(LED_CAN_PIN, OUTPUT);
    pinMode(LED_SIGNAL_PIN, OUTPUT);
    pinMode(LED_WARN_PIN, OUTPUT);
    pinMode(LED_ERROR_PIN, OUTPUT);
}

void LedController::cleanup() {
    digitalWrite(LED_ROS_PIN, LOW);
    digitalWrite(LED_CAN_PIN, LOW);
    digitalWrite(LED_SIGNAL_PIN, LOW);
    digitalWrite(LED_WARN_PIN, LOW);
    digitalWrite(LED_ERROR_PIN, LOW);
}

void LedController::onCan(const can_msgs::Frame::ConstPtr &/*_msg*/) {
    digitalWrite(LED_CAN_PIN, HIGH);
    ros::Duration(0.001).sleep();
    digitalWrite(LED_CAN_PIN, LOW);
}

void LedController::onRosout(const rosgraph_msgs::Log::ConstPtr &_msg) {
    if(_msg->level == 4) {
        digitalWrite(LED_WARN_PIN, HIGH);
        ros::Duration(0.1).sleep();
        digitalWrite(LED_WARN_PIN, LOW);
    }
    else if(_msg->level == 8 || _msg->level == 16) {
        digitalWrite(LED_ERROR_PIN, HIGH);
        ros::Duration(0.1).sleep();
        digitalWrite(LED_ERROR_PIN, LOW);
    }
}

void LedController::led_callback(const ros::TimerEvent &/*_event*/) {
    if(!led_on_) {
        digitalWrite(LED_ROS_PIN, HIGH);
        led_on_ = true;
    }
    else{
        digitalWrite(LED_ROS_PIN, LOW);
        led_on_ = false;
    }
}

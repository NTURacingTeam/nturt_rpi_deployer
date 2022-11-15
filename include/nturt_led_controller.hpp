/**
 * @file nturt_led_controller.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief ROS package for controlling led connected to rpi.
 */

#ifndef LED_CONTROLLER_HPP
#define LED_CONTROLLER_HPP

// led pin out in wiringpi fasion
#define LED_ROS_PIN 25
#define LED_CAN_PIN 24
#define LED_SIGNAL_PIN 23
#define LED_WARN_PIN 28
#define LED_ERROR_PIN 27

// std include
#include <memory>

// gpio include
#include <wiringPi.h>

// ros include
#include <ros/ros.h>

// ros message include
#include <can_msgs/Frame.h>
#include <rosgraph_msgs/Log.h>

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class fcontrolling led connected to rpi.
 */
class LedController {
    public:
        /// @brief Constructor of led controller.
        /// @param _nh Shared pointer to can handle.
        LedController(std::shared_ptr<ros::NodeHandle> &_nh);

        /// @brief Function that should be called before the function terminates.
        void cleanup();
    private:
        /// @brief ROS node handler.
        std::shared_ptr<ros::NodeHandle> nh_;

        /// @brief ROS sbscriber to "/received_messages", for receiving can signal.
        ros::Subscriber can_sub_;

        /// @brief ROS subscriber to "/rosout". for receiving error messages from other nodes.
        ros::Subscriber rosout_sub_;

        /// @brief Timer for determine if ros is alive.
        ros::Timer led_timer_;

        // internal state
        /// @brief If ros led is on.
        bool led_on_ = false;

        /// @brief Callback function when receiving message from "/received_messages".
        void onCan(const can_msgs::Frame::ConstPtr &_msg);

        /// @brief Callback function when receiving message from "/rosout".
        void onRosout(const rosgraph_msgs::Log::ConstPtr &_msg);

        /// @brief Timed callback function called for blinking ros led on and off.
        void led_callback(const ros::TimerEvent &_event);
};

#endif

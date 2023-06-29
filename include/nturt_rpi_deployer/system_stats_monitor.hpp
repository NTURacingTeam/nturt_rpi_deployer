/**
 * @file system_stats_monitor.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief ROS2 pacjage for monitoring the system stats and publish it to
 * "system_stats".
 */

#ifndef SYSTEM_STATS_MONITOR_HPP
#define SYSTEM_STATS_MONITOR_HPP

// std include
#include <memory>

// ros2 include
#include <rclcpp/rclcpp.hpp>

// nturt include
#include "nturt_ros_interface/msg/system_stats.hpp"
#include "nturt_rpi_deployer/system_stats.hpp"

class SystemStatsMonitor : public rclcpp::Node {
 public:
  /// @brief Constructor of SystemStatsMonitor.
  SystemStatsMonitor(rclcpp::NodeOptions options);

 private:
  /// @brief Timed callback function for updating system stats.
  void update_system_stats_timer_callback();

  /// @brief ROS2 timer for updating system stats.
  rclcpp::TimerBase::SharedPtr update_system_stats_timer_;

  /// @brief ROS2 publisher to "/system_stats", for publishing system stats.
  rclcpp::Publisher<nturt_ros_interface::msg::SystemStats>::SharedPtr
      system_stats_pub_;

  /// @brief Struct for storing "/system_stats" message data.
  nturt_ros_interface::msg::SystemStats system_stats_;

  /// @brief Class for monitoring cpu stats.
  CpuStats cpu_stats_;

  /// @brief Class for monitoring memory stats.
  MemoryStats memory_stats_;
};

#endif  // SYSTEM_STATS_MONITOR_HPP
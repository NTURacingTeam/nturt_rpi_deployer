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
#include <future>

// ros2 include
#include <rclcpp/rclcpp.hpp>

// wifi-scan include
#include "wifi_scan.h"

// nturt include
#include "nturt_ros_interface/msg/system_stats.hpp"
#include "nturt_rpi_deployer/system_stats.hpp"

class SystemStatsMonitor : public rclcpp::Node {
 public:
  /// @brief Constructor of SystemStatsMonitor.
  SystemStatsMonitor(rclcpp::NodeOptions options);

  /// @brief Destructor of SystemStatsMonitor.
  ~SystemStatsMonitor();

 private:
  /// @brief Timed callback function for updating system stats.
  void update_system_stats_timer_callback();

  /// @brief Timed callback function for updating ping.
  void update_ping_timer_callback();

  /// @brief ROS2 timer for updating system stats.
  rclcpp::TimerBase::SharedPtr update_system_stats_timer_;

  /// @brief ROS2 timer for updating ping.
  rclcpp::TimerBase::SharedPtr update_ping_timer_;

  /// @brief ROS2 publisher to "/system_stats", for publishing system stats.
  rclcpp::Publisher<nturt_ros_interface::msg::SystemStats>::SharedPtr
      system_stats_pub_;

  /// @brief Struct for storing "/system_stats" message data.
  nturt_ros_interface::msg::SystemStats system_stats_;

  /// @brief Class for monitoring cpu stats.
  CpuStats cpu_stats_;

  /// @brief Class for monitoring memory stats.
  MemoryStats memory_stats_;

  /// @brief Struct for storing wifi-scan library info.
  struct wifi_scan* wifi_;

  /// @brief Struct for storing wifi station info.
  struct station_info station_;

  /// @brief Future for storing ping result.
  std::future<double> ping_future_;
};

#endif  // SYSTEM_STATS_MONITOR_HPP

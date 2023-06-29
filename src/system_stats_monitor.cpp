#include "nturt_rpi_deployer/system_stats_monitor.hpp"

// std include
#include <functional>
#include <memory>

// ros2 include
#include <rclcpp/rclcpp.hpp>

// nturt include
#include "nturt_ros_interface/msg/system_stats.hpp"
#include "nturt_rpi_deployer/system_stats.hpp"

using namespace std::chrono_literals;

SystemStatsMonitor::SystemStatsMonitor(rclcpp::NodeOptions options)
    : Node("system_stats_monitor_node", options),
      update_system_stats_timer_(this->create_wall_timer(
          1s, std::bind(&SystemStatsMonitor::update_system_stats_timer_callback,
                        this))),
      system_stats_pub_(
          this->create_publisher<nturt_ros_interface::msg::SystemStats>(
              "/system_stats", 10)) {
  // init cpu stats
  cpu_stats_.update();
}

void SystemStatsMonitor::update_system_stats_timer_callback() {
  CpuStats cpu_stats;
  cpu_stats.update();
  system_stats_.cpu_usage = get_cpu_usage(cpu_stats_, cpu_stats);
  cpu_stats_ = cpu_stats;

  memory_stats_.update();
  system_stats_.memory_usage = memory_stats_.get_memory_usage();
  system_stats_.swap_usage = memory_stats_.get_swap_usage();

  system_stats_.disk_usage = get_disk_usage("/");

  system_stats_.cpu_temperature = get_thermalzone_temperature(0);

  system_stats_pub_->publish(system_stats_);
}

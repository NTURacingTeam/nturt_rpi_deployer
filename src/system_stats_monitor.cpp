#include "nturt_rpi_deployer/system_stats_monitor.hpp"

// std include
#include <functional>
#include <future>
#include <memory>

// ros2 include
#include <rclcpp/rclcpp.hpp>

// wifi-scan include
#include "wifi_scan.h"

// nturt include
#include "nturt_ros_interface/msg/system_stats.hpp"
#include "nturt_rpi_deployer/system_stats.hpp"

using namespace std::chrono_literals;

SystemStatsMonitor::SystemStatsMonitor(rclcpp::NodeOptions options)
    : Node("system_stats_monitor_node", options),
      update_system_stats_timer_(this->create_wall_timer(
          1s, std::bind(&SystemStatsMonitor::update_system_stats_timer_callback,
                        this))),
      update_ping_timer_(this->create_wall_timer(
          10s,
          std::bind(&SystemStatsMonitor::update_ping_timer_callback, this))),
      system_stats_pub_(
          this->create_publisher<nturt_ros_interface::msg::SystemStats>(
              "/system_stats", 10)) {
  // init cpu stats
  cpu_stats_.update();

// init wifi-scan
#if defined(__aarch64__) && !defined(__APPLE__)
  wifi_ = wifi_scan_init("wlan0");
#else
  wifi_ = wifi_scan_init("wlp3s0");
#endif
}

SystemStatsMonitor::~SystemStatsMonitor() { wifi_scan_close(wifi_); }

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

  int status = wifi_scan_station(wifi_, &station_);

  if (status > 0) {
    system_stats_.wifi_ssid = station_.ssid;
    system_stats_.wifi_strength = station_.signal_dbm;
  } else {
    system_stats_.wifi_ssid = "";
    system_stats_.wifi_strength = 0;
  }
}

void SystemStatsMonitor::update_ping_timer_callback() {
  if (ping_future_.valid()) {
    double ret = ping_future_.get();
    if(ret < 0) {
      system_stats_.ping = 0;
    } else {
      system_stats_.ping = ret;
    }
  } else {
    system_stats_.ping = 0;
  }

  ping_future_ = std::async(get_ping_latency, "1.1.1.1");
}

// std include
#include <memory>

// ros2 include
#include "rclcpp/rclcpp.hpp"

// nturt include
#include "nturt_rpi_deployer/system_stats_monitor.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  rclcpp::NodeOptions options;

  rclcpp::Node::SharedPtr system_stats_monitor_node =
      std::make_shared<SystemStatsMonitor>(options);

  executor.add_node(system_stats_monitor_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

// STL
#include <filesystem>
#include <stdexcept>

// ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// open_lmm
#include <open_lmm/server/map_server.hpp>
#include <open_lmm/utils/config.hpp>
// open_lmm_ros
#include "open_lmm_ros.hpp"

namespace open_lmm {

OpenLMMROS::OpenLMMROS(const rclcpp::NodeOptions &options)
    : Node("open_lmm_ros", options) {
  std::string config_path;
  this->declare_parameter<std::string>("config_path", "config");
  this->get_parameter<std::string>("config_path", config_path);

  if (config_path.empty()) {
    throw std::invalid_argument("The 'config_path' parameter must not be empty");
  }

  std::filesystem::path resolved_config_path(config_path);
  if (resolved_config_path.is_relative()) {
    resolved_config_path =
        std::filesystem::path(
            ament_index_cpp::get_package_share_directory("open_lmm")) /
        resolved_config_path;
  }
  resolved_config_path = resolved_config_path.lexically_normal();

  if (!std::filesystem::is_regular_file(resolved_config_path / "config.json")) {
    throw std::runtime_error("Configuration file not found: " +
                             (resolved_config_path / "config.json").string());
  }

  RCLCPP_INFO(this->get_logger(), "Using configuration directory: %s",
              resolved_config_path.c_str());
  open_lmm::GlobalConfig::instance(resolved_config_path.string());

  open_lmm::MapServer map_server;
  map_server.process();

  // TODO(gil) : rviz visualization
}

} // namespace open_lmm
RCLCPP_COMPONENTS_REGISTER_NODE(open_lmm::OpenLMMROS);

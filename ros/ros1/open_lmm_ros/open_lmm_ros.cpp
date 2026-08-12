#include "open_lmm_ros.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>

#include <ros/package.h>
#include <ros/ros.h>

#include <open_lmm/server/map_server.hpp>
#include <open_lmm/utils/config.hpp>

namespace open_lmm {

OpenLMMROS::OpenLMMROS()
    : node_handle_(), private_node_handle_("~") {
  std::string config_path;
  private_node_handle_.param<std::string>("config_path", config_path,
                                          "config");

  if (config_path.empty()) {
    throw std::invalid_argument("The '~config_path' parameter must not be empty");
  }

  std::filesystem::path resolved_config_path(config_path);
  if (resolved_config_path.is_relative()) {
    const std::string package_path = ros::package::getPath("open_lmm");
    if (package_path.empty()) {
      throw std::runtime_error(
          "Unable to locate the 'open_lmm' package with rospack");
    }
    resolved_config_path =
        std::filesystem::path(package_path) / resolved_config_path;
  }
  resolved_config_path = resolved_config_path.lexically_normal();

  if (!std::filesystem::is_regular_file(resolved_config_path / "config.json")) {
    throw std::runtime_error("Configuration file not found: " +
                             (resolved_config_path / "config.json").string());
  }

  ROS_INFO_STREAM("Using configuration directory: " << resolved_config_path);
  open_lmm::GlobalConfig::instance(resolved_config_path.string());

  open_lmm::MapServer map_server;
  map_server.process();
}

}  // namespace open_lmm

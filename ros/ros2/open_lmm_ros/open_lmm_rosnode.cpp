#include <exception>

#include <rclcpp/rclcpp.hpp>

#include "open_lmm_ros.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;

  try {
    auto open_lmm =
        std::make_shared<open_lmm::OpenLMMROS>(rclcpp::NodeOptions{});
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("open_lmm_ros"), "%s", error.what());
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}

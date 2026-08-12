#include <exception>

#include <ros/ros.h>

#include "open_lmm_ros.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "open_lmm_ros");

  try {
    open_lmm::OpenLMMROS open_lmm;
  } catch (const std::exception& error) {
    ROS_FATAL("%s", error.what());
    ros::shutdown();
    return 1;
  }

  ros::shutdown();
  return 0;
}

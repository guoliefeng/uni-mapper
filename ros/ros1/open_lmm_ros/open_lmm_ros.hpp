#pragma once

#include <ros/node_handle.h>

namespace open_lmm {

class OpenLMMROS {
 public:
  OpenLMMROS();
  ~OpenLMMROS() = default;

 private:
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
};

}  // namespace open_lmm

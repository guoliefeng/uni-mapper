#pragma once

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace open_lmm {

// Convertion to string

template <typename T>
std::string convert_to_string(const T& value) {
  std::ostringstream stream;
  stream << std::boolalpha << value;
  return stream.str();
}

template <typename T2>
std::string convert_to_string(const std::vector<T2>& values) {
  std::stringstream sst;
  sst << "[";
  for (unsigned int i = 0; i < values.size(); i++) {
    if (i) {
      sst << ",";
    }
    sst << convert_to_string(values[i]);
  }
  sst << "]";
  return sst.str();
}

template <int D>
std::string convert_to_string(const Eigen::Matrix<double, D, 1>& value) {
  std::stringstream sst;
  sst << std::fixed << std::setprecision(6) << "vec(";
  for (unsigned int i = 0; i < value.size(); i++) {
    if (i) {
      sst << ",";
    }
    sst << value[i];
  }
  sst << ")";
  return sst.str();
}

template <>
inline std::string convert_to_string(const Eigen::Quaterniond& quat) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << "quat(" << quat.x() << ","
         << quat.y() << "," << quat.z() << "," << quat.w() << ")";
  return stream.str();
}

template <>
inline std::string convert_to_string(const Eigen::Isometry3d& pose) {
  const Eigen::Vector3d trans(pose.translation());
  const Eigen::Quaterniond quat(pose.linear());
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << "se3(" << trans.x() << ","
         << trans.y() << "," << trans.z() << "," << quat.x() << ","
         << quat.y() << "," << quat.z() << "," << quat.w() << ")";
  return stream.str();
}
}  // namespace open_lmm

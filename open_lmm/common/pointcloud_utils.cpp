#include "pointcloud_utils.hpp"

#include <Eigen/Dense>
#include <memory>
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/common/shared_data.hpp>
#include <vector>

namespace open_lmm {

std::vector<Eigen::Isometry3f> transformEigenPoses(
    const std::vector<Eigen::Isometry3d>& poses,
    const Eigen::Matrix4f& transform_matrix) {
  std::vector<Eigen::Isometry3f> transformed_poses;
  transformed_poses.reserve(poses.size());

  std::transform(poses.begin(), poses.end(),
                 std::back_inserter(transformed_poses),
                 [&](const Eigen::Isometry3d& pose) {
                   return Eigen::Isometry3f(transform_matrix *
                                            pose.cast<float>().matrix());
                 });

  return transformed_poses;
}

std::vector<Eigen::Vector3f> transformEigenPoints(
    const std::vector<Eigen::Vector3f>& map_points,
    const Eigen::Matrix4f& transform_matrix) {
  std::vector<Eigen::Vector3f> transformed_points;
  transformed_points.reserve(map_points.size());

  std::transform(map_points.begin(), map_points.end(),
                 std::back_inserter(transformed_points),
                 [&](const Eigen::Vector3f& point) {
                   return (transform_matrix * point.homogeneous()).head<3>();
                 });

  return transformed_points;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr downsampleWithRangeFilter(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr p_scan, const float voxel_size,
    const float min_range, const float max_range, const bool use_range_filter) {
  if (voxel_size < 0.01f) return p_scan;

  std::unordered_map<VOXEL_LOC, M_POINT> voxel_map;
  const float inv_voxel_size = 1.0f / voxel_size;

  for (const auto& point : *p_scan) {
    const float range_square =
        point.x * point.x + point.y * point.y + point.z * point.z;
    if (use_range_filter) {
      if (range_square < min_range * min_range ||
          range_square > max_range * max_range)
        continue;
    }

    float loc_xyz[3];
    for (int i = 0; i < 3; i++) {
      loc_xyz[i] = point.data[i] * inv_voxel_size;
      if (loc_xyz[i] < 0) {
        loc_xyz[i] -= 1.0f;
      }
    }

    const VOXEL_LOC voxel_idx(static_cast<uint32_t>(loc_xyz[0]),
                              static_cast<uint32_t>(loc_xyz[1]),
                              static_cast<uint32_t>(loc_xyz[2]));
    auto iter = voxel_map.find(voxel_idx);
    if (iter != voxel_map.end()) {
      iter->second.xyz[0] += point.x;
      iter->second.xyz[1] += point.y;
      iter->second.xyz[2] += point.z;
      iter->second.intensity += point.intensity;
      iter->second.count++;
    } else {
      M_POINT anp;
      anp.xyz[0] = point.x;
      anp.xyz[1] = point.y;
      anp.xyz[2] = point.z;
      anp.intensity = point.intensity;
      anp.count = 1;
      voxel_map[voxel_idx] = anp;
    }
  }

  const uint32_t num_scan_filtered = static_cast<uint32_t>(voxel_map.size());
  pcl::PointCloud<pcl::PointXYZI>::Ptr p_scan_filtered =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(
          new pcl::PointCloud<pcl::PointXYZI>());
  p_scan_filtered->resize(num_scan_filtered);

  std::transform(voxel_map.begin(), voxel_map.end(), p_scan_filtered->begin(),
                 [](const auto& item) -> pcl::PointXYZI {
                   const auto& p = item.second;
                   const float inv_count = 1.0f / p.count;
                   pcl::PointXYZI point;
                   point.x = p.xyz[0] * inv_count;
                   point.y = p.xyz[1] * inv_count;
                   point.z = p.xyz[2] * inv_count;
                   point.intensity = p.intensity * inv_count;
                   return point;
                 });

  return p_scan_filtered;
}

void pclToEigen(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                std::vector<Eigen::Vector3f>& points) {
  points.resize(cloud.size());
  std::transform(
      cloud.begin(), cloud.end(), points.begin(),
      [](const pcl::PointXYZI& p) { return Eigen::Vector3f(p.x, p.y, p.z); });
}

pcl::PointCloud<pcl::PointXYZI>::Ptr readPointsFromPCD(std::string scan_file) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile(scan_file, *cloud);
  return cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr readPointsFromBin(
    const std::string scan_file) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  cloud->clear();
  std::fstream input(scan_file, std::ios::in | std::ios::binary);
  while (input.good() && !input.eof()) {
    Eigen::Vector3f point;
    float intensity;
    if (!input.read(reinterpret_cast<char*>(&point.x()), sizeof(float)) ||
        !input.read(reinterpret_cast<char*>(&point.y()), sizeof(float)) ||
        !input.read(reinterpret_cast<char*>(&point.z()), sizeof(float)) ||
        !input.read(reinterpret_cast<char*>(&intensity), sizeof(float))) {
      break;
    }

    pcl::PointXYZI pcl_point;
    pcl_point.x = point.x();
    pcl_point.y = point.y();
    pcl_point.z = point.z();
    pcl_point.intensity = intensity;
    cloud->push_back(pcl_point);
  }
  return cloud;
}

}  // namespace open_lmm

#include "registration.hpp"

#include <limits>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <small_gicp/registration/registration_helper.hpp>

#include "pointcloud_utils.hpp"

namespace open_lmm {
pcl::PointCloud<pcl::PointXYZI>::Ptr createSubmap(
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& scans_vec,
    const std::vector<Eigen::Isometry3d>& poses_vec, const int key,
    const int search_num) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr submap(
      new pcl::PointCloud<pcl::PointXYZI>);
  const int size = scans_vec.size();

  for (int i = -search_num; i <= search_num; ++i) {
    int key_near = key + i;
    if (key_near < 0 || key_near >= size) continue;
    pcl::PointCloud<pcl::PointXYZI>::Ptr key_near_scan(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*scans_vec[key_near], *key_near_scan,
                             poses_vec[key_near].matrix());
    *submap += *key_near_scan;
  }

  pcl::transformPointCloud(*submap, *submap, poses_vec[key].inverse().matrix());
  return submap;
}

std::optional<Eigen::Isometry3d> calculateFinalTransform(
    const small_gicp::RegistrationResult& result,
    const Eigen::Isometry3d& init_rel_pose, const double fitness) {
  if (!result.converged || fitness > 0.5) {
    return std::nullopt;
  }
  return (result.T_target_source * init_rel_pose).inverse();
}

double calculateFitnessScore(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& target,
    const Eigen::Isometry3d& target_source) {
  if (source->empty() || target->empty()) {
    return std::numeric_limits<double>::infinity();
  }

  pcl::KdTreeFLANN<pcl::PointXYZI> target_tree;
  target_tree.setInputCloud(target);

  double squared_error = 0.0;
  size_t correspondence_count = 0;
  std::vector<int> nearest_index(1);
  std::vector<float> nearest_squared_distance(1);

  for (const auto& source_point : *source) {
    const Eigen::Vector3d transformed =
        target_source * source_point.getVector3fMap().cast<double>();
    pcl::PointXYZI transformed_point;
    transformed_point.x = static_cast<float>(transformed.x());
    transformed_point.y = static_cast<float>(transformed.y());
    transformed_point.z = static_cast<float>(transformed.z());

    if (target_tree.nearestKSearch(transformed_point, 1, nearest_index,
                                   nearest_squared_distance) > 0) {
      squared_error += nearest_squared_distance.front();
      ++correspondence_count;
    }
  }

  return correspondence_count == 0
             ? std::numeric_limits<double>::infinity()
             : squared_error / static_cast<double>(correspondence_count);
}

std::optional<Eigen::Isometry3d> registerPointCloud(const SharedDatabase& db,
                                                    const LoopPair& loop_pair,
                                                    const int& search_num) {
  //! load scans and poses(to)
  const auto& scans_vec_to = db.db_scans.at(loop_pair.to.first);
  const auto& poses_vec_to = db.db_odom_poses.at(loop_pair.to.first);
  //! submap merging(to)
  auto submap_to =
      createSubmap(scans_vec_to, poses_vec_to, loop_pair.to.second, search_num);

  //! load scan(from)
  const auto& scan_from =
      db.db_scans.at(loop_pair.from.first)[loop_pair.from.second];
  //! transform scan(from) based on init_rel_pose
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_init_from(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*scan_from, *scan_init_from,
                           loop_pair.init_rel_pose.matrix());

  std::vector<Eigen::Vector3f> source_points;
  std::vector<Eigen::Vector3f> target_points;
  pclToEigen(*scan_init_from, source_points);
  pclToEigen(*submap_to, target_points);

  constexpr size_t kMinimumRegistrationPoints = 10;
  if (source_points.size() < kMinimumRegistrationPoints ||
      target_points.size() < kMinimumRegistrationPoints) {
    return std::nullopt;
  }

  small_gicp::RegistrationSetting setting;
  setting.type = small_gicp::RegistrationSetting::GICP;
  setting.num_threads = 16;
  setting.max_correspondence_distance = 150.0;
  const auto result = small_gicp::align(
      target_points, source_points, Eigen::Isometry3d::Identity(), setting);

  const double fitness = calculateFitnessScore(
      scan_init_from, submap_to, result.T_target_source);

  return calculateFinalTransform(result, loop_pair.init_rel_pose, fitness);
}

}  // namespace open_lmm

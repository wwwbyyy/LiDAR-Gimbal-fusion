#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <octomap/OcTree.h>

namespace voxel_motion_strategy {

struct VoxelData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
  int point_count = 0;
};

class OctomapBuilder {
 public:
  OctomapBuilder() = default;

  /// Build octree + per-voxel normals from PLY files.
  /// points_file: .ply with global map points.
  /// normals_file: .ply with precomputed normals (same count as points).
  /// resolution: octree voxel size in meters.
  /// range_max: ignore points beyond this distance from origin (negative = unlimited).
  bool buildFromPLY(const std::string& points_file,
                    const std::string& normals_file,
                    double resolution = 1.0,
                    double range_max = 150.0);

  /// Serialize to disk (prefix.ot + prefix.normals).
  bool save(const std::string& prefix) const;

  /// Deserialize from disk.
  bool load(const std::string& prefix);

  /// Query normal at a 3D point (O(1) after coordToKey).
  bool getNormal(const octomap::point3d& coord,
                 Eigen::Vector3f& normal,
                 int& count) const;

  /// Query normal at octree key.
  bool getNormalAt(const octomap::OcTreeKey& key,
                   Eigen::Vector3f& normal,
                   int& count) const;

  /// Access the underlying OcTree for ray-casting / traversal.
  const octomap::OcTree* tree() const { return tree_.get(); }
  octomap::OcTree* tree() { return tree_.get(); }

  size_t numVoxels() const { return voxel_data_.size(); }
  double resolution() const { return tree_ ? tree_->getResolution() : 0.0; }

 private:
  std::unique_ptr<octomap::OcTree> tree_;
  std::unordered_map<uint64_t, VoxelData> voxel_data_;

  static uint64_t keyToU64(const octomap::OcTreeKey& key);
  static octomap::OcTreeKey u64ToKey(uint64_t val);
};

}  // namespace voxel_motion_strategy

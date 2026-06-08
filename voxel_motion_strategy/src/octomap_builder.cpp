#include "voxel_motion_strategy/octomap_builder.h"

#include <fstream>
#include <iostream>
#include <cmath>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>

namespace voxel_motion_strategy {

// ---- key conversion (21 bits per axis fits in ±1M voxels) ----

uint64_t OctomapBuilder::keyToU64(const octomap::OcTreeKey& key) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(key.k[0])) << 42) |
         (static_cast<uint64_t>(static_cast<uint32_t>(key.k[1])) << 21) |
          static_cast<uint64_t>(static_cast<uint32_t>(key.k[2]) & 0x1FFFFF);
}

octomap::OcTreeKey OctomapBuilder::u64ToKey(uint64_t val) {
  octomap::OcTreeKey key;
  key.k[0] = static_cast<octomap::key_type>(val >> 42);
  key.k[1] = static_cast<octomap::key_type>((val >> 21) & 0x1FFFFF);
  key.k[2] = static_cast<octomap::key_type>(val & 0x1FFFFF);
  return key;
}

// ---- build ----

bool OctomapBuilder::buildFromPLY(const std::string& points_file,
                                   const std::string& normals_file,
                                   double resolution,
                                   double range_max,
                                   int min_points_per_voxel) {
  // Load point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "[OctomapBuilder] Loading " << points_file << " ..." << std::endl;
  if (pcl::io::loadPLYFile(points_file, *cloud) < 0) {
    std::cerr << "[OctomapBuilder] ERROR: failed to load " << points_file << std::endl;
    return false;
  }

  // Remove NaN
  cloud->erase(
      std::remove_if(cloud->begin(), cloud->end(), [](const pcl::PointXYZ& p) {
        return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
      }),
      cloud->end());
  size_t n_total = cloud->size();
  std::cout << "[OctomapBuilder] " << n_total << " valid points." << std::endl;

  // Load normals
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  std::cout << "[OctomapBuilder] Loading " << normals_file << " ..." << std::endl;
  if (pcl::io::loadPLYFile(normals_file, *normals) < 0) {
    std::cerr << "[OctomapBuilder] WARN: failed to load normals, computing on-the-fly."
              << std::endl;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(0.5);
    ne.setNumberOfThreads(8);
    ne.compute(*normals);
  }

  if (normals->size() != cloud->size()) {
    std::cerr << "[OctomapBuilder] ERROR: normal count mismatch (" << normals->size()
              << " vs " << cloud->size() << ")" << std::endl;
    return false;
  }

  // Build octree
  tree_ = std::make_unique<octomap::OcTree>(resolution);
  voxel_data_.clear();

  size_t sk_range = 0, sk_nan = 0;
  double range_max_sq = (range_max > 0) ? range_max * range_max : -1.0;

  // First pass: insert all points into the octree (batch by coordinate)
  // and accumulate normals per key
  for (size_t i = 0; i < n_total; ++i) {
    const auto& pt = cloud->points[i];

    if (range_max_sq > 0) {
      double d2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if (d2 > range_max_sq) { sk_range++; continue; }
    }

    auto& n = normals->points[i];
    Eigen::Vector3f nf = n.getNormalVector3fMap();
    float nf_norm = nf.norm();
    if (!std::isfinite(nf_norm) || nf_norm < 1e-6) { sk_nan++; continue; }
    nf /= nf_norm;

    octomap::point3d coord(static_cast<float>(pt.x),
                            static_cast<float>(pt.y),
                            static_cast<float>(pt.z));
    octomap::OcTreeKey key;
    if (!tree_->coordToKeyChecked(coord, key)) {
      sk_range++;  // outside octree bounds
      continue;
    }

    // Update occupancy
    tree_->updateNode(key, true);

    // Accumulate normal (hemisphere-flipped relative to first normal seen)
    uint64_t uk = keyToU64(key);
    auto& vd = voxel_data_[uk];
    if (vd.point_count == 0) {
      vd.normal = nf;
      vd.point_count = 1;
    } else {
      // flip to same hemisphere
      if (vd.normal.dot(nf) < 0.0f) nf = -nf;
      vd.normal = (vd.normal * vd.point_count + nf) / (vd.point_count + 1);
      vd.normal.normalize();
      vd.point_count++;
    }
  }

  // Filter voxels below point-count threshold: erase from map, rebuild tree
  if (min_points_per_voxel > 1) {
    size_t removed = 0;
    auto it = voxel_data_.begin();
    while (it != voxel_data_.end()) {
      if (it->second.point_count < min_points_per_voxel) {
        it = voxel_data_.erase(it);
        removed++;
      } else {
        ++it;
      }
    }
    // Rebuild tree from filtered voxel set (deleteNode + prune may corrupt)
    tree_->clear();
    for (const auto& kv : voxel_data_) {
      tree_->updateNode(u64ToKey(kv.first), true);
    }
    std::cout << "[OctomapBuilder] Filtered " << removed
              << " voxels (min_points<" << min_points_per_voxel << ")." << std::endl;
  }

  // Shrink to fit
  tree_->toMaxLikelihood();
  tree_->prune();

  std::cout << "[OctomapBuilder] Built octree: " << voxel_data_.size() << " occupied voxels, "
            << "skipped (range): " << sk_range << ", (nan): " << sk_nan << std::endl;
  std::cout << "[OctomapBuilder] Tree size: " << tree_->size() << " nodes, "
            << "memory: " << tree_->memoryUsage() << " bytes." << std::endl;

  return true;
}

// ---- save / load ----

bool OctomapBuilder::save(const std::string& prefix) const {
  if (!tree_) {
    std::cerr << "[OctomapBuilder] ERROR: no tree to save." << std::endl;
    return false;
  }

  // Save octree
  std::string ot_file = prefix + ".ot";
  std::cout << "[OctomapBuilder] Saving octree to " << ot_file << " ..." << std::endl;
  if (!tree_->write(ot_file)) {
    std::cerr << "[OctomapBuilder] ERROR: failed to write octree." << std::endl;
    return false;
  }

  // Save normals
  std::string nm_file = prefix + ".normals";
  std::cout << "[OctomapBuilder] Saving " << voxel_data_.size() << " normals to "
            << nm_file << " ..." << std::endl;
  std::ofstream ofs(nm_file, std::ios::binary);
  if (!ofs) {
    std::cerr << "[OctomapBuilder] ERROR: cannot open " << nm_file << std::endl;
    return false;
  }

  uint64_t count = voxel_data_.size();
  ofs.write(reinterpret_cast<const char*>(&count), sizeof(count));
  for (const auto& kv : voxel_data_) {
    uint64_t key = kv.first;
    ofs.write(reinterpret_cast<const char*>(&key), sizeof(key));
    float nx = kv.second.normal.x(), ny = kv.second.normal.y(),
          nz = kv.second.normal.z();
    ofs.write(reinterpret_cast<const char*>(&nx), sizeof(nx));
    ofs.write(reinterpret_cast<const char*>(&ny), sizeof(ny));
    ofs.write(reinterpret_cast<const char*>(&nz), sizeof(nz));
    int32_t pc = static_cast<int32_t>(kv.second.point_count);
    ofs.write(reinterpret_cast<const char*>(&pc), sizeof(pc));
  }
  ofs.close();

  std::cout << "[OctomapBuilder] Saved: " << ot_file << " + " << nm_file << std::endl;
  return true;
}

bool OctomapBuilder::load(const std::string& prefix) {
  std::string ot_file = prefix + ".ot";
  std::cout << "[OctomapBuilder] Loading octree from " << ot_file << " ..." << std::endl;

  auto* raw = octomap::AbstractOcTree::read(ot_file);
  if (!raw) {
    std::cerr << "[OctomapBuilder] ERROR: failed to read octree." << std::endl;
    return false;
  }
  auto* ot = dynamic_cast<octomap::OcTree*>(raw);
  if (!ot) {
    std::cerr << "[OctomapBuilder] ERROR: not an OcTree file." << std::endl;
    delete raw;
    return false;
  }
  tree_.reset(ot);

  // Load normals
  std::string nm_file = prefix + ".normals";
  std::cout << "[OctomapBuilder] Loading normals from " << nm_file << " ..." << std::endl;
  std::ifstream ifs(nm_file, std::ios::binary);
  if (!ifs) {
    std::cerr << "[OctomapBuilder] ERROR: cannot open " << nm_file << std::endl;
    tree_.reset();
    return false;
  }

  voxel_data_.clear();
  uint64_t count = 0;
  ifs.read(reinterpret_cast<char*>(&count), sizeof(count));
  voxel_data_.reserve(count);

  for (uint64_t i = 0; i < count; ++i) {
    uint64_t key;
    float nx, ny, nz;
    int32_t pc;
    ifs.read(reinterpret_cast<char*>(&key), sizeof(key));
    ifs.read(reinterpret_cast<char*>(&nx), sizeof(nx));
    ifs.read(reinterpret_cast<char*>(&ny), sizeof(ny));
    ifs.read(reinterpret_cast<char*>(&nz), sizeof(nz));
    ifs.read(reinterpret_cast<char*>(&pc), sizeof(pc));
    VoxelData vd;
    vd.normal = Eigen::Vector3f(nx, ny, nz);
    vd.point_count = pc;
    voxel_data_[key] = vd;
  }
  ifs.close();

  // Verify consistency: every occupied leaf in the octree should have an entry
  // Every entry in voxel_data_ should correspond to an occupied node
  // (If the octree was pruned after building, some inner nodes might be occupied
  //  but not in voxel_data_. This is expected — we only store leaf data.)

  std::cout << "[OctomapBuilder] Loaded: " << voxel_data_.size() << " normals, "
            << "tree nodes: " << tree_->size() << "." << std::endl;
  return true;
}

// ---- query ----

bool OctomapBuilder::getNormal(const octomap::point3d& coord,
                                Eigen::Vector3f& normal,
                                int& count) const {
  if (!tree_) return false;
  octomap::OcTreeKey key;
  if (!tree_->coordToKeyChecked(coord, key)) return false;
  return getNormalAt(key, normal, count);
}

bool OctomapBuilder::getNormalAt(const octomap::OcTreeKey& key,
                                  Eigen::Vector3f& normal,
                                  int& count) const {
  uint64_t uk = keyToU64(key);
  auto it = voxel_data_.find(uk);
  if (it == voxel_data_.end()) {
    // The key may not exist if this node was pruned (inner node or empty)
    return false;
  }
  normal = it->second.normal;
  count = it->second.point_count;
  return true;
}

}  // namespace voxel_motion_strategy

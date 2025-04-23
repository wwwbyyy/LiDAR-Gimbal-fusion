#include "FreeSLAM/Downsample.hpp"

#include <tsl/robin_map.h>

#include <random>
#include <unordered_map>

namespace FreeSLAM {

void RangeSample(const PointVec &points, std::vector<int> &indices, float range_min,
                 float range_max, float z_min, float z_max) {
    bool no_indices = indices.empty();
    int points_num = no_indices ? points.size() : indices.size();
    indices.resize(points_num);
    int indices_num = 0;
    for (int i = 0; i < points_num; i++) {
        int idx = no_indices ? i : indices[i];
        const auto &pt = points[idx];
        float range = pt.getVector3fMap().norm();
        if (range_min < range && range < range_max && z_min < pt.z && pt.z < z_max) {
            indices[indices_num++] = idx;
        }
    }
    indices.resize(indices_num);
}

void RandomVoxelSample(const PointVec &points, std::vector<int> &indices, float resolution) {
    bool no_indices = indices.empty();
    int points_num = no_indices ? points.size() : indices.size();
    // static thread_local tsl::robin_map<Eigen::Vector3i, std::vector<int>, HashVec3i> voxel;
    static thread_local std::unordered_map<Eigen::Vector3i, std::vector<int>, HashVec3i> voxel;
    voxel.reserve(points_num * 3);
    for (int i = 0; i < points_num; i++) {
        int idx = no_indices ? i : indices[i];
        Eigen::Vector3i key = (points[i].getArray3fMap() / resolution + 0.5).cast<int>();
        voxel[key].emplace_back(idx);
    }

    std::random_device rd;
    std::default_random_engine rng(rd());
    indices.resize(voxel.size());
    int indices_num = 0;
    for (const auto &[k, v] : voxel) {
        std::uniform_int_distribution<std::size_t> uniform(0, v.size() - 1);
        indices[indices_num++] = v[uniform(rng)];
    }
    voxel.clear();
}

}  // namespace FreeSLAM

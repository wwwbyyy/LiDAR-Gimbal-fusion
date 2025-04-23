#include "FreeSLAM/HashVoxel.hpp"

#include <Eigen/Dense>
#include <limits>

namespace FreeSLAM {

HashVoxel::HashVoxel(const HashVoxelConfigs &configs) : HashVoxelConfigs(configs) {
    if (max_num_voxels != -1) lut.reserve(max_num_voxels * 10);
}

HashVoxel::HashVoxel(const HashVoxel &other) : HashVoxelConfigs(other) {
    if (max_num_voxels != -1) lut.reserve(max_num_voxels * 10);
    for (auto [key, it] : other.lut) {
        data.emplace_front(*it);
        lut[key] = data.begin();
    }
}

HashVoxel &HashVoxel::operator=(const HashVoxel &other) {
    Reset();
    lut.reserve(other.lut.bucket_count());
    for (auto [key, it] : other.lut) {
        data.emplace_front(*it);
        lut[key] = data.begin();
    }
    return *this;
}

void HashVoxel::Reset() {
    data.clear();
    lut.clear();
}

void HashVoxel::AddPoints(const PointVec &points, const std::vector<int> &indices,
                          Eigen::Ref<const Eigen::Matrix4f> pose) {
    bool no_indices = indices.empty();
    int points_num = no_indices ? points.size() : indices.size();
    for (int i = 0; i < points_num; i++) {
        int idx = no_indices ? i : indices[i];
        auto pt = points[idx];
        pt.getVector4fMap() = pose * pt.getVector4fMap();
        VoxelKey key = (pt.getArray3fMap() / resolution + 0.5).cast<int>();
        auto [it, ok] = lut.emplace(key, std::list<VoxelNode>::iterator{});
        if (ok) {
            data.emplace_front();
            it.value() = data.begin();
            it.value()->push_back(pt);
            it.value()->key = key;
            if (max_num_voxels != -1 && (int)data.size() >= max_num_voxels) {
                lut.erase(data.back().key);
                data.pop_back();
            }
        } else {
            if (max_num_points == -1 || (int)it.value()->size() < max_num_points) {
                bool ok = true;
                for (auto p : *it.value()) {
                    if ((pt.getVector3fMap() - p.getVector3fMap()).norm() < min_distance) {
                        ok = false;
                        break;
                    }
                }
                if (ok) {
                    it.value()->push_back(pt);
                }
            }
            data.splice(data.begin(), data, it.value());
        }
    }
}

PointVec HashVoxel::GetNearestPoints(const PointType &query, int k) const {
    const static VoxelKey nearby[] = {
        {0, 0, 0},  {1, 0, 0},  {0, 1, 0},   {0, 0, 1},   {-1, 0, 0},  {0, -1, 0},  {0, 0, -1},
        {0, 1, 1},  {1, 0, 1},  {1, 1, 0},   {0, -1, -1}, {-1, 0, -1}, {-1, -1, 0}, {0, -1, 1},
        {0, 1, -1}, {-1, 0, 1}, {1, 0, -1},  {-1, 1, 0},  {1, -1, 0},  {1, 1, 1},   {-1, 1, 1},
        {1, -1, 1}, {1, 1, -1}, {1, -1, -1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1}};

    VoxelKey key = (query.getArray3fMap() / resolution + 0.5).cast<int>();
    PointVec candidates;
    for (auto s : nearby) {
        VoxelKey skey = key + s;
        auto it = lut.find(skey);
        if (it == lut.end()) continue;
        candidates.insert(candidates.end(), it.value()->begin(), it.value()->end());
    }
    if ((int)candidates.size() > k) {
        std::nth_element(candidates.begin(), candidates.begin() + k, candidates.end(),
                         [&](const auto &p1, const auto &p2) {
                             return (p1.getVector3fMap() - query.getVector3fMap()).norm() <
                                    (p2.getVector3fMap() - query.getVector3fMap()).norm();
                         });
        candidates.resize(k);
    }
    return candidates;
}

void HashVoxel::GetAllPoints(PointVec &points) const {
    points.clear();
    if (max_num_points != -1) points.reserve(data.size() * max_num_points);
    for (auto n : data) {
        points.insert(points.end(), n.begin(), n.end());
    }
}

}  // namespace FreeSLAM

#pragma once

#include <tsl/robin_map.h>

#include <Eigen/Core>
#include <list>
#include <unordered_map>

#include "FreeSLAM/types.hpp"
#include "reflcpp/core.hpp"
#include "reflcpp/yaml.hpp"

namespace FreeSLAM {
/**
 * @brief the config parameters for HashVoxel
 *
 */
struct HashVoxelConfigs {
    // maximum total voxels number. set -1 unlimited
    int max_num_voxels = -1;
    // maximum points number in one voxel. set -1 unlimited
    int max_num_points = -1;
    // voxel resolution
    double resolution = 0.5;
    // min distance between two points
    double min_distance = 0.1;
};
}  // namespace FreeSLAM

REFLCPP_METAINFO(FreeSLAM::HashVoxelConfigs, ,
                 (max_num_voxels)(max_num_points)(resolution)(min_distance))
REFLCPP_YAML(FreeSLAM::HashVoxelConfigs)

namespace FreeSLAM {

class HashVoxel : private HashVoxelConfigs {
    using VoxelKey = Eigen::Vector3i;
    struct VoxelNode : public PointVec {
        VoxelKey key;
    };

   public:
    HashVoxel(const HashVoxelConfigs &configs = {});

    HashVoxel(HashVoxel &&) = default;

    HashVoxel(const HashVoxel &other);

    HashVoxel &operator=(HashVoxel &&) = default;

    HashVoxel &operator=(const HashVoxel &other);

    void Reset();

    void AddPoints(const PointVec &points, const std::vector<int> &indices = {},
                   Eigen::Ref<const Eigen::Matrix4f> pose = Eigen::Matrix4f::Identity());

    PointVec GetNearestPoints(const PointType &query, int k) const;

    void GetAllPoints(PointVec &points) const;

    std::size_t GetVoxelNum() const { return data.size(); }

   private:
    std::list<VoxelNode> data;
    tsl::robin_map<VoxelKey, std::list<VoxelNode>::iterator, HashVec3i> lut;
    // std::unordered_map<VoxelKey, std::list<VoxelNode>::iterator, HashVec3i> lut;
};

}  // namespace FreeSLAM
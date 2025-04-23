#pragma once

#include "FreeSLAM/types.hpp"

namespace FreeSLAM {

/**
 * @brief sample the points distance in range
 *
 * @param points    input points
 * @param indices   input/output indices
 * @param range_min minimum distance threshold
 * @param range_max maximum distance threshold
 * @param z_min     minimum z-axis threshold
 * @param z_max     maximum z-axis threshold
 */
void RangeSample(const PointVec &points, std::vector<int> &indices, float range_min,
                 float range_max, float z_min = -std::numeric_limits<float>::infinity(),
                 float z_max = std::numeric_limits<float>::infinity());

/**
 * @brief random sample one point in each voxel grid
 *
 * @param points     input points
 * @param indices    input/output indices
 * @param resolution voxel resolution
 */
void RandomVoxelSample(const PointVec &points, std::vector<int> &indices, float resolution);

}  // namespace FreeSLAM
#pragma once

#include "FreeSLAM/types.hpp"

namespace FreeSLAM {

/**
 * @brief Log on SE3
 *
 * @param value
 *
 * @return Eigen::Vector6f [rx, ry, rz, tx, ty, tz]
 */
Eigen::Vector6f SE3Log(Eigen::Ref<const Eigen::Matrix4f> value);

/**
 * @brief Exp on SE3
 *
 * @param value
 * @param epsilon
 *
 * @return Eigen::Matrix4f
 */
Eigen::Matrix4f SE3Exp(Eigen::Ref<const Eigen::Vector6f> value, float epsilon = 1e-8);

/**
 * @brief Deskew point cloud to the zero timestamp coordinate
 *
 * @param points    pointcloud
 * @param stamp_min the zero timestamp
 * @param stamp_max the one timestamp
 * @param indices   (optional) indices to be computed.
 * @param delta     delta pose between zero timestamp and one timestamp coordinate
 * @param forward   true: deskew to the zero timestamp coordinate
 *                  false: deskew to the one timestamp coordinate
 */
void Deskew(PointVec& points, double stamp_min, double stamp_max, const std::vector<int>& indices,
            const Eigen::Matrix4f& delta, bool forward = true);

/**
 * @brief Deskew point cloud to the zero timestamp coordinate
 *
 * @param points  pointcloud
 * @param stamp_min the zero timestamp
 * @param stamp_max the one timestamp
 * @param indices (optional) indices to be computed.
 * @param delta   delta pose between zero timestamp and one timestamp coordinate
 * @param forward true: deskew to the zero timestamp coordinate
 *                false: deskew to the one timestamp coordinate
 */
void Deskew(PointVec& points, double stamp_min, double stamp_max, const std::vector<int>& indices,
            const Eigen::Vector6f& delta_log, bool forward = true);

}  // namespace FreeSLAM
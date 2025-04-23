#pragma once

#include <limits>

#include "FreeSLAM/HashVoxel.hpp"
#include "reflcpp/core.hpp"
#include "reflcpp/yaml.hpp"

namespace FreeSLAM {
/**
 * @brief the config parameters for Registration
 *
 */
struct RegistrationConfigs {
    // max iteration
    int max_iterations = 10;
    // robust kernel [1 / (1 + kernel * error)]
    double robust_kernel = 0.0;
    // iteration terminal criteria for rotation
    double rotation_epsilon = 1e-4;
    // iteration terminal criteria for translation
    double translation_epsilon = 1e-4;
};
/**
 * @brief the registration result
 *
 */
struct RegistrationResults {
    // iterations of this registration
    int iterations = -1;
    // success matched point number
    int matches = -1;
    // MSE for matches point
    double fitness = std::numeric_limits<double>::infinity();
    // delta transformation use by deskew [result = initial * delta]
    Eigen::Matrix4f delta = Eigen::Matrix4f::Identity();
    // information matrix for log(delta)
    Eigen::Matrix6f information_matrix = Eigen::Matrix6f::Identity();
};
}  // namespace FreeSLAM

REFLCPP_METAINFO(FreeSLAM::RegistrationConfigs, ,
                 (max_iterations)(robust_kernel)(rotation_epsilon)(translation_epsilon))
REFLCPP_YAML(FreeSLAM::RegistrationConfigs)

namespace FreeSLAM {
/**
 * @brief registration the frame to the voxel map.
 *
 * @param voxel   reference pointcloud
 * @param frame   pointcloud to be registered.
 * @param indices (optional) indices of points to be compute
 * @param initial (optional) initial transformation estimation
 * @param configs (optional) registration config parameters
 * @return RegistrationResults
 */
RegistrationResults Registration(const HashVoxel &voxel, const PointVec &frame,
                                 const std::vector<int> &indices, const Eigen::Matrix4f &initial,
                                 const RegistrationConfigs &configs);

}  // namespace FreeSLAM

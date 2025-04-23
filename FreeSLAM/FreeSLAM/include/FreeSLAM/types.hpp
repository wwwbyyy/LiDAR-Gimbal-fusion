#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

namespace Eigen {
using Matrix6f = Eigen::Matrix<float, 6, 6>;
using Vector6f = Eigen::Matrix<float, 6, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

namespace FreeSLAM {

template <typename T>
using EigenVec = std::vector<T, Eigen::aligned_allocator<T>>;

struct PointXYZIT {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    double timestamp;
} EIGEN_ALIGN16;

using PointType = PointXYZIT;

using PointVec = pcl::PointCloud<PointType>;

struct HashVec3i {
    std::size_t operator()(const Eigen::Vector3i &v) const {
        return (v[0] * 73856093ull) ^ (v[1] * 471943ull) ^ (v[2] * 83492791ull);
    }
};

}  // namespace FreeSLAM

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(FreeSLAM::PointXYZIT, 
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    )
// clang-format on

#define PROFILE(tag, code)                                                                       \
    do {                                                                                         \
        using namespace std::chrono;                                                             \
        auto t1 = steady_clock::now();                                                           \
        code;                                                                                    \
        auto t2 = steady_clock::now();                                                           \
        std::cout << "[" tag "]: " << duration_cast<microseconds>(t2 - t1).count() / 1e3 << "ms" \
                  << std::endl;                                                                  \
    } while (0)

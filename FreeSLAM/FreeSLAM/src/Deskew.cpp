#include "FreeSLAM/Deskew.hpp"

#include <numeric>

namespace FreeSLAM {

Eigen::Vector6f SE3Log(Eigen::Ref<const Eigen::Matrix4f> value) {
    Eigen::Vector6f result;
    Eigen::AngleAxisf rotation(value.topLeftCorner<3, 3>());
    result.topRows<3>() = rotation.axis() * rotation.angle();
    result.bottomRows<3>() = value.topRightCorner<3, 1>();
    return result;
}

Eigen::Matrix4f SE3Exp(Eigen::Ref<const Eigen::Vector6f> value, float epsilon) {
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    float angle = value.topRows<3>().norm();
    if (std::abs(angle) > epsilon) {
        Eigen::AngleAxisf rotation(angle, value.topRows<3>() / angle);
        result.topLeftCorner<3, 3>() = rotation.matrix();
    }
    result.topRightCorner<3, 1>() = value.bottomRows<3>();
    return result;
}

void Deskew(PointVec& points, double stamp_min, double stamp_max, const std::vector<int>& indices,
            const Eigen::Matrix4f& delta, bool forward) {
    Eigen::Vector6f delta_log = SE3Log(delta);
    Deskew(points, stamp_min, stamp_max, indices, delta_log, forward);
}

void Deskew(PointVec& points, double stamp_min, double stamp_max, const std::vector<int>& indices,
            const Eigen::Vector6f& delta_log, bool forward) {
    bool no_indices = indices.empty();
    int points_num = no_indices ? points.size() : indices.size();
    double stamp_range = stamp_max - stamp_min;
    for (int i = 0; i < points_num; i++) {
        int idx = no_indices ? i : indices[i];
        auto& pt = points[idx];
        double rate = forward ? (pt.timestamp - stamp_min) / stamp_range
                              : (pt.timestamp - stamp_max) / stamp_range;
        pt.getVector4fMap() = SE3Exp(delta_log * rate) * pt.getVector4fMap();
    }
}

}  // namespace FreeSLAM

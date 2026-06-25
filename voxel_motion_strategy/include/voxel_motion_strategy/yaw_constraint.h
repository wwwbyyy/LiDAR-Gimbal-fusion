#pragma once

#include <algorithm>
#include <cmath>

namespace voxel_motion_strategy {

struct MotionConstraintParams {
  double pan_vel_max = 0.5;    // rad/s
  double tilt_vel_max = 0.35;  // rad/s
};

struct AngleRange {
  double min = 0.0;
  double max = 0.0;
  bool valid = false;
};

/// Compute feasible angle range: [current - vel*dt, current + vel*dt].
/// Optionally clamped to [clamp_min, clamp_max] (no clamping if min>=max).
inline AngleRange computeAngleRange(double current_rad,
                                    double vel_max_rad_per_s,
                                    double dt_s,
                                    double clamp_min = -M_PI,
                                    double clamp_max = M_PI) {
  AngleRange r;
  r.min = current_rad - vel_max_rad_per_s * dt_s;
  r.max = current_rad + vel_max_rad_per_s * dt_s;
  if (clamp_min < clamp_max) {
    r.min = std::max(r.min, clamp_min);
    r.max = std::min(r.max, clamp_max);
  }
  r.valid = (r.min <= r.max + 1e-12);
  return r;
}

}  // namespace voxel_motion_strategy

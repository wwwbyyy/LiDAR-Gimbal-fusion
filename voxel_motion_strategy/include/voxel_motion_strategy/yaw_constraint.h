#pragma once

#include <cmath>

namespace voxel_motion_strategy {

struct YawRange {
  double min = 0.0;   // rad
  double max = 0.0;   // rad
  bool valid = false; // false if constraints are inconsistent
};

struct YawConstraintParams {
  double max_angular_velocity = 1.0;       // rad/s  (gimbal physical limit)
  double max_angular_acceleration = 2.0;   // rad/s² (gimbal physical limit)
  double deadzone = 5.0 * M_PI / 180.0;   // rad    (5° default)
};

/// Compute feasible yaw range for the next strategy step.
///
/// Given current yaw θ and angular velocity ω, and a time step dt,
/// the gimbal can apply acceleration α ∈ [-α_max, α_max] such that
/// the resulting velocity ω' = ω + α·dt stays within [-ω_max, ω_max].
///
/// The new yaw is θ' = θ + ω·dt + ½·α·dt².
/// The deadzone δ expands the range outward to avoid locking onto
/// the current direction (small δyaw is handled by lidar deskew).
///
/// Returns the yaw range [θ_min, θ_max] (radians, NOT normalized to [-π,π]).
/// Caller should handle ±π wraparound when mapping to ERP pixels.
inline YawRange computeFeasibleYawRange(double current_yaw_rad,
                                         double current_yaw_vel_rad_per_s,
                                         double dt_s,
                                         const YawConstraintParams& params) {
  const double w_max = params.max_angular_velocity;
  const double a_max = params.max_angular_acceleration;
  const double dz = params.deadzone;

  // Feasible acceleration range (velocity-bounded)
  // ω' = ω + α·dt  ∈  [-w_max, w_max]
  // →  (-w_max - ω)/dt  ≤  α  ≤  (w_max - ω)/dt
  double alpha_vel_min = (-w_max - current_yaw_vel_rad_per_s) / dt_s;
  double alpha_vel_max = ( w_max - current_yaw_vel_rad_per_s) / dt_s;

  double alpha_eff_min = std::max(-a_max, alpha_vel_min);
  double alpha_eff_max = std::min( a_max, alpha_vel_max);

  YawRange range;
  range.valid = (alpha_eff_min <= alpha_eff_max);
  if (!range.valid) return range;

  // θ' = θ + ω·dt + ½·α·dt²
  const double base = current_yaw_rad + current_yaw_vel_rad_per_s * dt_s;
  const double half_dt2 = 0.5 * dt_s * dt_s;

  range.min = base + alpha_eff_min * half_dt2 - dz;
  range.max = base + alpha_eff_max * half_dt2 + dz;

  return range;
}

}  // namespace voxel_motion_strategy

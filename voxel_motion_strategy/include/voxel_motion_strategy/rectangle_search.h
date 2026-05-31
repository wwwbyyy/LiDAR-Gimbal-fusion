#pragma once

#include <Eigen/Dense>

#include "voxel_motion_strategy/integral_image.h"
#include "voxel_motion_strategy/erp_projector.h"

namespace voxel_motion_strategy {

struct RectSearchParams {
  double weight_pitch = 0.01;        // a: penalty for downward pitch (pitch↑ → score↓)
  double hysteresis_ratio = 0.05;    // switch only if new_score > prev_best × (1+ratio) (0=disabled)
  double yaw_step_deg = 3.0;         // search granularity (azimuth)
  double pitch_step_deg = 3.0;       // search granularity (elevation)
  double pitch_min_deg = -50.0;      // candidate pitch lower bound (looking up)
  double pitch_max_deg = 20.0;       // candidate pitch upper bound
  double fov_horizontal_deg = 60.0;  // Avia circular FoV inscribed rectangle
  double fov_vertical_deg = 68.0;    // Avia circular FoV inscribed rectangle
};

struct RectSearchResult {
  double best_yaw_rad = 0.0;
  double best_pitch_rad = 0.0;
  double best_score = -std::numeric_limits<double>::infinity();
  double lambda_min = 0.0;
  int N_eff = 0;
  bool valid = false;
};

/// Search over candidate (yaw, pitch) pairs within the feasible yaw range
/// and pitch bounds. For each candidate the Avia FoV rectangle is queried
/// from the integral image in O(1). The rectangle with the highest score
///
///   score = λ_min - a·pitch
///
/// is returned as the optimal gimbal target.
///
/// yaw_{min,max}_rad: feasible yaw range (from yaw_constraint)
/// erp: the ERP image providing angular → pixel mapping
/// ii:  precomputed integral image over the ERP
RectSearchResult searchBestRectangle(const IntegralImage& ii,
                                      const ERPImage& erp,
                                      double yaw_min_rad,
                                      double yaw_max_rad,
                                      const RectSearchParams& params);

}  // namespace voxel_motion_strategy

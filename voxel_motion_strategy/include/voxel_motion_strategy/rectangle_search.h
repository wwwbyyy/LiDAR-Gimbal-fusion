#pragma once

#include <Eigen/Dense>

#include "voxel_motion_strategy/integral_image.h"
#include "voxel_motion_strategy/erp_projector.h"

namespace voxel_motion_strategy {

struct RectSearchParams {
  double weight_pitch = 0.01;        // a: penalty for pitch (pitch↑ → score↓)
  double pitch_min_deg = -50.0;      // candidate pitch hard limit
  double pitch_max_deg = 20.0;       // candidate pitch hard limit
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

/// Search over candidate (yaw, pitch) pairs.
/// Step size = erp.resolution_rad (1 pixel) for both axes.
/// pitch range is passed dynamically (from motion constraint).
RectSearchResult searchBestRectangle(const IntegralImage& ii,
                                      const ERPImage& erp,
                                      double yaw_min_rad,
                                      double yaw_max_rad,
                                      double pitch_min_rad,
                                      double pitch_max_rad,
                                      const RectSearchParams& params);

}  // namespace voxel_motion_strategy

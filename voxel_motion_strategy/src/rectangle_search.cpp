#include "voxel_motion_strategy/rectangle_search.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include <Eigen/Eigenvalues>

namespace voxel_motion_strategy {

RectSearchResult searchBestRectangle(const IntegralImage& ii,
                                      const ERPImage& erp,
                                      double yaw_min_rad,
                                      double yaw_max_rad,
                                      const RectSearchParams& params) {
  RectSearchResult best;
  best.valid = false;

  if (ii.width() <= 1 || ii.height() <= 1 || erp.empty()) return best;

  const double res_rad = erp.resolution_rad;
  const double h_min = erp.h_min_rad;
  const double v_max = erp.v_max_rad;

  const double fov_hw = (params.fov_horizontal_deg * M_PI / 180.0) * 0.5; // half-FoV
  const double fov_hh = (params.fov_vertical_deg   * M_PI / 180.0) * 0.5;

  const double fov_w_px = fov_hw * 2.0 / res_rad;  // rectangle width in pixels
  const double fov_h_px = fov_hh * 2.0 / res_rad;  // rectangle height in pixels

  const double pitch_min_rad = params.pitch_min_deg * M_PI / 180.0;
  const double pitch_max_rad = params.pitch_max_deg * M_PI / 180.0;
  const double yaw_step_rad   = params.yaw_step_deg   * M_PI / 180.0;
  const double pitch_step_rad = params.pitch_step_deg * M_PI / 180.0;

  // ERP image dimensions (without extension)
  const int img_w = erp.width;

  int candidates = 0;
  int skipped_empty = 0;

  // Iterate over yaw and pitch candidates
  for (double yaw_c = yaw_min_rad; yaw_c <= yaw_max_rad + 1e-9; yaw_c += yaw_step_rad) {
    // Horizontal pixel range for this yaw center
    double u_center = (yaw_c - h_min) / res_rad;
    double u1 = u_center - fov_w_px * 0.5;
    double u2 = u_center + fov_w_px * 0.5;

    // Handle horizontal wraparound: shift to extended region
    if (u1 < 0.0) { u1 += img_w; u2 += img_w; }

    for (double pitch_c = pitch_min_rad; pitch_c <= pitch_max_rad + 1e-9;
         pitch_c += pitch_step_rad) {
      candidates++;

      // Vertical pixel range
      double v_center = (v_max - pitch_c) / res_rad;
      double v1 = v_center - fov_h_px * 0.5;
      double v2 = v_center + fov_h_px * 0.5;

      // Convert to SAT indices (SAT has +1 offset from pixel coords)
      int su1 = static_cast<int>(std::round(u1));
      int su2 = static_cast<int>(std::round(u2));
      int sv1 = static_cast<int>(std::round(v1));
      int sv2 = static_cast<int>(std::round(v2));

      // Clamp to valid SAT range [0, w_-1]
      su1 = std::max(0, std::min(su1, ii.width() - 1));
      su2 = std::max(0, std::min(su2, ii.width() - 1));
      sv1 = std::max(0, std::min(sv1, ii.height() - 1));
      sv2 = std::max(0, std::min(sv2, ii.height() - 1));

      if (su2 <= su1 || sv2 <= sv1) continue;

      // O(1) query from integral image
      Eigen::Matrix3d S;
      int N_eff;
      ii.query(su1, su2, sv1, sv2, S, N_eff);

      if (N_eff == 0) { skipped_empty++; continue; }

      // λ_min from S
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(S);
      double lambda_min = eig.eigenvalues()[0];  // smallest eigenvalue
      if (lambda_min < 0.0) lambda_min = 0.0;     // numerical noise guard

      // Score: λ_min already encodes N_eff via tr(S)=Σ|n|=N_eff
      double score = lambda_min - params.weight_pitch * pitch_c;

      if (score > best.best_score) {
        best.best_score = score;
        best.best_yaw_rad = yaw_c;
        best.best_pitch_rad = pitch_c;
        best.lambda_min = lambda_min;
        best.N_eff = N_eff;
        best.valid = true;
      }
    }
  }

  std::cout << "[RectSearch] Searched " << candidates << " candidates ("
            << skipped_empty << " empty), best: yaw="
            << best.best_yaw_rad * 180.0 / M_PI << "° pitch="
            << best.best_pitch_rad * 180.0 / M_PI << "° score="
            << best.best_score << " λ_min=" << best.lambda_min
            << " N=" << best.N_eff << std::endl;

  return best;
}

}  // namespace voxel_motion_strategy

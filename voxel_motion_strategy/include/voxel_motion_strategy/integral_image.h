#pragma once

#include <vector>
#include <Eigen/Dense>

#include "voxel_motion_strategy/erp_projector.h"

namespace voxel_motion_strategy {

/// Summed-area table over the ERP image for O(1) rectangle queries.
///
/// Tracks two quantities per pixel:
///   1. M = n·nᵀ  (3×3 symmetric, stored as 6 scalar SATs)
///   2. occupied flag (1 scalar SAT)
///
/// A rectangle query returns S = Σ M  and N_eff = Σ occupied over the region.
/// S is the second-moment matrix used to compute λ_min for localizability.
class IntegralImage {
 public:
  IntegralImage() = default;

  /// Build SATs from an ERP image.
  /// h_extend: extra columns to append from the left side for wraparound
  ///           (typically FoV_width_pixels for rectangle search across 0°/360°).
  static IntegralImage build(const ERPImage& erp, int h_extend = 0);

  int width() const { return w_; }
  int height() const { return h_; }

  /// Query sum over rectangle [u1, u2) × [v1, v2) (half-open).
  /// Returns S (3×3 symmetric) and N_eff (occupied pixel count).
  /// All indices are clamped to valid range.
  void query(int u1, int u2, int v1, int v2,
             Eigen::Matrix3d& S, int& N_eff) const;

  /// Low-level SAT access for testing.
  double satVal(int comp, int u, int v) const;  // comp: 0..5

 private:
  int w_ = 0;   // SAT width  = image width + 1
  int h_ = 0;   // SAT height = image height + 1

  // 6 symmetric matrix components: (0,0), (0,1), (0,2), (1,1), (1,2), (2,2)
  // Each is a size-(w_ * h_) row-major SAT.
  std::vector<double> sat_[6];
  // Occupancy SAT (int)
  std::vector<int> sat_occ_;

  size_t idx(int u, int v) const { return static_cast<size_t>(v) * w_ + u; }
};

}  // namespace voxel_motion_strategy

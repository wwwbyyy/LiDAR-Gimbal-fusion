#include "voxel_motion_strategy/integral_image.h"

#include <algorithm>
#include <iostream>
#include <cstring>

namespace voxel_motion_strategy {

IntegralImage IntegralImage::build(const ERPImage& erp, int h_extend) {
  IntegralImage ii;

  int img_w = erp.width;
  int img_h = erp.height;
  int ext_w = img_w + h_extend;  // extended width for wraparound
  if (img_w <= 0 || img_h <= 0) return ii;

  ii.w_ = ext_w + 1;   // SAT has +1 sentinel
  ii.h_ = img_h + 1;
  size_t n_cells = static_cast<size_t>(ii.w_) * ii.h_;

  for (int c = 0; c < 6; ++c) {
    ii.sat_[c].assign(n_cells, 0.0);
  }
  ii.sat_occ_.assign(n_cells, 0);

  // Build per-pixel values in-place in the SAT arrays (row 0 and col 0 stay 0)
  for (int v = 0; v < img_h; ++v) {
    for (int u = 0; u < ext_w; ++u) {
      // Map extended u back to original ERP column (wraparound)
      int src_u = (u < img_w) ? u : (u - img_w);

      const auto& n = erp.normalAt(src_u, v);
      bool occ = erp.isOccupied(src_u, v);

      double m00 = 0, m01 = 0, m02 = 0, m11 = 0, m12 = 0, m22 = 0;
      int o = 0;

      if (occ) {
        float nx = n.x(), ny = n.y(), nz = n.z();
        m00 = static_cast<double>(nx * nx);
        m01 = static_cast<double>(nx * ny);
        m02 = static_cast<double>(nx * nz);
        m11 = static_cast<double>(ny * ny);
        m12 = static_cast<double>(ny * nz);
        m22 = static_cast<double>(nz * nz);
        o = 1;
      }

      size_t cell = ii.idx(u + 1, v + 1);  // +1 for SAT sentinel
      size_t left  = ii.idx(u, v + 1);
      size_t up    = ii.idx(u + 1, v);
      size_t diag  = ii.idx(u, v);

      ii.sat_[0][cell] = m00 + ii.sat_[0][left] + ii.sat_[0][up] - ii.sat_[0][diag];
      ii.sat_[1][cell] = m01 + ii.sat_[1][left] + ii.sat_[1][up] - ii.sat_[1][diag];
      ii.sat_[2][cell] = m02 + ii.sat_[2][left] + ii.sat_[2][up] - ii.sat_[2][diag];
      ii.sat_[3][cell] = m11 + ii.sat_[3][left] + ii.sat_[3][up] - ii.sat_[3][diag];
      ii.sat_[4][cell] = m12 + ii.sat_[4][left] + ii.sat_[4][up] - ii.sat_[4][diag];
      ii.sat_[5][cell] = m22 + ii.sat_[5][left] + ii.sat_[5][up] - ii.sat_[5][diag];
      ii.sat_occ_[cell] = o + ii.sat_occ_[left] + ii.sat_occ_[up] - ii.sat_occ_[diag];
    }
  }

  return ii;
}

void IntegralImage::query(int u1, int u2, int v1, int v2,
                           Eigen::Matrix3d& S, int& N_eff) const {
  // Clamp
  u1 = std::max(0, std::min(u1, w_ - 1));
  u2 = std::max(0, std::min(u2, w_ - 1));
  v1 = std::max(0, std::min(v1, h_ - 1));
  v2 = std::max(0, std::min(v2, h_ - 1));

  if (u1 >= u2 || v1 >= v2) {
    S.setZero();
    N_eff = 0;
    return;
  }

  size_t a = idx(u1, v1);
  size_t b = idx(u2, v1);
  size_t c = idx(u1, v2);
  size_t d = idx(u2, v2);

  S(0,0) = sat_[0][d] - sat_[0][b] - sat_[0][c] + sat_[0][a];
  S(0,1) = S(1,0) = sat_[1][d] - sat_[1][b] - sat_[1][c] + sat_[1][a];
  S(0,2) = S(2,0) = sat_[2][d] - sat_[2][b] - sat_[2][c] + sat_[2][a];
  S(1,1) = sat_[3][d] - sat_[3][b] - sat_[3][c] + sat_[3][a];
  S(1,2) = S(2,1) = sat_[4][d] - sat_[4][b] - sat_[4][c] + sat_[4][a];
  S(2,2) = sat_[5][d] - sat_[5][b] - sat_[5][c] + sat_[5][a];

  N_eff = sat_occ_[d] - sat_occ_[b] - sat_occ_[c] + sat_occ_[a];
}

double IntegralImage::satVal(int comp, int u, int v) const {
  if (comp < 0 || comp > 5 || u < 0 || u >= w_ || v < 0 || v >= h_) return 0.0;
  return sat_[comp][idx(u, v)];
}

}  // namespace voxel_motion_strategy

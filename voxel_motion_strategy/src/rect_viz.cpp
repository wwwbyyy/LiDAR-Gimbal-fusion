#include <iostream>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <limits>

#include "voxel_motion_strategy/octomap_builder.h"
#include "voxel_motion_strategy/erp_projector.h"
#include "voxel_motion_strategy/integral_image.h"
#include "voxel_motion_strategy/rectangle_search.h"

using namespace voxel_motion_strategy;
static constexpr double kDeg = M_PI / 180.0;

int main(int argc, char** argv) {
  if (argc < 5) {
    std::cerr << "Usage: rect_viz <octomap_prefix> <x> <y> <z> [csv_out] [ppm_out]" << std::endl;
    return 1;
  }
  std::string prefix = argv[1];
  double px = std::stod(argv[2]), py = std::stod(argv[3]), pz = std::stod(argv[4]);
  std::string csv_file = (argc > 5) ? argv[5] : "";
  std::string ppm_file = (argc > 6) ? argv[6] : "";

  // Load
  OctomapBuilder builder;
  if (!builder.load(prefix)) { std::cerr << "FAILED" << std::endl; return 1; }

  // ERP params
  double res_deg = 2.0;  // 2°/pixel for speed; use 1° for final
  ERPParams ep; ep.resolution_deg = res_deg; ep.range_max_m = 150.0;
  double h_min = -180.0 * kDeg, h_max = 180.0 * kDeg;
  double v_min = -90.0 * kDeg, v_max = 60.0 * kDeg;

  // Project
  Eigen::Vector3d origin(px, py, pz);
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  std::cout << "Projecting ERP..." << std::endl;
  ERPImage erp = projectERP(builder, origin, R, h_min, h_max, v_min, v_max, ep);

  // Integral image
  RectSearchParams sp;
  sp.weight_pitch = 0.0;
  double fov_w_px = sp.fov_horizontal_deg / res_deg;
  int h_ext = static_cast<int>(std::ceil(fov_w_px));
  IntegralImage ii = IntegralImage::build(erp, h_ext);

  // Per-yaw search: for each yaw, find best pitch
  double yaw_step = 5.0 * kDeg;
  double pitch_step = 3.0 * kDeg;
  double p_min = sp.pitch_min_deg * kDeg, p_max = sp.pitch_max_deg * kDeg;
  double fov_hw = (sp.fov_horizontal_deg * kDeg) * 0.5;
  double fov_hh = (sp.fov_vertical_deg   * kDeg) * 0.5;
  double fov_w2 = fov_hw / erp.resolution_rad;  // half-width in pixels
  double fov_h2 = fov_hh / erp.resolution_rad;

  std::vector<std::tuple<double,double,double,double,int>> per_yaw;  // yaw,pitch,score,lambda,N

  double global_best_score = -std::numeric_limits<double>::infinity();
  double global_best_yaw = 0, global_best_pitch = 0;

  for (double yaw = -180.0 * kDeg; yaw < 180.0 * kDeg; yaw += yaw_step) {
    double best_score = -std::numeric_limits<double>::infinity();
    double best_pitch = 0, best_lambda = 0;
    int best_N = 0;

    double u_c = (yaw - erp.h_min_rad) / erp.resolution_rad;
    double u1 = u_c - fov_w2, u2 = u_c + fov_w2;
    if (u1 < 0.0) { u1 += erp.width; u2 += erp.width; }

    for (double pitch = p_min; pitch <= p_max + 1e-9; pitch += pitch_step) {
      double v_c = (erp.v_max_rad - pitch) / erp.resolution_rad;
      double v1 = v_c - fov_h2, v2 = v_c + fov_h2;

      int su1 = std::max(0, std::min(static_cast<int>(std::round(u1)), ii.width()-1));
      int su2 = std::max(0, std::min(static_cast<int>(std::round(u2)), ii.width()-1));
      int sv1 = std::max(0, std::min(static_cast<int>(std::round(v1)), ii.height()-1));
      int sv2 = std::max(0, std::min(static_cast<int>(std::round(v2)), ii.height()-1));
      if (su2 <= su1 || sv2 <= sv1) continue;

      Eigen::Matrix3d S; int N;
      ii.query(su1, su2, sv1, sv2, S, N);
      if (N == 0) continue;

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(S);
      double lm = std::max(0.0, eig.eigenvalues()[0]);
      double score = lm;  // λ_min alone; weight_pitch = 0
      if (score > best_score) { best_score = score; best_pitch = pitch; best_lambda = lm; best_N = N; }
    }
    per_yaw.emplace_back(yaw, best_pitch, best_score, best_lambda, best_N);
    if (best_score > global_best_score) {
      global_best_score = best_score; global_best_yaw = yaw; global_best_pitch = best_pitch;
    }
  }

  // Output CSV
  std::ofstream csv;
  if (!csv_file.empty()) csv.open(csv_file);
  std::ostream& out = csv_file.empty() ? std::cout : csv;
  out << "yaw_deg,best_pitch_deg,score,lambda_min,N_eff" << std::endl;
  for (const auto& [y, p, s, l, n] : per_yaw) {
    out << y/kDeg << "," << p/kDeg << "," << s << "," << l << "," << n << std::endl;
  }
  if (!csv_file.empty()) { csv.close(); std::cout << "CSV saved: " << csv_file << std::endl; }

  std::cout << "Global best: yaw=" << global_best_yaw/kDeg << "° pitch=" << global_best_pitch/kDeg
            << "° score=" << global_best_score << std::endl;

  // Best-view depth map PPM
  if (!ppm_file.empty()) {
    // Extract a centered window around the best yaw
    int view_w = static_cast<int>(90.0 / res_deg);  // 90° wide view
    int view_h = erp.height;
    int center_u = static_cast<int>((global_best_yaw - erp.h_min_rad) / erp.resolution_rad);
    int u_start = center_u - view_w/2;
    // wraparound handled simplistically: clamp
    u_start = std::max(0, std::min(u_start, erp.width - view_w));

    std::ofstream ppm(ppm_file);
    ppm << "P3\n" << view_w << " " << view_h << "\n255\n";
    for (int v = 0; v < view_h; ++v) {
      for (int u = 0; u < view_w; ++u) {
        int src_u = (u_start + u) % erp.width;
        auto n = erp.normalAt(src_u, v);
        if (erp.isOccupied(src_u, v)) {
          Eigen::Vector3f nv = n;
          if (nv.z() < 0) nv = -nv;
          int r = (nv.x()+1)*0.5*255, g = (nv.y()+1)*0.5*255, b = (nv.z()+1)*0.5*255;
          ppm << r << " " << g << " " << b << " ";
        } else { ppm << "0 0 0 "; }
      }
      ppm << "\n";
    }
    ppm.close();
    std::cout << "PPM saved: " << ppm_file << std::endl;
  }

  return 0;
}

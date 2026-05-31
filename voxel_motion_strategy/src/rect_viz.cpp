#include <iostream>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <limits>
#include <algorithm>
#include <vector>
#include <tuple>

#include <yaml-cpp/yaml.h>

#include "voxel_motion_strategy/octomap_builder.h"
#include "voxel_motion_strategy/erp_projector.h"
#include "voxel_motion_strategy/integral_image.h"
#include "voxel_motion_strategy/rectangle_search.h"

using namespace voxel_motion_strategy;
static constexpr double kDeg = M_PI / 180.0;

int main(int argc, char** argv) {
  if (argc < 6) {
    std::cerr << "Usage: rect_viz <octomap_prefix> <x> <y> <z> <config_yaml> [csv_out] [ppm_heat] [ppm_full] [ppm_best]" << std::endl
              << "  config_yaml : path to voxel_strategy.yaml (reads search + ERP params)" << std::endl
              << "  csv_out     : full 2D grid (yaw × pitch → score)" << std::endl
              << "  ppm_heat    : score heatmap image (yaw × pitch)" << std::endl
              << "  ppm_full    : full-range ERP depth map (normal colors)" << std::endl
              << "  ppm_best    : best-view FoV depth map (normal colors)" << std::endl;
    return 1;
  }
  std::string prefix       = argv[1];
  double px = std::stod(argv[2]), py = std::stod(argv[3]), pz = std::stod(argv[4]);
  std::string yaml_file     = argv[5];
  std::string csv_file      = (argc > 6) ? argv[6] : "";
  std::string ppm_heat      = (argc > 7) ? argv[7] : "";
  std::string ppm_full      = (argc > 8) ? argv[8] : "";
  std::string ppm_best      = (argc > 9) ? argv[9] : "";

  // ---- Load params from YAML ----
  YAML::Node cfg = YAML::LoadFile(yaml_file);

  // ERP params
  double res_deg     = cfg["erp_resolution_deg"].as<double>(2.0);
  double range_max   = cfg["erp_range_max_m"].as<double>(150.0);
  double self_range  = cfg["erp_self_range_m"].as<double>(2.0);
  double vfov_min_d  = cfg["erp_vfov_min_deg"].as<double>(-90.0);
  double vfov_max_d  = cfg["erp_vfov_max_deg"].as<double>(60.0);

  ERPParams ep;
  ep.resolution_deg = res_deg;
  ep.range_max_m    = range_max;
  ep.self_range_m   = self_range;

  double h_min = -180.0 * kDeg, h_max = 180.0 * kDeg;
  double v_min = vfov_min_d * kDeg, v_max = vfov_max_d * kDeg;

  // Rectangle search params
  RectSearchParams sp;
  sp.fov_horizontal_deg = cfg["fov_horizontal_deg"].as<double>(60.0);
  sp.fov_vertical_deg   = cfg["fov_vertical_deg"].as<double>(68.0);
  sp.yaw_step_deg       = cfg["yaw_step_deg"].as<double>(3.0);
  sp.pitch_step_deg     = cfg["pitch_step_deg"].as<double>(3.0);
  sp.pitch_min_deg      = cfg["pitch_min_deg"].as<double>(-50.0);
  sp.pitch_max_deg      = cfg["pitch_max_deg"].as<double>(20.0);
  sp.weight_pitch       = cfg["weight_pitch"].as<double>(0.0);

  std::cout << "Config from " << yaml_file << ":" << std::endl
            << "  ERP: " << res_deg << "°/px, range=" << range_max
            << "m self_range=" << self_range << "m, vfov=["
            << vfov_min_d << "°, " << vfov_max_d << "°]" << std::endl
            << "  FoV: " << sp.fov_horizontal_deg << "°×" << sp.fov_vertical_deg << "°" << std::endl
            << "  Search: yaw_step=" << sp.yaw_step_deg << "° pitch_step=" << sp.pitch_step_deg
            << "° pitch_range=[" << sp.pitch_min_deg << "°, " << sp.pitch_max_deg
            << "°] weight_pitch=" << sp.weight_pitch << std::endl;

  // ---- Load octomap ----
  OctomapBuilder builder;
  if (!builder.load(prefix)) { std::cerr << "FAILED" << std::endl; return 1; }

  // ---- Project ERP ----
  Eigen::Vector3d origin(px, py, pz);
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  std::cout << "Projecting ERP..." << std::endl;
  ERPImage erp = projectERP(builder, origin, R, h_min, h_max, v_min, v_max, ep);

  // ---- Integral image with wraparound ----
  double fov_w_px = sp.fov_horizontal_deg / res_deg;
  int h_ext = static_cast<int>(std::ceil(fov_w_px));
  IntegralImage ii = IntegralImage::build(erp, h_ext);

  double yaw_step_rad   = sp.yaw_step_deg   * kDeg;
  double pitch_step_rad = sp.pitch_step_deg * kDeg;
  double p_min_rad = sp.pitch_min_deg * kDeg;
  double p_max_rad = sp.pitch_max_deg * kDeg;
  double fov_hw = (sp.fov_horizontal_deg * kDeg) * 0.5;
  double fov_hh = (sp.fov_vertical_deg   * kDeg) * 0.5;
  double fov_w2 = fov_hw / erp.resolution_rad;
  double fov_h2 = fov_hh / erp.resolution_rad;

  // ---- Collect candidates ----
  std::vector<double> yaws, pitches;
  for (double y = -180.0 * kDeg; y < 180.0 * kDeg; y += yaw_step_rad) yaws.push_back(y);
  for (double p = p_min_rad; p <= p_max_rad + 1e-9; p += pitch_step_rad) pitches.push_back(p);

  std::cout << "Searching " << yaws.size() << " yaws × " << pitches.size()
            << " pitches = " << (yaws.size() * pitches.size()) << " candidates..." << std::endl;

  // ---- 2D score grid ----
  struct Cell { double yaw_rad, pitch_rad, score, lambda_min; int N_eff; };
  std::vector<Cell> all_cells;
  all_cells.reserve(yaws.size() * pitches.size());

  double global_best_score = -std::numeric_limits<double>::infinity();
  double global_best_yaw = 0, global_best_pitch = 0;

  for (double yaw : yaws) {
    double u_c = (yaw - erp.h_min_rad) / erp.resolution_rad;
    double u1 = u_c - fov_w2, u2 = u_c + fov_w2;
    if (u1 < 0.0) { u1 += erp.width; u2 += erp.width; }

    for (double pitch : pitches) {
      double v_c = (erp.v_max_rad - pitch) / erp.resolution_rad;
      double v1 = v_c - fov_h2, v2 = v_c + fov_h2;

      int su1 = std::max(0, std::min(static_cast<int>(std::round(u1)), ii.width()-1));
      int su2 = std::max(0, std::min(static_cast<int>(std::round(u2)), ii.width()-1));
      int sv1 = std::max(0, std::min(static_cast<int>(std::round(v1)), ii.height()-1));
      int sv2 = std::max(0, std::min(static_cast<int>(std::round(v2)), ii.height()-1));

      double lm = 0.0, score = 0.0;
      int N = 0;

      if (su2 > su1 && sv2 > sv1) {
        Eigen::Matrix3d S;
        ii.query(su1, su2, sv1, sv2, S, N);
        if (N > 0) {
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(S);
          lm = std::max(0.0, eig.eigenvalues()[0]);
          score = lm - sp.weight_pitch * pitch;
        }
      }

      all_cells.push_back({yaw, pitch, score, lm, N});

      if (score > global_best_score) {
        global_best_score = score;
        global_best_yaw = yaw;
        global_best_pitch = pitch;
      }
    }
  }

  // ---- CSV: full 2D grid ----
  if (!csv_file.empty()) {
    std::ofstream csv(csv_file);
    csv << "yaw_deg,pitch_deg,score,lambda_min,N_eff" << std::endl;
    for (const auto& c : all_cells) {
      csv << c.yaw_rad / kDeg << "," << c.pitch_rad / kDeg << ","
          << c.score << "," << c.lambda_min << "," << c.N_eff << std::endl;
    }
    csv.close();
    std::cout << "CSV saved: " << csv_file << " (" << all_cells.size() << " rows)" << std::endl;
  }

  std::cout << "Global best: yaw=" << global_best_yaw/kDeg << "° pitch=" << global_best_pitch/kDeg
            << "° score=" << global_best_score << std::endl;

  // ---- PPM: score heatmap (yaw=cols, pitch=rows) ----
  if (!ppm_heat.empty()) {
    int cw = static_cast<int>(yaws.size());
    int ch = static_cast<int>(pitches.size());

    double smin = std::numeric_limits<double>::infinity();
    double smax = -std::numeric_limits<double>::infinity();
    for (const auto& c : all_cells) {
      if (c.N_eff > 0) {
        smin = std::min(smin, c.score);
        smax = std::max(smax, c.score);
      }
    }
    if (!std::isfinite(smin)) smin = 0;
    if (smax <= smin) smax = smin + 1;
    std::cout << "Score range: [" << smin << ", " << smax << "]" << std::endl;

    std::ofstream ppm(ppm_heat);
    ppm << "P3\n" << cw << " " << ch << "\n255\n";
    for (int vi = 0; vi < ch; ++vi) {
      int cell_base = vi * cw;
      for (int ui = 0; ui < cw; ++ui) {
        const auto& c = all_cells[cell_base + ui];
        int r, g, b;
        if (c.N_eff == 0) {
          r = 20; g = 20; b = 20;
        } else {
          double t = (c.score - smin) / (smax - smin);
          t = std::max(0.0, std::min(1.0, t));
          if (t < 0.125)      { r = 0;                 g = 0;                  b = 128 + t/0.125 * 127; }
          else if (t < 0.375) { r = 0;                 g = (t-0.125)/0.25 * 255; b = 255; }
          else if (t < 0.625) { r = (t-0.375)/0.25*255; g = 255;               b = 255 - (t-0.375)/0.25*255; }
          else if (t < 0.875) { r = 255;               g = 255 - (t-0.625)/0.25*255; b = 0; }
          else                { r = 255;               g = (t-0.875)/0.125*127; b = 0; }
        }
        ppm << r << " " << g << " " << b << " ";
      }
      ppm << "\n";
    }
    ppm.close();
    std::cout << "PPM heatmap saved: " << ppm_heat << " (" << cw << "×" << ch << ")" << std::endl;
  }

  // ---- Helper: render normal-colored PPM from ERP excerpt ----
  auto writeDepthPPM = [&](const std::string& path, int u_start, int u_len, int v_start, int v_len) {
    if (path.empty() || u_len <= 0 || v_len <= 0) return;
    std::ofstream ppm(path);
    ppm << "P3\n" << u_len << " " << v_len << "\n255\n";
    for (int v = 0; v < v_len; ++v) {
      for (int u = 0; u < u_len; ++u) {
        int src_u = ((u_start + u) % erp.width + erp.width) % erp.width;
        int src_v = v_start + v;
        if (src_v >= 0 && src_v < erp.height) {
          auto n = erp.normalAt(src_u, src_v);
          if (erp.isOccupied(src_u, src_v)) {
            Eigen::Vector3f nv = n;
            if (nv.z() < 0.0f) nv = -nv;
            int r = static_cast<int>((nv.x() + 1.0f) * 0.5f * 255.0f);
            int g = static_cast<int>((nv.y() + 1.0f) * 0.5f * 255.0f);
            int b = static_cast<int>((nv.z() + 1.0f) * 0.5f * 255.0f);
            ppm << r << " " << g << " " << b << " ";
          } else {
            ppm << "0 0 0 ";
          }
        } else {
          ppm << "0 0 0 ";
        }
      }
      ppm << "\n";
    }
    ppm.close();
    std::cout << "Depth PPM saved: " << path << " (" << u_len << "×" << v_len << ")" << std::endl;
  };

  // ---- PPM: full-range depth map ----
  if (!ppm_full.empty()) {
    writeDepthPPM(ppm_full, 0, erp.width, 0, erp.height);
  }

  // ---- PPM: best-view depth map (FoV around best yaw/pitch) ----
  if (!ppm_best.empty()) {
    double u_center = (global_best_yaw - erp.h_min_rad) / erp.resolution_rad;
    double v_center = (erp.v_max_rad - global_best_pitch) / erp.resolution_rad;
    int u_best = static_cast<int>(std::round(u_center));
    int v_best = static_cast<int>(std::round(v_center));
    int fov_w = static_cast<int>(sp.fov_horizontal_deg / res_deg);
    int fov_h = static_cast<int>(sp.fov_vertical_deg   / res_deg);
    int u1 = u_best - fov_w / 2;
    int v1 = v_best - fov_h / 2;
    writeDepthPPM(ppm_best, u1, fov_w, v1, fov_h);
  }

  return 0;
}

#include <iostream>
#include <cmath>

#include "voxel_motion_strategy/octomap_builder.h"
#include "voxel_motion_strategy/erp_projector.h"
#include "voxel_motion_strategy/integral_image.h"
#include "voxel_motion_strategy/rectangle_search.h"
#include "voxel_motion_strategy/yaw_constraint.h"

using namespace voxel_motion_strategy;

static int failures = 0;

#define CHECK(cond, msg) do { \
  if (!(cond)) { std::cerr << "  FAIL: " << msg << std::endl; failures++; } \
  else { std::cout << "  PASS: " << msg << std::endl; } \
} while(0)

static constexpr double kDeg = M_PI / 180.0;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: rectangle_search_test <octomap_prefix> <x> <y> [z]" << std::endl;
    return 1;
  }

  std::string prefix = argv[1];
  double px = std::stod(argv[2]);
  double py = std::stod(argv[3]);
  double pz = (argc > 4) ? std::stod(argv[4]) : 2.0;

  std::cout << "=== RectangleSearch Test ===" << std::endl;
  std::cout << "Octomap: " << prefix << "  Pose: (" << px << ", " << py << ", " << pz << ")" << std::endl << std::endl;

  // ---- Load ----
  OctomapBuilder builder;
  std::cout << "[0] Loading octomap..." << std::endl;
  if (!builder.load(prefix)) { std::cerr << "FAILED" << std::endl; return 1; }

  // ---- Project ERP ----
  Eigen::Vector3d origin(px, py, pz);
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  ERPParams erp_params;
  erp_params.resolution_deg = 2.0;  // 2°/pixel for faster test
  erp_params.range_max_m = 150.0;

  double h_min = -180.0 * kDeg, h_max = 180.0 * kDeg;
  double v_min = -90.0 * kDeg, v_max = 60.0 * kDeg;

  std::cout << "[1] Projecting ERP..." << std::endl;
  ERPImage erp = projectERP(builder, origin, R, h_min, h_max, v_min, v_max, erp_params);

  // ---- Build integral image ----
  double fov_h_deg = 70.4;
  double fov_h_px = fov_h_deg / erp_params.resolution_deg;
  int h_extend = static_cast<int>(std::ceil(fov_h_px));

  std::cout << "[2] Building integral image (h_extend=" << h_extend << ")..." << std::endl;
  IntegralImage ii = IntegralImage::build(erp, h_extend);

  // ---- Test 1: basic search ----
  std::cout << std::endl << "--- Test 1: full-range search ---" << std::endl;
  {
    RectSearchParams sp;
    sp.weight_pitch = 0.0;  // explicitly zero for assertion below
    sp.yaw_step_deg = 10.0;
    sp.pitch_step_deg = 10.0;

    auto result = searchBestRectangle(ii, erp, -180.0 * kDeg, 180.0 * kDeg, sp);
    CHECK(result.valid, "search returns valid result");
    CHECK(result.N_eff > 0, "best rectangle has hits");
    CHECK(std::isfinite(result.best_score), "score is finite");
    // score = λ_min (no N_eff multiplier)
    CHECK(std::abs(result.best_score - result.lambda_min) < 1e-9,
          "score equals λ_min when weight_pitch=0");

    std::cout << "  Best: yaw=" << result.best_yaw_rad / kDeg << "°"
              << " pitch=" << result.best_pitch_rad / kDeg << "°"
              << " λ_min=" << result.lambda_min
              << " N=" << result.N_eff
              << " score=" << result.best_score << std::endl;
  }

  // ---- Test 2: constrained yaw range ----
  std::cout << std::endl << "--- Test 2: constrained yaw range ---" << std::endl;
  {
    YawConstraintParams yp;
    yp.max_angular_velocity = 60.0 * kDeg;
    yp.max_angular_acceleration = 120.0 * kDeg;
    yp.deadzone = 5.0 * kDeg;
    double dt = 0.25;

    // Simulate: currently looking at 50°, moving at 20°/s
    auto yr = computeFeasibleYawRange(50.0 * kDeg, 20.0 * kDeg, dt, yp);
    CHECK(yr.valid, "yaw range is valid");

    RectSearchParams sp;
    sp.yaw_step_deg = 5.0;
    sp.pitch_step_deg = 5.0;

    auto result = searchBestRectangle(ii, erp, yr.min, yr.max, sp);
    CHECK(result.valid, "constrained search returns valid result");
    CHECK(result.best_yaw_rad >= yr.min - 1e-9 && result.best_yaw_rad <= yr.max + 1e-9,
          "best yaw is within feasible range");

    std::cout << "  Yaw range: [" << yr.min/kDeg << "°, " << yr.max/kDeg << "°]" << std::endl;
    std::cout << "  Best: yaw=" << result.best_yaw_rad / kDeg << "°"
              << " pitch=" << result.best_pitch_rad / kDeg << "°"
              << " N=" << result.N_eff << std::endl;
  }

  // ---- Test 3: higher pitch penalty → prefers looking up ----
  std::cout << std::endl << "--- Test 3: pitch penalty effect ---" << std::endl;
  {
    RectSearchParams sp_lo, sp_hi;
    sp_lo.weight_pitch = 0.0;     // no penalty
    sp_hi.weight_pitch = 1.0;     // heavy penalty
    sp_lo.yaw_step_deg = 10.0; sp_lo.pitch_step_deg = 10.0;
    sp_hi.yaw_step_deg = 10.0; sp_hi.pitch_step_deg = 10.0;

    auto r_lo = searchBestRectangle(ii, erp, -180.0 * kDeg, 180.0 * kDeg, sp_lo);
    auto r_hi = searchBestRectangle(ii, erp, -180.0 * kDeg, 180.0 * kDeg, sp_hi);

    CHECK(r_lo.valid && r_hi.valid, "both searches valid");
    // Higher pitch penalty should prefer lower pitch (more negative = looking up)
    CHECK(r_hi.best_pitch_rad <= r_lo.best_pitch_rad + 1e-6,
          "higher penalty → pitch ≤ no-penalty pitch (prefers looking up)");

    std::cout << "  No penalty:    pitch=" << r_lo.best_pitch_rad / kDeg
              << "° score=" << r_lo.best_score << std::endl;
    std::cout << "  Heavy penalty: pitch=" << r_hi.best_pitch_rad / kDeg
              << "° score=" << r_hi.best_score << std::endl;
  }

  // ---- Test 4: empty integral image ----
  std::cout << std::endl << "--- Test 4: edge cases ---" << std::endl;
  {
    IntegralImage empty_ii;
    ERPImage empty_erp;
    RectSearchParams sp;
    auto r = searchBestRectangle(empty_ii, empty_erp, -1.0, 1.0, sp);
    CHECK(!r.valid, "empty integral → invalid result");
  }
  {
    // Zero yaw range
    RectSearchParams sp;
    sp.yaw_step_deg = 10.0;
    auto r = searchBestRectangle(ii, erp, 0.0, 0.0, sp);
    // Should still search the single yaw value at 0°
    CHECK(r.valid, "zero-width yaw range still produces result");
  }

  std::cout << std::endl;
  if (failures == 0) {
    std::cout << "=== ALL TESTS PASSED ===" << std::endl;
  } else {
    std::cout << "=== " << failures << " TEST(S) FAILED ===" << std::endl;
  }
  return failures > 0 ? 1 : 0;
}

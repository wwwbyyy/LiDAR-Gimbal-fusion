#include <iostream>
#include <cmath>
#include <iomanip>

#include "voxel_motion_strategy/yaw_constraint.h"

using namespace voxel_motion_strategy;

static constexpr double kDeg = M_PI / 180.0;
static int failures = 0;

#define CHECK(cond, msg) do { \
  if (!(cond)) { std::cerr << "  FAIL: " << msg << std::endl; failures++; } \
  else { std::cout << "  PASS: " << msg << std::endl; } \
} while(0)

#define CHECK_CLOSE(a, b, tol, msg) do { \
  if (std::abs((a)-(b)) > tol) { \
    std::cerr << "  FAIL: " << msg << " (" << a << " vs " << b << ")" << std::endl; \
    failures++; \
  } else { std::cout << "  PASS: " << msg << std::endl; } \
} while(0)

void printRange(const YawRange& r) {
  std::cout << "  range: [" << r.min / kDeg << "°, " << r.max / kDeg << "°] "
            << "width=" << (r.max - r.min) / kDeg << "° "
            << (r.valid ? "valid" : "INVALID") << std::endl;
}

int main() {
  YawConstraintParams params;
  params.max_angular_velocity = 60.0 * kDeg;       // 60°/s
  params.max_angular_acceleration = 120.0 * kDeg;   // 120°/s²
  params.deadzone = 5.0 * kDeg;                     // 5°
  double dt = 0.25;  // 4 Hz update rate

  std::cout << "=== YawConstraint Test ===" << std::endl;
  std::cout << "Params: w_max=" << params.max_angular_velocity/kDeg << "°/s, "
            << "a_max=" << params.max_angular_acceleration/kDeg << "°/s², "
            << "deadzone=" << params.deadzone/kDeg << "°, dt=" << dt << "s"
            << std::endl << std::endl;

  // ---- Test 1: stationary ----
  std::cout << "--- Test 1: stationary (ω=0, θ=0) ---" << std::endl;
  {
    auto r = computeFeasibleYawRange(0.0, 0.0, dt, params);
    CHECK(r.valid, "stationary range is valid");
    printRange(r);
    // Expected: range = [-½a_max·dt² - dz, +½a_max·dt² + dz]
    // = [-½·120·0.0625 - 5, +½·120·0.0625 + 5] = [-3.75-5, 3.75+5] = [-8.75°, 8.75°]
    double half_a_dt2 = 0.5 * params.max_angular_acceleration * dt * dt;
    CHECK_CLOSE(r.min, -half_a_dt2 - params.deadzone, 1e-6, "stationary min");
    CHECK_CLOSE(r.max, half_a_dt2 + params.deadzone, 1e-6, "stationary max");
    CHECK(r.min < -1.0 * kDeg, "stationary min < -1°");
    CHECK(r.max > 1.0 * kDeg, "stationary max > 1°");
  }

  // ---- Test 2: moving right at half max velocity ----
  std::cout << std::endl << "--- Test 2: moving right (ω=30°/s) ---" << std::endl;
  {
    double w = 30.0 * kDeg;
    auto r = computeFeasibleYawRange(0.0, w, dt, params);
    CHECK(r.valid, "moving right range is valid");
    printRange(r);
    // Base = ω·dt = 30·0.25 = 7.5°
    // α_vel_min = (-60-30)/0.25 = -360, α_vel_max = (60-30)/0.25 = 120
    // α_eff = [-120, 120]  (both within vel bounds)
    // range = [7.5 - 3.75 - 5, 7.5 + 3.75 + 5] = [-1.25°, 16.25°]
    CHECK(r.min < r.max, "range is non-empty");
    CHECK(r.max > r.min + 10.0 * kDeg, "range width > 10° (has room to accelerate/decelerate)");
    // Should be asymmetrical: max > |min| because we're already moving right
    CHECK(r.max > -r.min, "asymmetrical: more room in direction of motion");
  }

  // ---- Test 3: near max velocity ---
  std::cout << std::endl << "--- Test 3: near max velocity (ω=55°/s) ---" << std::endl;
  {
    double w = 55.0 * kDeg;
    auto r = computeFeasibleYawRange(0.0, w, dt, params);
    CHECK(r.valid, "near-max-vel range is valid");
    printRange(r);
    // α_vel_max = (60-55)/0.25 = 20°/s²  → clamped by velocity, not acceleration!
    // α_eff_max = min(120, 20) = 20°/s²
    // So the positive half is much smaller than negative half
    CHECK(r.max - r.min < 30.0 * kDeg, "range narrow near velocity limit");
  }

  // ---- Test 4: at max velocity ---
  std::cout << std::endl << "--- Test 4: at max velocity (ω=60°/s) ---" << std::endl;
  {
    double w = 60.0 * kDeg;
    auto r = computeFeasibleYawRange(0.0, w, dt, params);
    CHECK(r.valid, "at-max-vel range is valid");
    printRange(r);
    // α_vel_max = (60-60)/0.25 = 0 → α_eff_max = 0
    // range max = base + 0 - dz ... actually wait:
    // α_eff_max = min(120, 0) = 0
    // So max = base + 0*half_dt2 + dz = base + dz
    // And min = base + (-120)*half_dt2 - dz = base - 3.75° - 5°
    // → can only decelerate, cannot accelerate further
  }

  // ---- Test 5: deadzone disabled ----
  std::cout << std::endl << "--- Test 5: deadzone=0 ---" << std::endl;
  {
    YawConstraintParams p = params;
    p.deadzone = 0.0;
    auto r0 = computeFeasibleYawRange(0.0, 0.0, dt, p);
    auto r5 = computeFeasibleYawRange(0.0, 0.0, dt, params);
    CHECK(r0.valid && r5.valid, "both ranges valid");
    double width_diff = (r5.max - r5.min) - (r0.max - r0.min);
    CHECK_CLOSE(width_diff, 2.0 * params.deadzone, 1e-6, "deadzone adds 2*dz to width");
  }

  // ---- Test 6: different dt scales ----
  std::cout << std::endl << "--- Test 6: scaling with dt ---" << std::endl;
  {
    auto r_short = computeFeasibleYawRange(0.0, 0.0, 0.1, params);   // 10 Hz
    auto r_long  = computeFeasibleYawRange(0.0, 0.0, 0.5, params);   // 2 Hz
    CHECK(r_short.valid && r_long.valid, "both dt ranges valid");
    // Longer dt → larger range (more time to accelerate)
    double w_short = r_short.max - r_short.min - 2 * params.deadzone;
    double w_long  = r_long.max  - r_long.min  - 2 * params.deadzone;
    CHECK(w_long > w_short, "longer dt gives wider acceleration range");
    CHECK_CLOSE(w_long / w_short, 25.0, 0.5, "width scales with dt² (0.5²/0.1² = 25)");
  }

  // ---- Test 7: angle wraparound handling ----
  std::cout << std::endl << "--- Test 7: near ±π boundary ---" << std::endl;
  {
    // Yaw at 179°, moving right at 30°/s
    double yaw = 179.0 * kDeg;
    double w = 30.0 * kDeg;
    auto r = computeFeasibleYawRange(yaw, w, dt, params);
    CHECK(r.valid, "near-boundary range is valid");
    printRange(r);
    // range should extend past 180° (raw, not wrapped)
    CHECK(r.max > 180.0 * kDeg, "range extends past 180° (raw, caller wraps)");
  }

  // ---- Test 8: negative yaw / velocity ----
  std::cout << std::endl << "--- Test 8: negative motion ---" << std::endl;
  {
    auto r = computeFeasibleYawRange(-30.0 * kDeg, -20.0 * kDeg, dt, params);
    CHECK(r.valid, "negative motion range is valid");
    printRange(r);
    CHECK(r.min < -30.0 * kDeg, "range extends left of current yaw");
  }

  // ---- Edge case: very small dt ----
  std::cout << std::endl << "--- Edge: very small dt (1ms) ---" << std::endl;
  {
    auto r = computeFeasibleYawRange(0.0, 30.0 * kDeg, 0.001, params);
    CHECK(r.valid, "tiny dt range is valid");
    printRange(r);
    // With dt→0, the acceleration-induced range → 0, so width ≈ 2*dz
    double accel_width = (r.max - r.min) - 2 * params.deadzone;
    CHECK(accel_width < 0.1 * kDeg, "accel portion is negligible at tiny dt");
  }

  // ---- Edge case: invalid parameters ----
  std::cout << std::endl << "--- Edge: very large initial velocity ---" << std::endl;
  {
    // ω = 100°/s, but ω_max = 60°/s — this shouldn't happen in practice
    // but the formula should still produce a valid result
    // α_vel_min = (-60-100)/0.25 = -640, α_vel_max = (60-100)/0.25 = -160
    // α_eff_min = max(-120, -640) = -120
    // α_eff_max = min(120, -160) = -160  → invalid!
    // This means the gimbal CAN'T possibly stay within velocity limits.
    // In practice, if the current velocity exceeds ω_max, that's a fault.
    double w_over = 100.0 * kDeg;
    auto r = computeFeasibleYawRange(0.0, w_over, dt, params);
    CHECK(!r.valid, "over-max-velocity returns invalid (fault condition)");
  }

  std::cout << std::endl;
  if (failures == 0) {
    std::cout << "=== ALL TESTS PASSED ===" << std::endl;
  } else {
    std::cout << "=== " << failures << " TEST(S) FAILED ===" << std::endl;
  }
  return failures > 0 ? 1 : 0;
}

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

void printRange(const AngleRange& r) {
  std::cout << "  range: [" << r.min / kDeg << "°, " << r.max / kDeg << "°] "
            << "width=" << (r.max - r.min) / kDeg << "° "
            << (r.valid ? "valid" : "INVALID") << std::endl;
}

int main() {
  double vel_max = 30.0 * kDeg;
  double dt = 0.5;

  std::cout << "=== MotionConstraint Test ===" << std::endl;
  std::cout << "Params: vel_max=" << vel_max/kDeg << "°/s, dt=" << dt << "s"
            << std::endl << std::endl;

  // Test 1: stationary
  std::cout << "--- Test 1: stationary (θ=0) ---" << std::endl;
  {
    auto r = computeAngleRange(0.0, vel_max, dt);
    CHECK(r.valid, "stationary range is valid");
    printRange(r);
    CHECK_CLOSE(r.min, -vel_max * dt, 1e-9, "min = -vel*dt");
    CHECK_CLOSE(r.max,  vel_max * dt, 1e-9, "max = +vel*dt");
  }

  // Test 2: from non-zero angle
  std::cout << std::endl << "--- Test 2: from θ=10° ---" << std::endl;
  {
    double yaw = 10.0 * kDeg;
    auto r = computeAngleRange(yaw, vel_max, dt);
    CHECK(r.valid, "range from 10° is valid");
    printRange(r);
    CHECK_CLOSE(r.min, yaw - vel_max * dt, 1e-6, "min = yaw - vel*dt");
    CHECK_CLOSE(r.max, yaw + vel_max * dt, 1e-6, "max = yaw + vel*dt");
  }

  // Test 3: clamping
  std::cout << std::endl << "--- Test 3: clamping to [-10°, 10°] ---" << std::endl;
  {
    double cmin = -10.0 * kDeg, cmax = 10.0 * kDeg;
    auto r = computeAngleRange(0.0, vel_max, dt, cmin, cmax);
    CHECK(r.valid, "clamped range is valid");
    printRange(r);
    CHECK_CLOSE(r.min, cmin, 1e-6, "min clamped");
    CHECK_CLOSE(r.max, cmax, 1e-6, "max clamped");
  }

  // Test 4: clamping makes range invalid
  std::cout << std::endl << "--- Test 4: clamping outside feasible ---" << std::endl;
  {
    auto r = computeAngleRange(50.0 * kDeg, vel_max, dt, -5.0 * kDeg, 5.0 * kDeg);
    CHECK(!r.valid, "range invalid when clamp excludes current angle");
  }

  // Test 5: near ±π boundary (unclamped)
  std::cout << std::endl << "--- Test 5: near ±π boundary ---" << std::endl;
  {
    double yaw = 179.0 * kDeg;
    auto r = computeAngleRange(yaw, vel_max, dt);
    CHECK(r.valid, "near-boundary range is valid");
    printRange(r);
    CHECK(r.max > 180.0 * kDeg, "range extends past 180° (caller wraps)");
  }

  // Test 6: negative angle
  std::cout << std::endl << "--- Test 6: negative angle ---" << std::endl;
  {
    auto r = computeAngleRange(-30.0 * kDeg, vel_max, dt);
    CHECK(r.valid, "negative angle range is valid");
    printRange(r);
    CHECK(r.min < -30.0 * kDeg, "range extends below current angle");
  }

  // Test 7: linear scaling with dt
  std::cout << std::endl << "--- Test 7: scaling with dt ---" << std::endl;
  {
    auto r_short = computeAngleRange(0.0, vel_max, 0.1);
    auto r_long  = computeAngleRange(0.0, vel_max, 0.5);
    CHECK(r_short.valid && r_long.valid, "both dt ranges valid");
    double ratio = (r_long.max - r_long.min) / (r_short.max - r_short.min);
    CHECK_CLOSE(ratio, 5.0, 0.01, "width scales linearly with dt");
  }

  // Test 8: tilt-like use case (clamped)
  std::cout << std::endl << "--- Test 8: tilt constraint (vel=20°/s, clamp [-15°,15°]) ---" << std::endl;
  {
    double tv = 20.0 * kDeg;
    double tmin = -15.0 * kDeg, tmax = 15.0 * kDeg;
    auto r = computeAngleRange(5.0 * kDeg, tv, dt, tmin, tmax);
    CHECK(r.valid, "tilt range is valid");
    printRange(r);
    CHECK_CLOSE(r.min, -5.0 * kDeg, 1e-6, "tilt min: 5° - 10° = -5°");
    CHECK_CLOSE(r.max, tmax, 1e-6, "tilt max clamped");
  }

  std::cout << std::endl;
  if (failures == 0) {
    std::cout << "=== ALL TESTS PASSED ===" << std::endl;
  } else {
    std::cout << "=== " << failures << " TEST(S) FAILED ===" << std::endl;
  }
  return failures > 0 ? 1 : 0;
}

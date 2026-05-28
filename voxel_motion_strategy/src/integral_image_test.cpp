#include <iostream>
#include <cmath>
#include <iomanip>

#include "voxel_motion_strategy/integral_image.h"

using namespace voxel_motion_strategy;

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

// Build a small synthetic ERP for testing
ERPImage makeTestERP() {
  ERPImage img;
  img.width = 5;
  img.height = 3;

  // Row 0 (top):    all empty
  // Row 1 (middle): alternating normals (1,0,0) and (0,1,0)
  // Row 2 (bottom): all ground (0,0,1)
  size_t n = 15;
  img.normals.assign(n, Eigen::Vector3f::Zero());
  img.occupied.assign(n, 0);

  // row 0: all empty → stays (0,0,0), occupied=0

  // row 1: alternating
  for (int u = 0; u < 5; ++u) {
    int idx = 1 * 5 + u;
    img.occupied[idx] = 1;
    img.normals[idx] = (u % 2 == 0)
        ? Eigen::Vector3f(1, 0, 0)   // X-wall
        : Eigen::Vector3f(0, 1, 0);  // Y-wall
  }

  // row 2: all ground
  for (int u = 0; u < 5; ++u) {
    int idx = 2 * 5 + u;
    img.occupied[idx] = 1;
    img.normals[idx] = Eigen::Vector3f(0, 0, 1);
  }

  img.h_min_rad = 0;
  img.h_max_rad = 5.0 * M_PI / 180.0;
  img.v_min_rad = 0;
  img.v_max_rad = 3.0 * M_PI / 180.0;
  img.resolution_rad = M_PI / 180.0;
  return img;
}

int main() {
  std::cout << "=== IntegralImage Test ===" << std::endl << std::endl;

  // ---- Test 1: basic build ----
  std::cout << "--- Test 1: build from synthetic ERP ---" << std::endl;
  ERPImage erp = makeTestERP();
  IntegralImage ii = IntegralImage::build(erp, 0);
  CHECK(ii.width() == 6, "SAT width = img_w + 1 = 6");   // 5 + 1
  CHECK(ii.height() == 4, "SAT height = img_h + 1 = 4");  // 3 + 1

  // ---- Test 2: query single pixel ----
  std::cout << std::endl << "--- Test 2: single-pixel queries ---" << std::endl;
  {
    // Pixel (0,1) = X-wall normal (1,0,0)
    Eigen::Matrix3d S;
    int N;
    ii.query(0, 1, 1, 2, S, N);
    // Expected: n·nᵀ = diag(1,0,0), N=1
    CHECK_CLOSE(S(0,0), 1.0, 1e-9, "X-wall S(0,0)=1");
    CHECK_CLOSE(S(1,1), 0.0, 1e-9, "X-wall S(1,1)=0");
    CHECK_CLOSE(S(2,2), 0.0, 1e-9, "X-wall S(2,2)=0");
    CHECK(N == 1, "X-wall N=1");
  }
  {
    // Pixel (1,1) = Y-wall normal (0,1,0)
    Eigen::Matrix3d S; int N;
    ii.query(1, 2, 1, 2, S, N);
    CHECK_CLOSE(S(0,0), 0.0, 1e-9, "Y-wall S(0,0)=0");
    CHECK_CLOSE(S(1,1), 1.0, 1e-9, "Y-wall S(1,1)=1");
    CHECK(N == 1, "Y-wall N=1");
  }
  {
    // Pixel (0,2) = ground normal (0,0,1)
    Eigen::Matrix3d S; int N;
    ii.query(0, 1, 2, 3, S, N);
    CHECK_CLOSE(S(2,2), 1.0, 1e-9, "Ground S(2,2)=1");
    CHECK_CLOSE(S(0,0), 0.0, 1e-9, "Ground S(0,0)=0");
    CHECK(N == 1, "Ground N=1");
  }
  {
    // Pixel (0,0) = empty → S=0, N=0
    Eigen::Matrix3d S; int N;
    ii.query(0, 1, 0, 1, S, N);
    CHECK_CLOSE(S.trace(), 0.0, 1e-9, "Empty pixel S=0");
    CHECK(N == 0, "Empty pixel N=0");
  }

  // ---- Test 3: multi-pixel rectangle ----
  std::cout << std::endl << "--- Test 3: multi-pixel rectangles ---" << std::endl;
  {
    // Row 1, cols 0-1: X+Y walls → S = diag(1,1,0), N=2
    Eigen::Matrix3d S; int N;
    ii.query(0, 2, 1, 2, S, N);
    CHECK_CLOSE(S(0,0), 1.0, 1e-9, "2-pixel S(0,0)=1");
    CHECK_CLOSE(S(1,1), 1.0, 1e-9, "2-pixel S(1,1)=1");
    CHECK_CLOSE(S(2,2), 0.0, 1e-9, "2-pixel S(2,2)=0");
    CHECK(N == 2, "2-pixel N=2");
  }
  {
    // Entire image: row0 empty + row1 (3X, 2Y) + row2 (5 ground)
    // S = diag(3, 2, 5) = [3 0 0; 0 2 0; 0 0 5]
    // N = 0 + 5 + 5 = 10
    Eigen::Matrix3d S; int N;
    ii.query(0, 5, 0, 3, S, N);
    CHECK_CLOSE(S(0,0), 3.0, 1e-9, "full S(0,0)=3");
    CHECK_CLOSE(S(1,1), 2.0, 1e-9, "full S(1,1)=2");
    CHECK_CLOSE(S(2,2), 5.0, 1e-9, "full S(2,2)=5");
    CHECK_CLOSE(S(0,1), 0.0, 1e-9, "full S off-diagonal=0");
    CHECK(N == 10, "full N=10");
  }

  // ---- Test 4: S is PSD ----
  std::cout << std::endl << "--- Test 4: S is positive semi-definite ---" << std::endl;
  {
    // All wall normals in row 1 → S = diag(3, 2, 0), λ = {3,2,0}
    Eigen::Matrix3d S; int N;
    ii.query(0, 5, 1, 2, S, N);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(S);
    CHECK(eig.eigenvalues()[0] >= -1e-9, "λ_min >= 0");
    CHECK(N == 5, "N=5");
  }

  // ---- Test 5: wraparound extension ----
  std::cout << std::endl << "--- Test 5: horizontal extension ---" << std::endl;
  {
    // Build with extension of 2 pixels
    IntegralImage ii_ext = IntegralImage::build(erp, 2);
    CHECK(ii_ext.width() == 8, "extended SAT width = 5+2+1 = 8");

    // Query in the extended region: cols [5,7) should be copies of [0,2)
    Eigen::Matrix3d S1, S2; int N1, N2;
    ii_ext.query(0, 2, 2, 3, S1, N1);  // original cols 0-1
    ii_ext.query(5, 7, 2, 3, S2, N2); // extended cols (copies of 0-1)
    CHECK_CLOSE(S1(0,0), S2(0,0), 1e-9, "extended region matches original S(0,0)");
    CHECK(N1 == N2, "extended region N matches");
  }

  // ---- Test 6: symmetric S ----
  std::cout << std::endl << "--- Test 6: S is symmetric ---" << std::endl;
  {
    // Build with normals that have cross-terms
    ERPImage erp2;
    erp2.width = 3; erp2.height = 2;
    erp2.normals.resize(6, Eigen::Vector3f::Zero());
    erp2.occupied.resize(6, 1);
    erp2.normals[0] = Eigen::Vector3f( 0.5,  0.5,  std::sqrt(0.5)).normalized();
    erp2.normals[1] = Eigen::Vector3f(-0.3,  0.8,  0.0).normalized();
    erp2.normals[2] = Eigen::Vector3f( 0.0,  0.0,  1.0);
    erp2.normals[3] = Eigen::Vector3f( 1.0,  0.0,  0.0);
    erp2.normals[4] = Eigen::Vector3f( 0.0,  0.6,  0.8).normalized();
    erp2.normals[5] = Eigen::Vector3f( 0.7, -0.7,  0.0).normalized();
    erp2.h_min_rad = 0; erp2.h_max_rad = 3.0 * M_PI/180;
    erp2.v_min_rad = 0; erp2.v_max_rad = 2.0 * M_PI/180;
    erp2.resolution_rad = M_PI/180;

    IntegralImage ii2 = IntegralImage::build(erp2, 0);
    Eigen::Matrix3d S; int N;
    ii2.query(0, 3, 0, 2, S, N);

    CHECK_CLOSE(S(0,1), S(1,0), 1e-9, "S(0,1) == S(1,0)");
    CHECK_CLOSE(S(0,2), S(2,0), 1e-9, "S(0,2) == S(2,0)");
    CHECK_CLOSE(S(1,2), S(2,1), 1e-9, "S(1,2) == S(2,1)");
    std::cout << "  S = [" << S(0,0) << " " << S(0,1) << " " << S(0,2) << "; "
              << S(1,0) << " " << S(1,1) << " " << S(1,2) << "; "
              << S(2,0) << " " << S(2,1) << " " << S(2,2) << "]" << std::endl;
    std::cout << "  N = " << N << " (expected 6)" << std::endl;
    CHECK(N == 6, "N=6");
  }

  // ---- Test 7: empty ERP ----
  std::cout << std::endl << "--- Test 7: edge cases ---" << std::endl;
  {
    ERPImage empty;
    IntegralImage ii0 = IntegralImage::build(empty, 0);
    CHECK(ii0.width() == 0 && ii0.height() == 0, "empty ERP → empty SAT");
  }
  {
    // Clamped query on empty
    IntegralImage ii0;
    Eigen::Matrix3d S; int N;
    ii0.query(0, 1, 0, 1, S, N);
    CHECK_CLOSE(S.trace(), 0.0, 1e-9, "empty SAT query returns S=0");
    CHECK(N == 0, "empty SAT query returns N=0");
  }

  // ---- Test 8: large rectangle equals sum of small rectangles ----
  std::cout << std::endl << "--- Test 8: region additivity ---" << std::endl;
  {
    ERPImage erp = makeTestERP();
    IntegralImage ii = IntegralImage::build(erp, 0);

    Eigen::Matrix3d S_full, S_a, S_b; int N_full, N_a, N_b;
    ii.query(1, 4, 1, 3, S_full, N_full);       // cols 1-3, rows 1-2
    ii.query(1, 2, 1, 3, S_a, N_a);              // left half
    ii.query(2, 4, 1, 3, S_b, N_b);              // right half

    CHECK_CLOSE(S_full(0,0), S_a(0,0) + S_b(0,0), 1e-9, "additivity S(0,0)");
    CHECK_CLOSE(S_full(1,1), S_a(1,1) + S_b(1,1), 1e-9, "additivity S(1,1)");
    CHECK(N_full == N_a + N_b, "additivity N");
  }

  std::cout << std::endl;
  if (failures == 0) {
    std::cout << "=== ALL TESTS PASSED ===" << std::endl;
  } else {
    std::cout << "=== " << failures << " TEST(S) FAILED ===" << std::endl;
  }
  return failures > 0 ? 1 : 0;
}

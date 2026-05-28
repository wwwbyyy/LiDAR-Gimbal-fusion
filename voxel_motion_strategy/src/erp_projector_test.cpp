#include <iostream>
#include <cmath>
#include <iomanip>

#include "voxel_motion_strategy/octomap_builder.h"
#include "voxel_motion_strategy/erp_projector.h"

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

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: erp_projector_test <octomap_prefix> <test_pose_x> <test_pose_y> [test_pose_z]" << std::endl;
    std::cerr << "  octomap_prefix: path prefix for .ot + .normals files" << std::endl;
    std::cerr << "  test_pose: map-frame position to project from" << std::endl;
    return 1;
  }

  std::string prefix = argv[1];
  double px = std::stod(argv[2]);
  double py = std::stod(argv[3]);
  double pz = (argc > 4) ? std::stod(argv[4]) : 0.0;

  std::cout << "=== ERPProjector Test ===" << std::endl;
  std::cout << "Octomap: " << prefix << "  Pose: (" << px << ", " << py << ", " << pz << ")" << std::endl << std::endl;

  // ---- Load octomap ----
  OctomapBuilder builder;
  std::cout << "[0] Loading octomap..." << std::endl;
  if (!builder.load(prefix)) {
    std::cerr << "FAILED to load octomap. Run octomap_builder_test first." << std::endl;
    return 1;
  }
  std::cout << "  Loaded " << builder.numVoxels() << " voxels." << std::endl << std::endl;

  Eigen::Vector3d origin(px, py, pz);
  Eigen::Matrix3d R_erp_to_map = Eigen::Matrix3d::Identity();
  // ERP: x=forward, y=left, z=up. With identity rotation, ERP forward = map +X.

  ERPParams params;
  params.resolution_deg = 1.0;
  params.range_max_m = 150.0;

  // ---- Test 1: full 360° projection ----
  std::cout << "--- Test 1: full 360°x150°, 1°/pixel ---" << std::endl;
  {
    double h_min = -180.0 * kDeg;
    double h_max =  180.0 * kDeg;
    double v_min = -90.0 * kDeg;
    double v_max =  60.0 * kDeg;

    ERPImage img = projectERP(builder, origin, R_erp_to_map,
                               h_min, h_max, v_min, v_max, params);
    CHECK(!img.empty(), "image not empty");
    CHECK(img.width == 360, "width = 360 pixels");
    CHECK(img.height == 150, "height = 150 pixels");
    CHECK(img.pixelCount() == 360 * 150, "54000 total pixels");

    // Count hits
    size_t hits = 0, misses = 0;
    for (size_t i = 0; i < img.pixelCount(); ++i) {
      if (img.occupied[i]) hits++; else misses++;
    }
    std::cout << "  Hits: " << hits << " (" << 100.0 * hits / img.pixelCount()
              << "%), Misses: " << misses << std::endl;
    CHECK(hits > 0, "at least some rays hit geometry");

    // Verify all hit normals are unit vectors
    int bad_normals = 0;
    for (size_t i = 0; i < img.pixelCount(); ++i) {
      if (img.occupied[i]) {
        float n = img.normals[i].norm();
        if (n < 0.99f || n > 1.01f) bad_normals++;
      }
    }
    CHECK(bad_normals == 0, "all hit normals are unit vectors");

    // Verify occupied[] is consistent with normals
    int inconsistent = 0;
    for (size_t i = 0; i < img.pixelCount(); ++i) {
      bool is_zero = img.normals[i].isZero();
      bool is_occ = img.occupied[i] != 0;
      if (is_zero == is_occ) inconsistent++;  // (0,0,0) should NOT be occupied
    }
    CHECK(inconsistent == 0, "occupied flag consistent with normal (0,0,0)");
  }

  // ---- Test 2: horizontal pruning (narrow yaw range) ----
  std::cout << std::endl << "--- Test 2: pruned horizontal range ---" << std::endl;
  {
    double h_min = -10.0 * kDeg;
    double h_max =  20.0 * kDeg;
    double v_min = -90.0 * kDeg;
    double v_max =  60.0 * kDeg;

    ERPImage img = projectERP(builder, origin, R_erp_to_map,
                               h_min, h_max, v_min, v_max, params);
    CHECK(!img.empty(), "pruned image not empty");
    CHECK(img.width == 30, "width = 30 pixels (30° span)");
    CHECK(img.height == 150, "height = 150 pixels");

    size_t hits = 0;
    for (size_t i = 0; i < img.pixelCount(); ++i)
      if (img.occupied[i]) hits++;
    std::cout << "  Hits: " << hits << " / " << img.pixelCount() << std::endl;
    CHECK(hits > 0, "pruned projection has hits");
  }

  // ---- Test 3: consistency - same pose, same params, same result ----
  std::cout << std::endl << "--- Test 3: deterministic output ---" << std::endl;
  {
    auto img1 = projectERP(builder, origin, R_erp_to_map,
                           -5.0 * kDeg, 5.0 * kDeg,
                           -10.0 * kDeg, 10.0 * kDeg, params);
    auto img2 = projectERP(builder, origin, R_erp_to_map,
                           -5.0 * kDeg, 5.0 * kDeg,
                           -10.0 * kDeg, 10.0 * kDeg, params);
    CHECK(img1.width == img2.width && img1.height == img2.height,
          "same dimensions on repeated call");

    int mismatches = 0;
    for (size_t i = 0; i < img1.pixelCount(); ++i) {
      if (img1.occupied[i] != img2.occupied[i]) mismatches++;
      else if (img1.occupied[i] &&
               (img1.normals[i] - img2.normals[i]).norm() > 1e-6f) mismatches++;
    }
    CHECK(mismatches == 0, "identical output on repeated call (deterministic)");
  }

  // ---- Test 4: different pose → different hits ----
  std::cout << std::endl << "--- Test 4: different poses give different results ---" << std::endl;
  {
    Eigen::Vector3d origin2(px + 50.0, py, pz);
    auto img1 = projectERP(builder, origin, R_erp_to_map,
                           -10.0 * kDeg, 10.0 * kDeg,
                           -10.0 * kDeg, 10.0 * kDeg, params);
    auto img2 = projectERP(builder, origin2, R_erp_to_map,
                           -10.0 * kDeg, 10.0 * kDeg,
                           -10.0 * kDeg, 10.0 * kDeg, params);

    // Different poses should see different geometry
    int same_count = 0;
    for (size_t i = 0; i < img1.pixelCount(); ++i) {
      if (img1.occupied[i] == img2.occupied[i] &&
          (!img1.occupied[i] || (img1.normals[i] - img2.normals[i]).norm() < 1e-4f))
        same_count++;
    }
    double same_pct = 100.0 * same_count / img1.pixelCount();
    std::cout << "  Pixels identical across 50m shift: " << same_count
              << " / " << img1.pixelCount() << " (" << same_pct << "%)" << std::endl;
    CHECK(same_pct < 95.0, "shifting pose changes projection (not all identical)");
  }

  // ---- Test 5: empty range → empty image ----
  std::cout << std::endl << "--- Test 5: edge cases ---" << std::endl;
  {
    auto img = projectERP(builder, origin, R_erp_to_map,
                           0.0, 0.0, 0.0, 0.0, params);
    CHECK(img.empty(), "zero angular range → empty image");
  }

  // ---- Test 6: rotated ERP frame ----
  std::cout << std::endl << "--- Test 6: rotated ERP frame ---" << std::endl;
  {
    // Rotate ERP 90° around Z: ERP forward → map +Y
    Eigen::Matrix3d R90 = Eigen::AngleAxisd(90.0 * kDeg, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    auto img_rot = projectERP(builder, origin, R90,
                               -5.0 * kDeg, 5.0 * kDeg,
                               -5.0 * kDeg, 5.0 * kDeg, params);
    CHECK(!img_rot.empty(), "rotated projection succeeds");
    CHECK(img_rot.width > 0 && img_rot.height > 0, "rotated image has valid dimensions");
  }

  // ---- Test 7: empty builder ----
  std::cout << std::endl << "--- Test 7: empty builder ---" << std::endl;
  {
    OctomapBuilder empty;
    auto img = projectERP(empty, origin, R_erp_to_map,
                           -10.0 * kDeg, 10.0 * kDeg,
                           -10.0 * kDeg, 10.0 * kDeg, params);
    CHECK(img.empty(), "empty builder → empty image");
  }

  // ---- Summary ----
  std::cout << std::endl;
  if (failures == 0) {
    std::cout << "=== ALL TESTS PASSED ===" << std::endl;
  } else {
    std::cout << "=== " << failures << " TEST(S) FAILED ===" << std::endl;
  }
  return failures > 0 ? 1 : 0;
}

#include <iostream>
#include <cstdlib>

#include "voxel_motion_strategy/octomap_builder.h"

int main(int argc, char** argv) {
  std::string points_file, normals_file, save_prefix;
  double resolution = 1.0, range_max = 150.0;

  if (argc < 4) {
    std::cerr << "Usage: octomap_builder_test <points.ply> <normals.ply> <save_prefix> [--voxel N] [--range M]" << std::endl;
    return 1;
  }

  points_file = argv[1];
  normals_file = argv[2];
  save_prefix = argv[3];
  for (int i = 4; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--voxel" && i + 1 < argc) resolution = std::stod(argv[++i]);
    else if (arg == "--range" && i + 1 < argc) range_max = std::stod(argv[++i]);
  }

  std::cout << "=== OctomapBuilder Test ===" << std::endl;
  std::cout << "Points: " << points_file << "  Normals: " << normals_file << std::endl;
  std::cout << "Save: " << save_prefix << "  Voxel: " << resolution << "m  Range: " << range_max << "m" << std::endl;
  std::cout << std::endl;

  // ---------- 1. Build ----------
  voxel_motion_strategy::OctomapBuilder builder;
  std::cout << "[1/4] Building..." << std::endl;
  if (!builder.buildFromPLY(points_file, normals_file, resolution, range_max)) {
    std::cerr << "FAILED: buildFromPLY" << std::endl;
    return 1;
  }

  size_t n_voxels = builder.numVoxels();
  if (n_voxels == 0) {
    std::cerr << "FAILED: 0 voxels built" << std::endl;
    return 1;
  }
  std::cout << "  Built " << n_voxels << " occupied voxels." << std::endl;

  // Sanity: query all occupied leaves, check normal validity
  {
    int hit = 0, miss = 0, bad_normal = 0, bad_count = 0;
    int max_check = 500;
    for (auto it = builder.tree()->begin_leafs(), end = builder.tree()->end_leafs();
         it != end; ++it) {
      if (!builder.tree()->isNodeOccupied(*it)) continue;
      Eigen::Vector3f n;
      int cnt;
      if (builder.getNormalAt(it.getKey(), n, cnt)) {
        hit++;
        float norm = n.norm();
        if (norm < 0.99f || norm > 1.01f) bad_normal++;
        if (cnt <= 0) bad_count++;
      } else {
        miss++;
      }
      if (--max_check <= 0) break;
    }
    std::cout << "  Query sanity: " << hit << " hits, " << miss << " misses, "
              << bad_normal << " bad normals, " << bad_count << " bad counts"
              << (bad_normal == 0 && bad_count == 0 ? " (OK)" : " (WARN)") << std::endl;
    if (miss > 0) {
      std::cout << "  Note: " << miss << " leaves have no normal data (pruned inner nodes)." << std::endl;
    }
  }

  // ---------- 2. Save ----------
  std::cout << "[2/4] Saving..." << std::endl;
  if (!builder.save(save_prefix)) {
    std::cerr << "FAILED: save" << std::endl;
    return 1;
  }

  // ---------- 3. Load ----------
  voxel_motion_strategy::OctomapBuilder builder2;
  std::cout << "[3/4] Loading..." << std::endl;
  if (!builder2.load(save_prefix)) {
    std::cerr << "FAILED: load" << std::endl;
    return 1;
  }
  if (builder2.numVoxels() != n_voxels) {
    std::cerr << "FAILED: voxel count mismatch: " << builder2.numVoxels() << " vs " << n_voxels << std::endl;
    return 1;
  }
  if (std::abs(builder2.resolution() - resolution) > 1e-6) {
    std::cerr << "FAILED: resolution mismatch: " << builder2.resolution() << " vs " << resolution << std::endl;
    return 1;
  }
  std::cout << "  Loaded " << builder2.numVoxels() << " voxels (matches build)." << std::endl;

  // ---------- 4. Round-trip spot-check ----------
  std::cout << "[4/4] Round-trip check..." << std::endl;
  {
    int checked = 0, match = 0, mismatch = 0;
    for (auto it = builder2.tree()->begin_leafs(), end = builder2.tree()->end_leafs();
         it != end; ++it) {
      if (!builder2.tree()->isNodeOccupied(*it)) continue;
      octomap::point3d coord = builder2.tree()->keyToCoord(it.getKey());
      Eigen::Vector3f n1, n2;
      int c1, c2;
      bool h1 = builder.getNormal(coord, n1, c1);
      bool h2 = builder2.getNormal(coord, n2, c2);
      if (h1 == h2 && h1) {
        if ((n1 - n2).norm() < 1e-4f && c1 == c2) match++;
        else mismatch++;
      }
      if (++checked >= 200) break;
    }
    std::cout << "  " << checked << " checked, " << match << " match, " << mismatch << " mismatch"
              << (mismatch == 0 ? " (PASS)" : " (FAIL)") << std::endl;
    if (mismatch > 0) return 1;
  }

  // --- Edge cases ---
  std::cout << std::endl << "--- Edge cases ---" << std::endl;

  // Query far point
  {
    Eigen::Vector3f n; int c;
    octomap::point3d far(1e9f, 1e9f, 1e9f);
    std::cout << "  far point: " << (builder2.getNormal(far, n, c) ? "hit (UNEXPECTED)" : "miss (OK)") << std::endl;
  }

  // Query near a known leaf center
  {
    auto it = builder2.tree()->begin_leafs();
    for (int i = 0; i < 10; ++i) ++it;
    octomap::point3d coord = builder2.tree()->keyToCoord(it.getKey());
    Eigen::Vector3f n; int c;
    bool hit = builder2.getNormal(coord, n, c);
    std::cout << "  known voxel center: " << (hit ? "hit (OK)" : "miss (UNEXPECTED)")
              << " n=(" << n.x() << "," << n.y() << "," << n.z() << ") cnt=" << c << std::endl;
  }

  // Query slightly inside a known voxel (offset by half voxel)
  {
    auto it = builder2.tree()->begin_leafs();
    for (int i = 0; i < 10; ++i) ++it;
    octomap::point3d coord = builder2.tree()->keyToCoord(it.getKey());
    coord.x() += static_cast<float>(resolution * 0.3);
    Eigen::Vector3f n; int c;
    bool hit = builder2.getNormal(coord, n, c);
    std::cout << "  offset inside voxel: " << (hit ? "hit (OK)" : "miss (UNEXPECTED)") << std::endl;
  }

  // Empty builder save
  {
    voxel_motion_strategy::OctomapBuilder empty;
    std::cout << "  empty save: " << (empty.save("/tmp/_octomap_test_empty_") ? "saved (UNEXPECTED)" : "false (OK)") << std::endl;
  }

  // Load non-existent file
  {
    voxel_motion_strategy::OctomapBuilder dummy;
    std::cout << "  load nonexistent: " << (dummy.load("/tmp/_octomap_nonexistent_") ? "loaded (UNEXPECTED)" : "false (OK)") << std::endl;
  }

  std::cout << std::endl << "=== ALL TESTS PASSED ===" << std::endl;
  return 0;
}

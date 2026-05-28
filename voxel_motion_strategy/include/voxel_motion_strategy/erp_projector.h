#pragma once

#include <cstdint>
#include <vector>
#include <Eigen/Dense>

namespace voxel_motion_strategy {

class OctomapBuilder;

struct ERPImage {
  int width = 0;   // horizontal pixels
  int height = 0;  // vertical pixels

  // Per-pixel data, row-major: pixel(u,v) at index v * width + u
  // u=0 → h_min, u=width-1 → h_max
  // v=0 → v_max (top row), v=height-1 → v_min (bottom row)
  std::vector<Eigen::Vector3f> normals;   // (0,0,0) if no hit
  std::vector<uint8_t> occupied;           // 1 = hit, 0 = miss

  // Angular bounds (radians) of this image
  double h_min_rad = 0, h_max_rad = 0;   // horizontal (azimuth)
  double v_min_rad = 0, v_max_rad = 0;   // vertical (elevation)
  double resolution_rad = 0;

  // Convenience
  bool empty() const { return width == 0 || height == 0; }
  size_t pixelCount() const { return static_cast<size_t>(width) * height; }

  Eigen::Vector3f normalAt(int u, int v) const {
    return normals[v * width + u];
  }
  bool isOccupied(int u, int v) const {
    return occupied[v * width + u] != 0;
  }
  // Pixel index for integral image (u, v within bounds)
  int index(int u, int v) const { return v * width + u; }
};

struct ERPParams {
  double resolution_deg = 1.0;      // degrees per pixel
  double range_max_m = 150.0;       // max ray distance
};

/// Project occupied voxels from an Octomap into an equirectangular image.
///
/// origin_map:       ray origin in map frame (e.g. vehicle position)
/// R_erp_to_map:     3x3 rotation from ERP spherical frame to map frame.
///                   ERP frame convention:
///                     θ=0, φ=0  →  +X (forward in map, after rotation)
///                     θ=π/2     →  +Y (left)
///                     φ=π/2     →  +Z (up)
/// h_range_rad:      [h_min, h_max] azimuth range to project
/// v_range_rad:      [v_min, v_max] elevation range to project
///
/// For each pixel (θ, φ), a ray is cast through the octree.
/// The first occupied voxel's normal is recorded.
/// No hit → normal = (0,0,0), occupied = 0.
ERPImage projectERP(const OctomapBuilder& builder,
                     const Eigen::Vector3d& origin_map,
                     const Eigen::Matrix3d& R_erp_to_map,
                     double h_min_rad, double h_max_rad,
                     double v_min_rad, double v_max_rad,
                     const ERPParams& params = ERPParams());

}  // namespace voxel_motion_strategy

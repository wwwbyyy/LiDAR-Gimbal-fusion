#include "voxel_motion_strategy/erp_projector.h"
#include "voxel_motion_strategy/octomap_builder.h"

#include <cmath>
#include <iostream>
#include <algorithm>

#include <octomap/OcTree.h>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace voxel_motion_strategy {

bool g_verbose = true;

ERPImage projectERP(const OctomapBuilder& builder,
                     const Eigen::Vector3d& origin_map,
                     const Eigen::Matrix3d& R_erp_to_map,
                     double h_min_rad, double h_max_rad,
                     double v_min_rad, double v_max_rad,
                     const ERPParams& params) {
  ERPImage img;  // default: empty (width=0, height=0)

  const octomap::OcTree* tree = builder.tree();
  if (!tree) {
    std::cerr << "[ERP] ERROR: builder has no tree" << std::endl;
    return img;
  }

  img.resolution_rad = params.resolution_deg * M_PI / 180.0;

  const double res = img.resolution_rad;
  const double range_max = params.range_max_m;
  const double self_range = params.self_range_m;

  // Image dimensions
  double h_span = h_max_rad - h_min_rad;
  double v_span = v_max_rad - v_min_rad;
  if (h_span <= 0 || v_span <= 0) {
    std::cerr << "[ERP] ERROR: invalid angular range" << std::endl;
    return img;
  }

  img.width  = static_cast<int>(std::ceil(h_span / res));
  img.height = static_cast<int>(std::ceil(v_span / res));
  img.h_min_rad = h_min_rad;
  img.h_max_rad = h_min_rad + img.width * res;   // actual (slightly larger)
  img.v_min_rad = v_min_rad;
  img.v_max_rad = v_min_rad + img.height * res;

  size_t npix = img.pixelCount();
  img.normals.assign(npix, Eigen::Vector3f::Zero());
  img.occupied.assign(npix, 0);

  // Cast origin to octomap point
  octomap::point3d origin(static_cast<float>(origin_map.x()),
                           static_cast<float>(origin_map.y()),
                           static_cast<float>(origin_map.z()));

  if (g_verbose) {
    std::cout << "[ERP] Projecting " << img.width << "x" << img.height
              << " (" << npix << " pixels), h=[" << h_min_rad * 180.0 / M_PI
              << "°, " << h_max_rad * 180.0 / M_PI << "°] v=["
              << v_min_rad * 180.0 / M_PI << "°, " << v_max_rad * 180.0 / M_PI
              << "°]" << std::endl;
  }

  size_t hit_count = 0;

#pragma omp parallel for reduction(+:hit_count) schedule(dynamic)
  for (int v = 0; v < img.height; ++v) {
    // Elevation: top row (v=0) = v_max, bottom row = v_min
    double phi = (v_max_rad + v_min_rad > 0)
        ? img.v_max_rad - (v + 0.5) * res
        : v_max_rad - (v + 0.5) * res;
    // More robust: phi = v_max - (v + 0.5) * res
    phi = img.v_max_rad - (v + 0.5) * res;

    double cos_phi = std::cos(phi);
    double sin_phi = std::sin(phi);

    for (int u = 0; u < img.width; ++u) {
      double theta = img.h_min_rad + (u + 0.5) * res;
      double cos_theta = std::cos(theta);
      double sin_theta = std::sin(theta);

      // Direction in ERP frame
      Eigen::Vector3d dir_erp(cos_phi * cos_theta,
                               cos_phi * sin_theta,
                               sin_phi);
      // Transform to map frame
      Eigen::Vector3d dir_map = R_erp_to_map * dir_erp;

      octomap::point3d direction(
          static_cast<float>(dir_map.x()),
          static_cast<float>(dir_map.y()),
          static_cast<float>(dir_map.z()));

      // Nudge origin outside the voxel it's inside of, so castRay sees
      // geometry beyond, not the voxel containing the LiDAR itself.
      float nudge = tree->getResolution() * 0.75f;
      octomap::point3d nudged_origin(
          origin.x() + direction.x() * nudge,
          origin.y() + direction.y() * nudge,
          origin.z() + direction.z() * nudge);

      octomap::point3d end;
      bool hit = tree->castRay(nudged_origin, direction, end, true, range_max);

      size_t idx = static_cast<size_t>(v) * img.width + u;

      if (hit) {
        octomap::OcTreeKey key;
        if (tree->coordToKeyChecked(end, key)) {
          Eigen::Vector3f normal;
          int count;
          if (builder.getNormalAt(key, normal, count)) {
            img.occupied[idx] = 1;
            hit_count++;

            double dx = static_cast<double>(end.x() - origin.x());
            double dy = static_cast<double>(end.y() - origin.y());
            double dz = static_cast<double>(end.z() - origin.z());
            bool too_close = (dx*dx + dy*dy + dz*dz) < self_range * self_range;

            if (too_close) {
              img.normals[idx] = Eigen::Vector3f::Zero(); // counted but contributes 0 to S
            } else {
              img.normals[idx] = normal;
            }
          }
        }
      }
      // else: stays (0,0,0), occupied=0
    }
  }

  if (g_verbose) {
    std::cout << "[ERP] Done: " << hit_count << " hits, "
              << (npix - hit_count) << " misses ("
              << (npix > 0 ? 100.0 * hit_count / npix : 0) << "% hit rate)"
              << std::endl;
  }

  return img;
}

}  // namespace voxel_motion_strategy

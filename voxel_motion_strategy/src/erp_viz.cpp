#include <iostream>
#include <fstream>
#include <cmath>

#include "voxel_motion_strategy/octomap_builder.h"
#include "voxel_motion_strategy/erp_projector.h"

using namespace voxel_motion_strategy;

int main(int argc, char** argv) {
  if (argc < 5) {
    std::cerr << "Usage: erp_viz <octomap_prefix> <x> <y> <z> [output.ppm]" << std::endl;
    std::cerr << "  Saves a PPM image of the ERP depth map at the given pose." << std::endl;
    std::cerr << "  Color = normal direction (RGB = XYZ mapped to [0,255]), black = no hit." << std::endl;
    return 1;
  }

  std::string prefix = argv[1];
  double px = std::stod(argv[2]);
  double py = std::stod(argv[3]);
  double pz = std::stod(argv[4]);
  std::string outfile = (argc > 5) ? argv[5] : "/tmp/erp_viz.ppm";

  // Load octomap
  OctomapBuilder builder;
  std::cout << "Loading octomap..." << std::endl;
  if (!builder.load(prefix)) {
    std::cerr << "Failed to load." << std::endl;
    return 1;
  }

  // Project
  Eigen::Vector3d origin(px, py, pz);
  Eigen::Matrix3d R_erp_to_map = Eigen::Matrix3d::Identity();

  ERPParams params;
  params.resolution_deg = 1.0;
  params.range_max_m = 150.0;

  double h_min = -180.0 * M_PI / 180.0;
  double h_max =  180.0 * M_PI / 180.0;
  double v_min = -90.0 * M_PI / 180.0;
  double v_max =  60.0 * M_PI / 180.0;

  std::cout << "Projecting at (" << px << ", " << py << ", " << pz << ")..." << std::endl;
  ERPImage img = projectERP(builder, origin, R_erp_to_map,
                             h_min, h_max, v_min, v_max, params);

  if (img.empty()) {
    std::cerr << "Empty image." << std::endl;
    return 1;
  }

  // Write PPM (ASCII P3 format)
  std::ofstream ppm(outfile);
  ppm << "P3\n" << img.width << " " << img.height << "\n255\n";

  for (int v = 0; v < img.height; ++v) {
    for (int u = 0; u < img.width; ++u) {
      const auto& n = img.normalAt(u, v);
      if (img.isOccupied(u, v)) {
        // Force normal upward for visualization (flip if pointing down)
        Eigen::Vector3f n_viz = n;
        if (n_viz.z() < 0.0f) n_viz = -n_viz;
        int r = static_cast<int>((n_viz.x() + 1.0f) * 0.5f * 255.0f);
        int g = static_cast<int>((n_viz.y() + 1.0f) * 0.5f * 255.0f);
        int b = static_cast<int>((n_viz.z() + 1.0f) * 0.5f * 255.0f);
        ppm << r << " " << g << " " << b << " ";
      } else {
        ppm << "0 0 0 ";  // black = sky / no hit
      }
    }
    ppm << "\n";
  }
  ppm.close();

  std::cout << "Saved: " << outfile << " (" << img.width << "x" << img.height << ")" << std::endl;
  std::cout << "Color legend: R=normal.x  G=normal.y  B=normal.z  (black=no hit)" << std::endl;
  return 0;
}

// Offline tool: filter global point cloud to keep only planar points.
// Uses PCL NormalEstimationOMP curvature as a planarity indicator.
//
// Usage: ./filter_plane_points input.ply output.ply [voxel_m] [radius_m] [curv_thresh]

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>

#include <cstdio>
#include <cstdlib>
#include <string>

int main(int argc, char** argv) {
  if (argc < 3) {
    fprintf(stderr, "Usage: %s input.ply output.ply [voxel_m=0.5] [radius_m=2.0] [curv_thresh=0.03]\n", argv[0]);
    return 1;
  }

  std::string input_path  = argv[1];
  std::string output_path = argv[2];
  double voxel_m          = (argc >= 4) ? std::atof(argv[3]) : 0.5;
  double radius_m         = (argc >= 5) ? std::atof(argv[4]) : 2.0;
  double curv_thresh      = (argc >= 6) ? std::atof(argv[5]) : 0.03;

  printf("=== Plane Point Filter ===\n");
  printf("Input:       %s\n", input_path.c_str());
  printf("Output:      %s\n", output_path.c_str());
  printf("Voxel leaf:  %.2f m\n", voxel_m);
  printf("Radius:      %.2f m\n", radius_m);
  printf("Curv thresh: %.4f\n", curv_thresh);

  // 1. Load
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPLYFile(input_path, *raw) < 0) {
    fprintf(stderr, "ERROR: failed to load %s\n", input_path.c_str());
    return 1;
  }
  printf("Loaded %zu points.\n", raw->size());

  // 2. Voxel downsample
  pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>);
  {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(raw);
    vg.setLeafSize(static_cast<float>(voxel_m),
                   static_cast<float>(voxel_m),
                   static_cast<float>(voxel_m));
    vg.filter(*ds);
  }
  raw.reset();
  printf("Downsampled: %zu points.\n", ds->size());

  // 3. Normal estimation with curvature
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(ds);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(ds);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(static_cast<float>(radius_m));
    ne.setNumberOfThreads(0);   // use all cores
    ne.compute(*normals);
  }
  printf("Normals computed: %zu.\n", normals->size());

  // 4. Filter by curvature
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
  plane->reserve(ds->size());
  size_t kept = 0;
  for (size_t i = 0; i < ds->size(); ++i) {
    if (normals->points[i].curvature < static_cast<float>(curv_thresh)) {
      plane->points.push_back(ds->points[i]);
      ++kept;
    }
  }
  printf("Plane points: %zu / %zu (%.1f%%).\n", kept, ds->size(),
         100.0 * kept / ds->size());

  // 5. Save
  plane->width  = plane->points.size();
  plane->height = 1;
  if (pcl::io::savePLYFileBinary(output_path, *plane) < 0) {
    fprintf(stderr, "ERROR: failed to save %s\n", output_path.c_str());
    return 1;
  }
  printf("Saved to %s\n", output_path.c_str());
  return 0;
}

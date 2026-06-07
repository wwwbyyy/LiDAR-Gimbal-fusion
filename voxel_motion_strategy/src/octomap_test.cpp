#include <iostream>
#include <fstream>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <numeric>

#include <Eigen/Eigenvalues>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>

// ---- voxel key helpers ----

inline uint64_t voxelKey(int x, int y, int z) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 42) |
         (static_cast<uint64_t>(static_cast<uint32_t>(y)) << 21) |
         static_cast<uint64_t>(static_cast<uint32_t>(z) & 0x1FFFFF);
}

inline Eigen::Vector3d voxelCenter(uint64_t key, double voxel_size) {
  int x = static_cast<int>(key >> 42);
  int y = static_cast<int>((key >> 21) & 0x1FFFFF);
  int z = static_cast<int>(key & 0x1FFFFF);
  if (x & (1 << 20)) x |= ~((1 << 21) - 1);
  if (y & (1 << 20)) y |= ~((1 << 21) - 1);
  if (z & (1 << 20)) z |= ~((1 << 21) - 1);
  return Eigen::Vector3d((x + 0.5) * voxel_size,
                         (y + 0.5) * voxel_size,
                         (z + 0.5) * voxel_size);
}

// ---- normal hemisphere helpers ----

inline Eigen::Vector3d flipToHemisphere(const Eigen::Vector3d& n,
                                         const Eigen::Vector3d& ref) {
  if (n.dot(ref) < 0.0) return -n;
  return n;
}

inline double computeR(const Eigen::Vector3d& sum_flipped, int count) {
  if (count <= 1) return 1.0;
  double len = sum_flipped.norm();
  return std::clamp(len / count, 0.0, 1.0);
}

// ---- per-voxel accumulation (online, O(1) memory per voxel) ----

struct VoxelStats {
  // --- normal consistency ---
  Eigen::Vector3d sum_normal = Eigen::Vector3d::Zero();
  Eigen::Vector3d ref_normal = Eigen::Vector3d::Zero();
  // --- point spatial covariance (online) ---
  double sum_x = 0, sum_y = 0, sum_z = 0;
  double sum_xx = 0, sum_yy = 0, sum_zz = 0;
  double sum_xy = 0, sum_xz = 0, sum_yz = 0;
  int count = 0;
};

// ---- planarity from point covariance ----

inline double planarity(const VoxelStats& vs) {
  if (vs.count < 3) return 1.0; // trivially planar
  double invN = 1.0 / vs.count;
  double mx = vs.sum_x * invN, my = vs.sum_y * invN, mz = vs.sum_z * invN;
  // cov = (sum_xx - sum_x*mx, sum_xy - sum_x*my, ...) / N
  Eigen::Matrix3d cov;
  cov(0,0) = invN * vs.sum_xx - mx * mx;
  cov(1,1) = invN * vs.sum_yy - my * my;
  cov(2,2) = invN * vs.sum_zz - mz * mz;
  cov(0,1) = cov(1,0) = invN * vs.sum_xy - mx * my;
  cov(0,2) = cov(2,0) = invN * vs.sum_xz - mx * mz;
  cov(1,2) = cov(2,1) = invN * vs.sum_yz - my * mz;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
  Eigen::Vector3d ev = eig.eigenvalues(); // sorted ascending: λ₂ ≤ λ₁ ≤ λ₀
  double sum_ev = ev(0) + ev(1) + ev(2);
  if (sum_ev < 1e-12) return 1.0;
  // planarity: 1 = perfectly flat (λ₂ ≈ 0), 0 = volumetric (all λ similar)
  return 1.0 - 3.0 * ev(0) / sum_ev;
}

// ---- main ----

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: octomap_test <point_cloud.ply> [normal_cloud.ply] [options]" << std::endl;
    std::cerr << "  --voxel-size N     voxel size in meters (default 1.0)" << std::endl;
    std::cerr << "  --range-max M       max distance from origin, -1 = unlimited (default 150)" << std::endl;
    std::cerr << "  --output-ply FILE   export voxel centers with R + planarity scalars" << std::endl;
    return 1;
  }

  std::string points_file = argv[1];
  std::string normals_file, output_ply;
  double voxel_size = 1.0, range_max = 150.0;

  for (int i = 2; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--voxel-size" && i + 1 < argc)
      voxel_size = std::stod(argv[++i]);
    else if (arg == "--range-max" && i + 1 < argc)
      range_max = std::stod(argv[++i]);
    else if (arg == "--output-ply" && i + 1 < argc)
      output_ply = argv[++i];
    else if (arg[0] != '-')
      normals_file = arg;
  }

  std::cout << "=== Voxel Analysis: R (normal consistency) + Planarity (point flatness) ===" << std::endl;
  std::cout << "Points: " << points_file << std::endl;
  std::cout << "Normals: " << (normals_file.empty() ? "(on-the-fly)" : normals_file) << std::endl;
  std::cout << "Voxel size: " << voxel_size << " m  Range: "
            << (range_max < 0 ? "unlimited" : std::to_string(range_max) + " m") << std::endl;
  std::cout << std::endl;

  // -- load points --
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "Loading point cloud..." << std::endl;
  if (pcl::io::loadPLYFile(points_file, *cloud) < 0) {
    std::cerr << "Failed to load: " << points_file << std::endl;
    return 1;
  }
  cloud->erase(std::remove_if(cloud->begin(), cloud->end(), [](const pcl::PointXYZ& p) {
    return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
  }), cloud->end());
  std::cout << "  " << cloud->size() << " valid points." << std::endl;

  // -- load / compute normals --
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  if (!normals_file.empty()) {
    std::cout << "Loading normals..." << std::endl;
    if (pcl::io::loadPLYFile(normals_file, *normals) < 0) {
      std::cerr << "  failed, will compute on-the-fly." << std::endl;
      normals_file.clear();
    } else {
      std::cout << "  " << normals->size() << " normals." << std::endl;
    }
  }
  if (normals_file.empty() || normals->size() != cloud->size()) {
    std::cout << "Computing normals (r=0.5, k=20)..." << std::endl;
    normals->clear();
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(0.5);
    ne.setNumberOfThreads(8);
    ne.compute(*normals);
  }

  // -- voxelize --
  std::cout << "Voxelizing (normal + point covariance)..." << std::endl;
  std::unordered_map<uint64_t, VoxelStats> voxels;
  size_t sk_range = 0, sk_nan = 0;
  double inv_vs = 1.0 / voxel_size;

  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto& pt = cloud->points[i];
    auto& n = normals->points[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) { sk_nan++; continue; }
    if (range_max > 0) {
      if (pt.x*pt.x + pt.y*pt.y + pt.z*pt.z > range_max*range_max) { sk_range++; continue; }
    }
    Eigen::Vector3f nf = n.getNormalVector3fMap();
    float nf_norm = nf.norm();
    if (!std::isfinite(nf_norm) || nf_norm < 1e-6) { sk_nan++; continue; }
    nf /= nf_norm;

    int ix = static_cast<int>(std::floor(pt.x * inv_vs));
    int iy = static_cast<int>(std::floor(pt.y * inv_vs));
    int iz = static_cast<int>(std::floor(pt.z * inv_vs));
    uint64_t key = voxelKey(ix, iy, iz);
    auto& vs = voxels[key];

    // normals (hemisphere-flipped)
    Eigen::Vector3d nd = nf.cast<double>();
    if (vs.count == 0) {
      vs.ref_normal = nd;
      vs.sum_normal = nd;
    } else {
      vs.sum_normal += flipToHemisphere(nd, vs.ref_normal);
    }
    // point covariance (online)
    double px = pt.x, py = pt.y, pz = pt.z;
    vs.sum_x += px; vs.sum_y += py; vs.sum_z += pz;
    vs.sum_xx += px*px; vs.sum_yy += py*py; vs.sum_zz += pz*pz;
    vs.sum_xy += px*py; vs.sum_xz += px*pz; vs.sum_yz += py*pz;
    vs.count++;
  }
  std::cout << "  Voxels: " << voxels.size() << "  skipped: range=" << sk_range << " nan=" << sk_nan << std::endl;

  // -- compute R and planarity per voxel --
  struct VoxelResult {
    Eigen::Vector3d center;
    int count;
    double R, planarity;
  };
  std::vector<VoxelResult> results;
  results.reserve(voxels.size());

  for (const auto& kv : voxels) {
    const auto& vs = kv.second;
    double R = computeR(vs.sum_normal, vs.count);
    double pl = planarity(vs);
    results.push_back({voxelCenter(kv.first, voxel_size), vs.count, R, pl});
  }

  size_t N = results.size();

  // -- statistics --
  {
    std::vector<double> all_R, all_pl;
    all_R.reserve(N); all_pl.reserve(N);
    for (const auto& r : results) { all_R.push_back(r.R); all_pl.push_back(r.planarity); }
    std::sort(all_R.begin(), all_R.end());
    std::sort(all_pl.begin(), all_pl.end());

    std::cout << std::endl;
    std::cout << "=== R Statistics ===" << std::endl;
    std::cout << "  Mean: " << std::setprecision(4) << std::accumulate(all_R.begin(), all_R.end(), 0.0)/N << std::endl;
    for (double p : {10.,25.,50.,75.,90.,95.,99.})
      std::cout << "  P" << int(p) << ": " << all_R[std::min(size_t(p/100.*N), N-1)] << std::endl;

    std::cout << std::endl;
    std::cout << "=== Planarity Statistics (1.0 = flat, 0.0 = volumetric) ===" << std::endl;
    double mean_pl = std::accumulate(all_pl.begin(), all_pl.end(), 0.0)/N;
    std::cout << "  Mean: " << std::setprecision(4) << mean_pl << std::endl;
    for (double p : {10.,25.,50.,75.,90.,95.,99.})
      std::cout << "  P" << int(p) << ": " << all_pl[std::min(size_t(p/100.*N), N-1)] << std::endl;
  }

  // -- R vs planarity joint distribution --
  {
    const int B = 10;
    int joint[B][B] = {};
    double r_min = 0, r_max = 1, pl_min = 0, pl_max = 1;
    for (const auto& r : results) {
      int ri = std::min(int((r.R - r_min) / (r_max - r_min) * B), B-1);
      int pi = std::min(int((r.planarity - pl_min) / (pl_max - pl_min) * B), B-1);
      joint[ri][pi]++;
    }
    std::cout << std::endl;
    std::cout << "=== R vs Planarity Joint Distribution ===" << std::endl;
    std::cout << "  planarity →     ";
    for (int p = 0; p < B; ++p) std::cout << std::setw(7) << std::fixed << std::setprecision(1) << pl_min + (p+0.5)*(pl_max-pl_min)/B;
    std::cout << std::endl;
    std::cout << "  R ↓" << std::endl;
    for (int ri = B-1; ri >= 0; --ri) {
      std::cout << "  [" << std::setprecision(1) << r_min + ri*(r_max-r_min)/B
                << "," << r_min + (ri+1)*(r_max-r_min)/B << ") ";
      for (int pi = 0; pi < B; ++pi) {
        double pct = 100.0 * joint[ri][pi] / N;
        std::cout << std::setw(6) << std::fixed << std::setprecision(1) << pct << "%";
      }
      std::cout << std::endl;
    }
  }

  // -- key question: can planarity separate walls from foliage? --
  {
    // High R voxels: split by planarity
    std::cout << std::endl;
    std::cout << "=== High-R voxels split by planarity ===" << std::endl;
    std::cout << "  (R >= 0.95 → 'normal-consistent'. Do they separate by planarity?)" << std::endl;
    std::cout << "  Planarity range | Voxels  | % of high-R | Interpretation" << std::endl;
    std::cout << "  ----------------+---------+-------------+---------------" << std::endl;
    for (double pl_lo : {0.0, 0.3, 0.5, 0.7, 0.8, 0.9, 0.95}) {
      double pl_hi = std::min(pl_lo + 0.1, 1.01);
      int cnt = 0;
      for (const auto& r : results)
        if (r.R >= 0.95 && r.planarity >= pl_lo && r.planarity < pl_hi) cnt++;
      int total_highR = 0;
      for (const auto& r : results) if (r.R >= 0.95) total_highR++;
      double pct = total_highR > 0 ? 100.0 * cnt / total_highR : 0;
      std::string interp;
      if (pl_lo < 0.6) interp = "volumetric (foliage-like)";
      else if (pl_lo < 0.85) interp = "mixed / rough surface";
      else interp = "planar (wall/ground-like)";
      std::cout << "  [" << std::setprecision(1) << pl_lo << ", " << pl_hi << ")  "
                << std::setw(8) << cnt << "  " << std::setw(8) << pct << "%  " << interp << std::endl;
    }
  }

  // -- threshold suggestions --
  {
    std::cout << std::endl;
    std::cout << "=== Combined Threshold Suggestions ===" << std::endl;
    std::cout << "  (R >= R_thresh) AND (planarity >= pl_thresh) → 'good' voxel" << std::endl;
    std::cout << "  R_thresh\\pl_thresh | 0.0    | 0.5    | 0.7    | 0.8    | 0.9    | 0.95" << std::endl;
    std::cout << "  ------------------+--------+--------+--------+--------+--------+-------" << std::endl;
    for (double r_th : {0.99, 0.98, 0.95, 0.90, 0.85}) {
      std::cout << "  " << std::fixed << std::setprecision(2) << r_th << "               ";
      for (double pl_th : {0.0, 0.5, 0.7, 0.8, 0.9, 0.95}) {
        int cnt = 0;
        for (const auto& r : results)
          if (r.R >= r_th && r.planarity >= pl_th) cnt++;
        std::cout << std::setw(6) << std::fixed << std::setprecision(1) << 100.0*cnt/N << "% ";
      }
      std::cout << std::endl;
    }
  }

  // -- PLY export --
  if (!output_ply.empty()) {
    std::cout << std::endl;
    std::cout << "=== Exporting PLY: " << output_ply << " ===" << std::endl;
    std::ofstream ply(output_ply);
    ply << "ply\nformat ascii 1.0\n";
    ply << "comment R = normal consistency (1=aligned), planarity = point flatness (1=flat, 0=volumetric)\n";
    ply << "element vertex " << results.size() << "\n";
    ply << "property float x\nproperty float y\nproperty float z\n";
    ply << "property float R\nproperty float planarity\nproperty int count\n";
    ply << "end_header\n";
    ply << std::fixed << std::setprecision(6);
    for (const auto& r : results)
      ply << r.center.x() << " " << r.center.y() << " " << r.center.z() << " "
          << r.R << " " << r.planarity << " " << r.count << "\n";
    ply.close();
    std::cout << "  Exported " << results.size() << " voxels." << std::endl;
    std::cout << "  CloudCompare: Color Scale → 'planarity' to see flat vs volumetric." << std::endl;
  }

  return 0;
}

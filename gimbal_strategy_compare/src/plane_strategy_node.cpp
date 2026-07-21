// Plane Point Counting Strategy — simpler gimbal strategy.
//
// Core idea:
//   1. Pre-filter global map for planar points (offline, filter_plane_points).
//   2. At each pose, extract local plane points in LiDAR frame.
//   3. Bin into a spherical count grid, build integral image, search for
//      the FoV rectangle with maximum plane point count.
//   4. Step toward target with linear motion constraints.
//
// Dual-loop: strategy_period (re-eval) + rotation_period (stepping).

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <cyber_msgs/GimbalCommand.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>

#include <cmath>
#include <string>
#include <vector>
#include <mutex>
#include <algorithm>
#include <cstdint>

// ----------------------------------------------------------------------
// Data structures
// ----------------------------------------------------------------------

struct CountGrid {
  int width = 0, height = 0;
  std::vector<int> counts;
  double h_min_rad = 0.0, h_max_rad = 0.0;
  double v_min_rad = 0.0, v_max_rad = 0.0;
  double resolution_rad = 0.0;

  bool empty() const { return width == 0 || height == 0; }
  int index(int u, int v) const { return v * width + u; }
};

struct CountSAT {
  int w_ = 0, h_ = 0;               // SAT dims = (grid_w + 1 + h_extend, grid_h + 1)
  int grid_w_ = 0;
  std::vector<int64_t> sat_;

  static CountSAT build(const CountGrid& grid, int h_extend = 0) {
    CountSAT sat;
    sat.grid_w_ = grid.width;
    sat.w_ = grid.width + h_extend + 1;
    sat.h_ = grid.height + 1;
    sat.sat_.assign(sat.w_ * sat.h_, 0);

    for (int v = 0; v < grid.height; ++v) {
      for (int u = 0; u < sat.w_ - 1; ++u) {
        int src_u = u;
        if (src_u >= grid.width) src_u -= grid.width;
        int64_t val = grid.counts[grid.index(src_u, v)];

        int64_t left  = (u > 0)           ? sat.sat_[v * sat.w_ + (u - 1) + sat.w_] : 0;
        int64_t up    = (v > 0)           ? sat.sat_[(v - 1) * sat.w_ + u + sat.w_] : 0;
        int64_t diag  = (u > 0 && v > 0)  ? sat.sat_[(v - 1) * sat.w_ + (u - 1) + sat.w_] : 0;

        sat.sat_[v * sat.w_ + u + sat.w_] = val + left + up - diag;
      }
    }
    return sat;
  }

  int64_t query(int u1, int u2, int v1, int v2) const {
    auto at = [this](int u, int v) -> int64_t {
      if (u < 0 || u >= w_ || v < 0 || v >= h_) return 0;
      return sat_[v * w_ + u];
    };
    return at(u2, v2) - at(u1, v2) - at(u2, v1) + at(u1, v1);
  }
};

struct PlaneRectResult {
  double best_yaw_rad   = 0.0;
  double best_pitch_rad = 0.0;
  int64_t best_count    = 0;
  bool valid            = false;
};

struct MotionConstraintParams {
  double pan_vel_max  = 0.35;   // rad/s
  double tilt_vel_max = 0.14;   // rad/s
};

// ----------------------------------------------------------------------
// Global state
// ----------------------------------------------------------------------

static pcl::PointCloud<pcl::PointXYZ>::Ptr g_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
static pcl::KdTreeFLANN<pcl::PointXYZ> g_plane_kdtree;
static bool g_plane_ready = false;

static nav_msgs::Odometry g_latest_odom;
static bool g_odom_received = false;
static std::mutex g_odom_mutex;

static double g_pan_deg = 0.0;
static bool g_pan_received = false;
static std::mutex g_pan_mutex;

static double g_tilt_deg = 0.0;
static bool g_tilt_received = false;
static std::mutex g_tilt_mutex;

// Target tracking
static double g_target_pan_deg   = 0.0;
static double g_target_pitch_deg = 0.0;   // ERP space
static bool g_has_target         = false;
static ros::Time g_last_eval_time(0.0);

// Parameters
static double g_strategy_period = 2.0;
static double g_rotation_period = 0.5;
static double g_erp_resolution_deg = 1.0;
static double g_range_max_m = 150.0;
static double g_fov_horizontal_deg = 60.0;
static double g_fov_vertical_deg   = 68.0;
static double g_pitch_min_deg = -15.0;
static double g_pitch_max_deg = 15.0;
static MotionConstraintParams g_motion_params;
static Eigen::Vector3d g_lidar_T(1.08, 0.0, 1.643);
static Eigen::Matrix3d g_R_lidar_to_vehicle = Eigen::Matrix3d::Identity();

// ----------------------------------------------------------------------
// Coordinate helpers
// ----------------------------------------------------------------------

static double shortestAngleDiff(double a_deg, double b_deg) {
  return std::fmod(a_deg - b_deg + 540.0, 360.0) - 180.0;
}

static double panToVehicleYaw(double pan_deg) {
  double yaw = std::fmod(pan_deg, 360.0);
  if (yaw > 180.0) yaw -= 360.0;
  return yaw;
}

static double vehicleYawToPan(double yaw_deg) {
  double pan = std::fmod(yaw_deg, 360.0);
  if (pan < 0.0) pan += 360.0;
  return pan;
}

// pelco tilt [0,360) → ERP elevation (0=horiz, +=down)
static double pelcoTiltToErpPitch(double tilt_deg) {
  double pitch = tilt_deg;
  if (pitch > 180.0) pitch -= 360.0;
  return pitch;
}

// ----------------------------------------------------------------------
// Callbacks
// ----------------------------------------------------------------------

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(g_odom_mutex);
  g_latest_odom = *msg;
  g_odom_received = true;
}

void panCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() < 2) return;
  std::lock_guard<std::mutex> lock(g_pan_mutex);
  g_pan_deg = msg->data[1];
  g_pan_received = true;
}

void tiltCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() < 2) return;
  std::lock_guard<std::mutex> lock(g_tilt_mutex);
  g_tilt_deg = msg->data[1];
  g_tilt_received = true;
}

// ----------------------------------------------------------------------
// Build count grid from local plane points
// ----------------------------------------------------------------------

static CountGrid buildCountGrid(const Eigen::Vector3d& origin_map,
                                 const Eigen::Matrix3d& R_map_to_lidar,
                                 double h_min, double h_max,
                                 double v_min, double v_max,
                                 double res_rad) {
  CountGrid grid;
  grid.h_min_rad = h_min;
  grid.h_max_rad = h_max;
  grid.v_min_rad = v_min;
  grid.v_max_rad = v_max;
  grid.resolution_rad = res_rad;

  grid.width  = static_cast<int>(std::ceil((h_max - h_min) / res_rad));
  grid.height = static_cast<int>(std::ceil((v_max - v_min) / res_rad));
  if (grid.width <= 0 || grid.height <= 0) return grid;

  grid.counts.assign(grid.width * grid.height, 0);

  // kd-tree radius search
  pcl::PointXYZ search_pt;
  search_pt.x = static_cast<float>(origin_map.x());
  search_pt.y = static_cast<float>(origin_map.y());
  search_pt.z = static_cast<float>(origin_map.z());

  std::vector<int> indices;
  std::vector<float> dists;
  g_plane_kdtree.radiusSearch(search_pt, static_cast<float>(g_range_max_m),
                               indices, dists);

  double inv_res = 1.0 / res_rad;
  for (int idx : indices) {
    const auto& pt = g_plane_cloud->points[idx];
    Eigen::Vector3d vm(pt.x, pt.y, pt.z);
    Eigen::Vector3d vl = R_map_to_lidar * (vm - origin_map);

    double r = vl.norm();
    if (r < 1e-6) continue;

    double azimuth  = std::atan2(vl.y(), vl.x());
    double elevation = std::asin(vl.z() / r);

    if (azimuth < h_min || azimuth > h_max) continue;
    if (elevation < v_min || elevation > v_max) continue;

    int u = static_cast<int>((azimuth - h_min) * inv_res);
    int v = static_cast<int>((v_max - elevation) * inv_res);  // top row = max elevation

    if (u < 0) u = 0;
    if (u >= grid.width)  u = grid.width - 1;
    if (v < 0) v = 0;
    if (v >= grid.height) v = grid.height - 1;

    ++grid.counts[grid.index(u, v)];
  }
  return grid;
}

// ----------------------------------------------------------------------
// Rectangle search — max plane-point count
// ----------------------------------------------------------------------

static PlaneRectResult searchBestCount(const CountSAT& sat,
                                        const CountGrid& grid,
                                        double yaw_min_rad, double yaw_max_rad,
                                        double pitch_min_rad, double pitch_max_rad,
                                        double fov_h_deg, double fov_v_deg) {
  PlaneRectResult result;

  double res_rad = grid.resolution_rad;
  int fov_w_px = static_cast<int>(std::round(fov_h_deg * M_PI / 180.0 / res_rad));
  int fov_h_px = static_cast<int>(std::round(fov_v_deg * M_PI / 180.0 / res_rad));
  int hw = fov_w_px / 2;
  int hh = fov_h_px / 2;

  int img_w = grid.width;
  double inv_res = 1.0 / res_rad;

  for (double yaw_c = yaw_min_rad; yaw_c <= yaw_max_rad + 1e-9; yaw_c += res_rad) {
    // yaw → grid column: u = (yaw - h_min) / res
    int uc = static_cast<int>((yaw_c - grid.h_min_rad) * inv_res + 0.5);

    for (double pitch_c = pitch_min_rad; pitch_c <= pitch_max_rad + 1e-9; pitch_c += res_rad) {
      // pitch → grid row: v=0 at top (v_max), so v = (v_max - pitch) / res
      int vc = static_cast<int>((grid.v_max_rad - pitch_c) * inv_res + 0.5);

      int u1 = uc - hw;
      int u2 = uc + hw;
      int v1 = vc - hh;
      int v2 = vc + hh;

      // Clamp v to grid
      if (v2 <= 0 || v1 >= grid.height) continue;
      if (v1 < 0) v1 = 0;
      if (v2 > grid.height) v2 = grid.height;
      if (v1 >= v2) continue;

      // Handle horizontal wraparound by shifting into SAT extended region
      if (u1 < 0) { u1 += img_w; u2 += img_w; }

      // Clamp u to SAT dimensions
      if (u1 < 0) u1 = 0;
      if (u2 > sat.w_ - 1) u2 = sat.w_ - 1;
      if (u1 >= u2) continue;

      int64_t cnt = sat.query(u1, u2, v1, v2);
      if (cnt > result.best_count) {
        result.best_count = cnt;
        result.best_yaw_rad = yaw_c;
        result.best_pitch_rad = pitch_c;
        result.valid = true;
      }
    }
  }
  return result;
}

// ----------------------------------------------------------------------
// Timer callback
// ----------------------------------------------------------------------

void strategyUpdate(const ros::TimerEvent&) {
  if (!g_odom_received || !g_pan_received || !g_tilt_received || !g_plane_ready) {
    ROS_WARN_THROTTLE(5.0, "[PlaneStrategy] Waiting for odometry, pan, tilt, and plane map...");
    return;
  }

  ros::Time now = ros::Time::now();

  // 1. Get current state
  nav_msgs::Odometry odom;
  double pan_deg, tilt_deg;
  {
    std::lock_guard<std::mutex> l1(g_odom_mutex);
    odom = g_latest_odom;
  }
  {
    std::lock_guard<std::mutex> l2(g_pan_mutex);
    pan_deg = g_pan_deg;
  }
  {
    std::lock_guard<std::mutex> l3(g_tilt_mutex);
    tilt_deg = g_tilt_deg;
  }

  // 2. Strategy re-evaluation (outer loop)
  bool re_eval = !g_has_target ||
                 (now - g_last_eval_time).toSec() >= g_strategy_period - 1e-6;

  if (re_eval) {
    ros::WallTime t0 = ros::WallTime::now();

    double erp_pitch_deg = pelcoTiltToErpPitch(tilt_deg);
    double vehicle_yaw_rad = panToVehicleYaw(pan_deg) * M_PI / 180.0;
    double erp_pitch_rad = erp_pitch_deg * M_PI / 180.0;

    // Feasible yaw / pitch ranges (clamp yaw to [-pi, pi] for atan2-compatible azimuth)
    double yaw_min = vehicle_yaw_rad - g_motion_params.pan_vel_max * g_strategy_period;
    double yaw_max = vehicle_yaw_rad + g_motion_params.pan_vel_max * g_strategy_period;
    if (yaw_min < -M_PI) yaw_min = -M_PI;
    if (yaw_max >  M_PI) yaw_max =  M_PI;
    if (yaw_min > yaw_max) yaw_min = yaw_max;

    double pitch_hard_min = g_pitch_min_deg * M_PI / 180.0;
    double pitch_hard_max = g_pitch_max_deg * M_PI / 180.0;
    double pitch_min = std::max(erp_pitch_rad - g_motion_params.tilt_vel_max * g_strategy_period,
                                pitch_hard_min);
    double pitch_max = std::min(erp_pitch_rad + g_motion_params.tilt_vel_max * g_strategy_period,
                                pitch_hard_max);
    if (pitch_min > pitch_max) pitch_min = pitch_max;

    // Vehicle pose + LiDAR extrinsic
    const auto& pos  = odom.pose.pose.position;
    const auto& quat = odom.pose.pose.orientation;
    Eigen::Vector3d vehicle_pos_map(pos.x, pos.y, pos.z);
    Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
    Eigen::Matrix3d R_vehicle_to_map = q.toRotationMatrix();

    Eigen::Vector3d origin_map = vehicle_pos_map + R_vehicle_to_map * g_lidar_T;
    Eigen::Matrix3d R_lidar_to_map = R_vehicle_to_map * g_R_lidar_to_vehicle;
    Eigen::Matrix3d R_map_to_lidar = R_lidar_to_map.transpose();

    // Build count grid: horizontal range = feasible yaw + FoV margin, clamped to [-pi, pi]
    double fov_hw_rad = g_fov_horizontal_deg * M_PI / 180.0 * 0.5;
    double h_min = yaw_min - fov_hw_rad;
    double h_max = yaw_max + fov_hw_rad;
    if (h_min < -M_PI) h_min = -M_PI;
    if (h_max >  M_PI) h_max =  M_PI;
    if (h_max - h_min < 1.0 * M_PI / 180.0) h_max = h_min + 1.0 * M_PI / 180.0;

    double v_min = g_pitch_min_deg * M_PI / 180.0;
    double v_max = g_pitch_max_deg * M_PI / 180.0;

    double res_rad = g_erp_resolution_deg * M_PI / 180.0;

    CountGrid grid = buildCountGrid(origin_map, R_map_to_lidar,
                                     h_min, h_max, v_min, v_max, res_rad);

    if (!grid.empty()) {
      // Horizontal wraparound extension for rectangle search
      int h_extend = static_cast<int>(std::round(g_fov_horizontal_deg / g_erp_resolution_deg));
      CountSAT sat = CountSAT::build(grid, h_extend);

      auto result = searchBestCount(sat, grid, yaw_min, yaw_max,
                                     pitch_min, pitch_max,
                                     g_fov_horizontal_deg, g_fov_vertical_deg);

      if (result.valid) {
        double target_yaw_deg = result.best_yaw_rad * 180.0 / M_PI;
        g_target_pan_deg = vehicleYawToPan(target_yaw_deg);
        g_target_pitch_deg = result.best_pitch_rad * 180.0 / M_PI;  // ERP space
        g_has_target = true;
        g_last_eval_time = now;

        double elapsed_ms = (ros::WallTime::now() - t0).toSec() * 1000.0;
        ROS_INFO("[PlaneStrategy] re-eval: target pan=%.1f deg tilt=%.1f deg (erp) "
                 "count=%ld grid=%dx%d pts=%d yaw=[%.0f,%.0f] deg pitch=[%.0f,%.0f] deg "
                 "%.0fms",
                 g_target_pan_deg, g_target_pitch_deg,
                 result.best_count, grid.width, grid.height,
                 static_cast<int>(grid.counts.size()),
                 yaw_min * 180.0 / M_PI, yaw_max * 180.0 / M_PI,
                 pitch_min * 180.0 / M_PI, pitch_max * 180.0 / M_PI,
                 elapsed_ms);
      } else {
        ROS_WARN_THROTTLE(1.0, "[PlaneStrategy] Search returned no valid rectangle.");
        return;
      }
    } else {
      ROS_WARN_THROTTLE(5.0, "[PlaneStrategy] Count grid empty (no plane points in range?).");
      return;
    }
  }

  // 3. Step toward target (inner loop — every rotation_period)
  double pan_step_max  = g_motion_params.pan_vel_max  * g_rotation_period * 180.0 / M_PI;
  double tilt_step_max = g_motion_params.tilt_vel_max * g_rotation_period * 180.0 / M_PI;

  double dpan = shortestAngleDiff(g_target_pan_deg, pan_deg);
  double dpan_clamped = std::max(-pan_step_max, std::min(pan_step_max, dpan));
  double cmd_pan = std::fmod(pan_deg + dpan_clamped + 360.0, 360.0);

  // Tilt step in ERP space (linear, no wrap). pelco_control normalizes to pelco format.
  double erp_tilt = pelcoTiltToErpPitch(tilt_deg);
  double dtilt = g_target_pitch_deg - erp_tilt;
  double dtilt_clamped = std::max(-tilt_step_max, std::min(tilt_step_max, dtilt));
  double cmd_tilt = erp_tilt + dtilt_clamped;

  // 4. Publish gimbal commands
  static ros::NodeHandle nh;
  static ros::Publisher pub = nh.advertise<cyber_msgs::GimbalCommand>("/gimbal_cmd", 10);

  cyber_msgs::GimbalCommand pan_cmd;
  pan_cmd.header.stamp = now;
  pan_cmd.cmd = 0x4B;
  pan_cmd.data = cmd_pan;
  pub.publish(pan_cmd);

  cyber_msgs::GimbalCommand tilt_cmd;
  tilt_cmd.header.stamp = now;
  tilt_cmd.cmd = 0x4D;
  tilt_cmd.data = cmd_tilt;
  pub.publish(tilt_cmd);

  ROS_INFO("[PlaneStrategy] step: pan %.1f->%.1f (d=%.1f max=%.1f) target=%.1f | "
           "tilt %.1f->%.1f erp (d=%.1f max=%.1f) target=%.1f erp",
           pan_deg, cmd_pan, dpan_clamped, pan_step_max, g_target_pan_deg,
           erp_tilt, cmd_tilt, dtilt_clamped, tilt_step_max, g_target_pitch_deg);
}

// ----------------------------------------------------------------------
// main
// ----------------------------------------------------------------------

int main(int argc, char** argv) {
  ros::init(argc, argv, "plane_strategy_node");
  ros::NodeHandle nh("~");

  // Load params
  std::string plane_map_path;
  nh.param<std::string>("plane_map_path", plane_map_path, "");
  nh.param<double>("strategy_period", g_strategy_period, 2.0);
  nh.param<double>("rotation_period", g_rotation_period, 0.5);
  nh.param<double>("erp_resolution_deg", g_erp_resolution_deg, 1.0);
  nh.param<double>("range_max_m", g_range_max_m, 150.0);
  nh.param<double>("fov_horizontal_deg", g_fov_horizontal_deg, 60.0);
  nh.param<double>("fov_vertical_deg",   g_fov_vertical_deg,   68.0);
  nh.param<double>("pitch_min_deg",      g_pitch_min_deg,     -15.0);
  nh.param<double>("pitch_max_deg",      g_pitch_max_deg,      15.0);

  // LiDAR extrinsic
  {
    std::vector<double> ext;
    nh.param<std::vector<double>>("lidar_extinct", ext,
      {-0.2, 0.25, 1.43, 0, 0, 0});
    if (ext.size() >= 6) {
      g_lidar_T = Eigen::Vector3d(ext[0], ext[1], ext[2]);
      double roll = ext[3], pitch = ext[4], yaw = ext[5];
      g_R_lidar_to_vehicle =
          Eigen::AngleAxisd(yaw   * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll  * M_PI / 180.0, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
      ROS_INFO("LiDAR extrinsic: T=[%.2f,%.2f,%.2f] RPY=[%.1f,%.1f,%.1f] deg",
               ext[0], ext[1], ext[2], roll, pitch, yaw);
    }
  }

  {
    double pan_vel_dps, tilt_vel_dps;
    nh.param<double>("pan_angular_velocity_deg",  pan_vel_dps,  20.0);
    nh.param<double>("tilt_angular_velocity_deg", tilt_vel_dps, 8.0);
    g_motion_params.pan_vel_max  = pan_vel_dps  * M_PI / 180.0;
    g_motion_params.tilt_vel_max = tilt_vel_dps * M_PI / 180.0;
  }

  // Load plane map
  if (plane_map_path.empty()) {
    ROS_ERROR("plane_map_path is required!");
    return 1;
  }
  ROS_INFO("Loading plane map from %s ...", plane_map_path.c_str());
  if (pcl::io::loadPLYFile(plane_map_path, *g_plane_cloud) < 0) {
    ROS_ERROR("Failed to load %s", plane_map_path.c_str());
    return 1;
  }
  ROS_INFO("Loaded %zu plane points. Building kd-tree ...", g_plane_cloud->size());
  g_plane_kdtree.setInputCloud(g_plane_cloud);
  g_plane_ready = true;
  ROS_INFO("Ready.");

  // Subscribers
  ros::NodeHandle nh_global;
  ros::Subscriber odom_sub = nh_global.subscribe("/iekf3d/odometry", 10, odomCallback);
  ros::Subscriber pan_sub  = nh_global.subscribe("/pan", 10, panCallback);
  ros::Subscriber tilt_sub = nh_global.subscribe("/tilt", 10, tiltCallback);

  // Timer: rotation_period drives the inner loop, strategy re-eval embedded
  ros::Timer timer = nh.createTimer(ros::Duration(g_rotation_period), strategyUpdate);

  ROS_INFO("Plane Strategy node started: strategy_period=%.1fs rotation_period=%.2fs "
           "pan_vel=%.0f deg/s tilt_vel=%.0f deg/s",
           g_strategy_period, g_rotation_period,
           g_motion_params.pan_vel_max * 180.0 / M_PI,
           g_motion_params.tilt_vel_max * 180.0 / M_PI);
  ros::spin();
  return 0;
}

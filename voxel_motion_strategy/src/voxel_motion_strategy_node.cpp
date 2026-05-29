#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <cyber_msgs/GimbalCommand.h>

#include <cmath>
#include <string>
#include <mutex>

#include "voxel_motion_strategy/octomap_builder.h"
#include "voxel_motion_strategy/erp_projector.h"
#include "voxel_motion_strategy/integral_image.h"
#include "voxel_motion_strategy/rectangle_search.h"
#include "voxel_motion_strategy/yaw_constraint.h"

using namespace voxel_motion_strategy;

// ---- state ----
static OctomapBuilder g_builder;
static nav_msgs::Odometry g_latest_odom;
static bool g_odom_received = false;

static double g_pan_deg = 0.0;       // current gimbal pan (0-360°)
static double g_pan_vel_dps = 0.0;   // pan angular velocity (deg/s)
static ros::Time g_last_pan_time;
static double g_last_pan_deg = 0.0;
static bool g_pan_received = false;

static std::mutex g_odom_mutex;
static std::mutex g_pan_mutex;

// ---- params ----
static std::string g_octomap_prefix;
static ERPParams g_erp_params;
static RectSearchParams g_search_params;
static YawConstraintParams g_yaw_params;
static double g_strategy_rate = 4.0;  // Hz
static double g_vfov_min_deg = -90.0;
static double g_vfov_max_deg = 60.0;

// LiDAR extrinsic (vehicle → LiDAR mount)
static Eigen::Vector3d g_lidar_T(1.08, 0.0, 1.643);
static Eigen::Matrix3d g_R_lidar_to_vehicle = Eigen::Matrix3d::Identity();

// ---- helpers ----

static double shortestAngleDiff(double a_deg, double b_deg) {
  double d = std::fmod(a_deg - b_deg + 540.0, 360.0) - 180.0;
  return d;
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

// ---- callbacks ----

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(g_odom_mutex);
  g_latest_odom = *msg;
  g_odom_received = true;
}

void panCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() < 2) return;
  std::lock_guard<std::mutex> lock(g_pan_mutex);

  double t_now = msg->data[0];
  double pan_now = msg->data[1];

  if (g_pan_received) {
    double dt = t_now - g_last_pan_time.toSec();
    if (dt > 1e-6) {
      double diff = shortestAngleDiff(pan_now, g_last_pan_deg);
      g_pan_vel_dps = diff / dt;
    }
  }

  g_pan_deg = pan_now;
  g_last_pan_deg = pan_now;
  g_last_pan_time = ros::Time(t_now);
  g_pan_received = true;
}

// ---- strategy update ----

void strategyUpdate(const ros::TimerEvent&) {
  // 0. Check prerequisites
  if (!g_odom_received || !g_pan_received) {
    ROS_WARN_THROTTLE(5.0, "[Strategy] Waiting for odometry and pan data...");
    return;
  }

  // 1. Get current state (copy under locks)
  nav_msgs::Odometry odom;
  double pan_deg, pan_vel_dps;
  {
    std::lock_guard<std::mutex> l1(g_odom_mutex);
    odom = g_latest_odom;
  }
  {
    std::lock_guard<std::mutex> l2(g_pan_mutex);
    pan_deg = g_pan_deg;
    pan_vel_dps = g_pan_vel_dps;
  }

  double vehicle_yaw_deg = panToVehicleYaw(pan_deg);
  double vehicle_yaw_rad = vehicle_yaw_deg * M_PI / 180.0;
  double pan_vel_rps = pan_vel_dps * M_PI / 180.0;

  double dt = 1.0 / g_strategy_rate;

  // 2. Feasible yaw range (in vehicle frame)
  auto yr = computeFeasibleYawRange(vehicle_yaw_rad, pan_vel_rps, dt, g_yaw_params);
  if (!yr.valid) {
    ROS_WARN_THROTTLE(1.0, "[Strategy] Invalid yaw range — using current yaw only.");
    yr.min = vehicle_yaw_rad;
    yr.max = vehicle_yaw_rad;
    yr.valid = true;
  }

  // 3. Vehicle pose + LiDAR extrinsic
  const auto& pos  = odom.pose.pose.position;
  const auto& quat = odom.pose.pose.orientation;
  Eigen::Vector3d vehicle_pos_map(pos.x, pos.y, pos.z);
  Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
  Eigen::Matrix3d R_vehicle_to_map = q.toRotationMatrix();

  // Ray origin in map frame: vehicle + LiDAR offset
  Eigen::Vector3d origin_map = vehicle_pos_map + R_vehicle_to_map * g_lidar_T;

  // ERP frame: (θ=0, φ=0) → LiDAR native forward.
  // R_lidar_to_vehicle rotates LiDAR frame → vehicle frame.
  // R_erp_to_map = R_vehicle_to_map * R_lidar_to_vehicle
  // (R_lidar_to_vehicle is a Z-rotation, which commutes with gimbal yaw Rz(θ),
  //  so the pre-composition is correct.)
  Eigen::Matrix3d R_erp_to_map = R_vehicle_to_map * g_R_lidar_to_vehicle;

  // 4. Project ERP (full 360° for now; horizontal pruning via integral image)
  double h_min = -180.0 * M_PI / 180.0;
  double h_max =  180.0 * M_PI / 180.0;
  double v_min = g_vfov_min_deg * M_PI / 180.0;
  double v_max = g_vfov_max_deg * M_PI / 180.0;

  ERPImage erp = projectERP(g_builder, origin_map, R_erp_to_map,
                             h_min, h_max, v_min, v_max, g_erp_params);
  if (erp.empty()) {
    ROS_WARN_THROTTLE(5.0, "[Strategy] ERP projection failed.");
    return;
  }

  // 5. Integral image (with horizontal extension for wraparound)
  double fov_w_deg = g_search_params.fov_horizontal_deg;
  int h_extend = static_cast<int>(std::ceil(fov_w_deg / g_erp_params.resolution_deg));
  IntegralImage ii = IntegralImage::build(erp, h_extend);

  // 6. Search best rectangle (yaw in vehicle frame)
  auto result = searchBestRectangle(ii, erp, yr.min, yr.max, g_search_params);

  if (!result.valid) {
    ROS_WARN_THROTTLE(1.0, "[Strategy] Search returned no valid rectangle.");
    return;
  }

  // 7. Publish gimbal commands
  double target_yaw_deg = result.best_yaw_rad * 180.0 / M_PI;
  double target_pitch_deg = result.best_pitch_rad * 180.0 / M_PI;
  double target_pan_deg = vehicleYawToPan(target_yaw_deg);

  static ros::NodeHandle nh;
  static ros::Publisher pub = nh.advertise<cyber_msgs::GimbalCommand>("/gimbal_cmd", 10);

  // PAN command
  cyber_msgs::GimbalCommand pan_cmd;
  pan_cmd.header.stamp = ros::Time::now();
  pan_cmd.cmd = 0x4B;   // PAN absolute position
  pan_cmd.data = target_pan_deg;
  pub.publish(pan_cmd);

  // TILT command (published separately for PELCO-D)
  cyber_msgs::GimbalCommand tilt_cmd;
  tilt_cmd.header.stamp = ros::Time::now();
  tilt_cmd.cmd = 0x4D;   // TILT absolute position
  tilt_cmd.data = target_pitch_deg;
  pub.publish(tilt_cmd);

  ROS_INFO("[Strategy] cmd: yaw=%.1f°(pan=%.1f°) pitch=%.1f° score=%.1f λ=%.1f N=%d",
           target_yaw_deg, target_pan_deg, target_pitch_deg,
           result.best_score, result.lambda_min, result.N_eff);
}

// ---- main ----

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxel_motion_strategy_node");
  ros::NodeHandle nh("~");  // private namespace for params

  // Load params
  nh.param<std::string>("octomap_prefix", g_octomap_prefix, "");
  nh.param<double>("strategy_rate", g_strategy_rate, 4.0);

  nh.param<double>("erp_resolution_deg", g_erp_params.resolution_deg, 1.0);
  nh.param<double>("erp_range_max_m", g_erp_params.range_max_m, 150.0);
  nh.param<double>("erp_vfov_min_deg", g_vfov_min_deg, -90.0);
  nh.param<double>("erp_vfov_max_deg", g_vfov_max_deg, 60.0);

  // LiDAR extrinsic: [tx, ty, tz, roll, pitch, yaw]  (m, deg) — same format as iekf3d
  {
    std::vector<double> ext;
    nh.param<std::vector<double>>("lidar_extinct", ext, {1.08, 0.0, 1.643, 1.67, 0.49, 89.24});
    if (ext.size() >= 6) {
      g_lidar_T = Eigen::Vector3d(ext[0], ext[1], ext[2]);
      double roll = ext[3], pitch = ext[4], yaw = ext[5];
      g_R_lidar_to_vehicle =
          Eigen::AngleAxisd(yaw   * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll  * M_PI / 180.0, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
      ROS_INFO("LiDAR extrinsic: T=[%.2f,%.2f,%.2f] RPY=[%.1f,%.1f,%.1f]°",
               ext[0], ext[1], ext[2], roll, pitch, yaw);
    } else {
      ROS_WARN("lidar_extinct has < 6 elements, using default.");
    }
  }

  nh.param<double>("fov_horizontal_deg", g_search_params.fov_horizontal_deg, 60.0);
  nh.param<double>("fov_vertical_deg",   g_search_params.fov_vertical_deg,   68.0);
  nh.param<double>("yaw_step_deg",       g_search_params.yaw_step_deg,       3.0);
  nh.param<double>("pitch_step_deg",     g_search_params.pitch_step_deg,     3.0);
  nh.param<double>("pitch_min_deg",      g_search_params.pitch_min_deg,     -50.0);
  nh.param<double>("pitch_max_deg",      g_search_params.pitch_max_deg,      20.0);
  nh.param<double>("weight_pitch",       g_search_params.weight_pitch,       0.0);

  {
    double max_vel_dps, max_acc_dps2, deadzone_deg;
    nh.param<double>("max_angular_velocity_deg",     max_vel_dps, 60.0);
    nh.param<double>("max_angular_acceleration_deg", max_acc_dps2, 120.0);
    nh.param<double>("dyaw_deadzone_deg",            deadzone_deg, 5.0);
    g_yaw_params.max_angular_velocity     = max_vel_dps  * M_PI / 180.0;
    g_yaw_params.max_angular_acceleration = max_acc_dps2 * M_PI / 180.0;
    g_yaw_params.deadzone                 = deadzone_deg * M_PI / 180.0;
  }

  // Load octomap
  if (g_octomap_prefix.empty()) {
    ROS_ERROR("octomap_prefix is required!");
    return 1;
  }
  ROS_INFO("Loading octomap from %s ...", g_octomap_prefix.c_str());
  if (!g_builder.load(g_octomap_prefix)) {
    ROS_ERROR("Failed to load octomap!");
    return 1;
  }
  ROS_INFO("Octomap loaded: %zu voxels, %.2f m resolution",
           g_builder.numVoxels(), g_builder.resolution());

  // Subscribers (global handle for pub/sub in timer callback)
  ros::NodeHandle nh_global;
  ros::Subscriber odom_sub = nh_global.subscribe("/iekf3d/odometry", 10, odomCallback);
  ros::Subscriber pan_sub  = nh_global.subscribe("/pan", 10, panCallback);

  // Strategy timer
  ros::Timer timer = nh.createTimer(ros::Duration(1.0 / g_strategy_rate), strategyUpdate);

  ROS_INFO("Voxel Motion Strategy node started (rate=%.1f Hz).", g_strategy_rate);
  ros::spin();

  return 0;
}

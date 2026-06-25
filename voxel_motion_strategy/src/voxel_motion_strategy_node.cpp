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

static double g_pan_deg = 0.0;
static double g_pan_vel_dps = 0.0;
static ros::Time g_last_pan_time;
static double g_last_pan_deg = 0.0;
static bool g_pan_received = false;

static double g_tilt_deg = 0.0;
static ros::Time g_last_tilt_time;
static double g_last_tilt_deg = 0.0;
static bool g_tilt_received = false;

static std::mutex g_odom_mutex;
static std::mutex g_pan_mutex;
static std::mutex g_tilt_mutex;

// Target tracking (step-to-target)
static double g_target_pan_deg = 0.0;
static double g_target_pitch_deg = 0.0;
static bool g_has_target = false;
static ros::Time g_last_eval_time(0.0);

// ---- params ----
static std::string g_octomap_prefix;
static ERPParams g_erp_params;
static RectSearchParams g_search_params;
static MotionConstraintParams g_motion_params;
static double g_strategy_period = 2.0;   // s
static double g_rotation_period = 0.5;   // s
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

// pelco tilt [0,360) → ERP elevation (0=horiz, +=down)
// pelco: 0=horiz, 0..90=down, 270..360=up
static double pelcoTiltToErpPitch(double tilt_deg) {
  double pitch = tilt_deg;
  if (pitch > 180.0) pitch -= 360.0;
  return pitch;
}

// ERP elevation → pelco tilt [0,360)
static double erpPitchToPelcoTilt(double pitch_deg) {
  double tilt = pitch_deg;
  if (tilt < 0.0) tilt += 360.0;
  return tilt;
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

void tiltCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() < 2) return;
  std::lock_guard<std::mutex> lock(g_tilt_mutex);

  double t_now = msg->data[0];
  double tilt_now = msg->data[1];

  g_tilt_deg = tilt_now;
  g_last_tilt_deg = tilt_now;
  g_last_tilt_time = ros::Time(t_now);
  g_tilt_received = true;
}

// ---- strategy update (fired at rotation_period) ----

void strategyUpdate(const ros::TimerEvent&) {
  if (!g_odom_received || !g_pan_received || !g_tilt_received) {
    ROS_WARN_THROTTLE(5.0, "[Strategy] Waiting for odometry, pan and tilt data...");
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
    // Convert pelco tilt to ERP elevation for search
    double erp_pitch_deg = pelcoTiltToErpPitch(tilt_deg);
    double vehicle_yaw_rad = panToVehicleYaw(pan_deg) * M_PI / 180.0;
    double erp_pitch_rad = erp_pitch_deg * M_PI / 180.0;

    // Feasible yaw / pitch ranges
    double yaw_min = vehicle_yaw_rad - g_motion_params.pan_vel_max * g_strategy_period;
    double yaw_max = vehicle_yaw_rad + g_motion_params.pan_vel_max * g_strategy_period;

    double pitch_hard_min = g_search_params.pitch_min_deg * M_PI / 180.0;
    double pitch_hard_max = g_search_params.pitch_max_deg * M_PI / 180.0;
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
    Eigen::Matrix3d R_erp_to_map = R_vehicle_to_map * g_R_lidar_to_vehicle;

    // ERP projection (pruned to feasible yaw + FoV margin)
    double fov_hw_rad = g_search_params.fov_horizontal_deg * M_PI / 180.0 * 0.5;
    double h_min = yaw_min - fov_hw_rad;
    double h_max = yaw_max + fov_hw_rad;
    if (h_max - h_min < 1.0 * M_PI / 180.0) h_max = h_min + 1.0 * M_PI / 180.0;

    double v_min = g_vfov_min_deg * M_PI / 180.0;
    double v_max = g_vfov_max_deg * M_PI / 180.0;

    ERPImage erp = projectERP(g_builder, origin_map, R_erp_to_map,
                               h_min, h_max, v_min, v_max, g_erp_params);
    if (!erp.empty()) {
      IntegralImage ii = IntegralImage::build(erp, 0);

      auto result = searchBestRectangle(ii, erp, yaw_min, yaw_max,
                                         pitch_min, pitch_max, g_search_params);

      if (result.valid) {
        double target_yaw_deg = result.best_yaw_rad * 180.0 / M_PI;
        g_target_pan_deg = vehicleYawToPan(target_yaw_deg);
        // Convert ERP pitch to pelco tilt for step-to-target
        g_target_pitch_deg = erpPitchToPelcoTilt(result.best_pitch_rad * 180.0 / M_PI);
        g_has_target = true;
        g_last_eval_time = now;

        ROS_INFO("[Strategy] re-eval: target pan=%.1f deg tilt=%.1f deg (erp=%.1f deg) "
                 "score=%.1f lambda=%.1f N=%d "
                 "yaw_range=[%.0f, %.0f] deg pitch_range=[%.0f, %.0f] deg",
                 g_target_pan_deg, g_target_pitch_deg,
                 result.best_pitch_rad * 180.0 / M_PI,
                 result.best_score, result.lambda_min, result.N_eff,
                 yaw_min * 180.0 / M_PI, yaw_max * 180.0 / M_PI,
                 pitch_min * 180.0 / M_PI, pitch_max * 180.0 / M_PI);

        // LiDAR pose debug
        Eigen::Vector3d fwd = R_erp_to_map.col(0);
        double lidar_yaw = std::atan2(fwd.y(), fwd.x());
        double lidar_pitch = std::asin(fwd.z());
        ROS_DEBUG("[Strategy] LiDAR pose: pos=(%.2f,%.2f,%.2f) yaw=%.1f deg pitch=%.1f deg",
                  origin_map.x(), origin_map.y(), origin_map.z(),
                  lidar_yaw * 180.0 / M_PI, lidar_pitch * 180.0 / M_PI);
      } else {
        ROS_WARN_THROTTLE(1.0, "[Strategy] Search returned no valid rectangle.");
        return;
      }
    } else {
      ROS_WARN_THROTTLE(5.0, "[Strategy] ERP projection failed.");
      return;
    }
  }

  // 3. Step toward target (inner loop — every rotation_period)
  double pan_step_max = g_motion_params.pan_vel_max * g_rotation_period * 180.0 / M_PI;
  double tilt_step_max = g_motion_params.tilt_vel_max * g_rotation_period * 180.0 / M_PI;

  double dpan = shortestAngleDiff(g_target_pan_deg, pan_deg);
  double dpan_clamped = std::max(-pan_step_max, std::min(pan_step_max, dpan));
  double cmd_pan = std::fmod(pan_deg + dpan_clamped + 360.0, 360.0);

  double dtilt = shortestAngleDiff(g_target_pitch_deg, tilt_deg);
  double dtilt_clamped = std::max(-tilt_step_max, std::min(tilt_step_max, dtilt));
  double cmd_tilt = std::fmod(tilt_deg + dtilt_clamped + 360.0, 360.0);

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

  ROS_INFO("[Strategy] step: pan %.1f -> %.1f (d=%.1f max=%.1f) target=%.1f | "
           "tilt %.1f -> %.1f (d=%.1f max=%.1f) target=%.1f",
           pan_deg, cmd_pan, dpan_clamped, pan_step_max, g_target_pan_deg,
           tilt_deg, cmd_tilt, dtilt_clamped, tilt_step_max, g_target_pitch_deg);
}

// ---- main ----

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxel_motion_strategy_node");
  g_verbose = false;
  ros::NodeHandle nh("~");

  // Load params
  nh.param<std::string>("octomap_prefix", g_octomap_prefix, "");
  nh.param<double>("strategy_period", g_strategy_period, 2.0);
  nh.param<double>("rotation_period", g_rotation_period, 0.5);

  nh.param<double>("erp_resolution_deg", g_erp_params.resolution_deg, 1.0);
  nh.param<double>("erp_range_max_m", g_erp_params.range_max_m, 150.0);
  nh.param<double>("erp_self_range_m", g_erp_params.self_range_m, 2.0);
  nh.param<double>("erp_vfov_min_deg", g_vfov_min_deg, -90.0);
  nh.param<double>("erp_vfov_max_deg", g_vfov_max_deg, 60.0);

  // LiDAR extrinsic
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
      ROS_INFO("LiDAR extrinsic: T=[%.2f,%.2f,%.2f] RPY=[%.1f,%.1f,%.1f] deg",
               ext[0], ext[1], ext[2], roll, pitch, yaw);
    } else {
      ROS_WARN("lidar_extinct has < 6 elements, using default.");
    }
  }

  nh.param<double>("fov_horizontal_deg", g_search_params.fov_horizontal_deg, 60.0);
  nh.param<double>("fov_vertical_deg",   g_search_params.fov_vertical_deg,   68.0);
  nh.param<double>("pitch_min_deg",      g_search_params.pitch_min_deg,     -50.0);
  nh.param<double>("pitch_max_deg",      g_search_params.pitch_max_deg,      20.0);
  nh.param<double>("weight_pitch",       g_search_params.weight_pitch,       0.0);

  {
    double pan_vel_dps, tilt_vel_dps;
    nh.param<double>("pan_angular_velocity_deg",  pan_vel_dps,  30.0);
    nh.param<double>("tilt_angular_velocity_deg", tilt_vel_dps, 20.0);
    g_motion_params.pan_vel_max  = pan_vel_dps  * M_PI / 180.0;
    g_motion_params.tilt_vel_max = tilt_vel_dps * M_PI / 180.0;
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

  // Subscribers
  ros::NodeHandle nh_global;
  ros::Subscriber odom_sub  = nh_global.subscribe("/iekf3d/odometry", 10, odomCallback);
  ros::Subscriber pan_sub   = nh_global.subscribe("/pan", 10, panCallback);
  ros::Subscriber tilt_sub  = nh_global.subscribe("/tilt", 10, tiltCallback);

  // Timer: driven by rotation_period (inner loop), strategy re-eval embedded
  ros::Timer timer = nh.createTimer(ros::Duration(g_rotation_period), strategyUpdate);

  ROS_INFO("Voxel Motion Strategy node started: strategy_period=%.1fs rotation_period=%.2fs "
           "pan_vel=%.0f deg/s tilt_vel=%.0f deg/s",
           g_strategy_period, g_rotation_period,
           g_motion_params.pan_vel_max * 180.0 / M_PI,
           g_motion_params.tilt_vel_max * 180.0 / M_PI);
  ros::spin();

  return 0;
}

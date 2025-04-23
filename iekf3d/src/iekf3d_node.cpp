#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <execution>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <gps_common/conversions.h>
#include <oneapi/tbb.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/auto_io.h>
#include <pcl/search/flann_search.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <sophus/se3.hpp>
#include <tsl/robin_map.h>
#include <unsupported/Eigen/MatrixFunctions>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cyber_msgs/LocalizationEstimate.h>
#include <cyber_msgs/SpeedFeedback.h>
#include <cyber_msgs/SteerFeedback.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "reflcpp/core.h"
#include "reflcpp/yaml.h"
#include "sophus/so3.hpp"

#define IVAR(var) ROS_INFO_STREAM(#var " = " << var)
#define DVAR(var) ROS_DEBUG_STREAM(#var " = " << var)

using namespace std::chrono_literals;

struct ProgramConfigs {
  std::string map_path;
  std::string pointcloud2_topic;
  std::string gnss_topic;
  std::string imu_topic;
  std::string speed_topic;
  std::string steer_topic;

  std::vector<int> valid_gnss_status;

  std::vector<double> lidar_extinct;
  std::vector<double> imu_extinct;

  double voxel_leaf;
  double range_filter_min;
  double range_filter_max;

  double local_range;
  double update_range;

  std::vector<std::vector<double>> initialposes;
  double initializing_fitness_threshold;
  double initializing_translation_threshold;
  double initializing_rotation_threshold;

  int points_interval;

  double tracking_bad_fitness;

  struct ICP {
    int iterations;
    double distance_thres;
    double planefit_thres;
    double robust_kernel;
    double translation_epsilon;
    double rotation_epsilon;
  } icp;

  struct IEKF {
    int max_iterations;
    double max_duration_ms;
    double distance_thres;
    double robust_kernel;

    double attitude_std;
    double position_std;
    double angular_velocity_std;
    double linear_velocity_std;

    double icp_measure_std;
    double speed_measure_std;
    double imu_measure_std;

    double translation_epsilon;
    double rotation_epsilon;
    double linear_velocity_epsilon;
    double angular_velocity_epsilon;

    double max_possible_linear_velocity;
    double max_possible_angular_velocity;
  } iekf;

} cfg;
REFLCPP_METAINFO(ProgramConfigs::IEKF, ,
                 (max_iterations)(max_duration_ms)(distance_thres)(robust_kernel)(attitude_std)(position_std)(linear_velocity_std)(angular_velocity_std)(icp_measure_std)(speed_measure_std)(imu_measure_std)(translation_epsilon)(rotation_epsilon)(linear_velocity_epsilon)(angular_velocity_epsilon)(max_possible_linear_velocity)(max_possible_angular_velocity))
REFLCPP_METAINFO(
    ProgramConfigs::ICP, ,
    (iterations)(distance_thres)(planefit_thres)(robust_kernel)(translation_epsilon)(rotation_epsilon))
REFLCPP_METAINFO(
    ProgramConfigs, , (speed_topic)(steer_topic)(iekf)(lidar_extinct)(imu_extinct)(tracking_bad_fitness)(voxel_leaf)(range_filter_min)(range_filter_max)(initializing_fitness_threshold)(initializing_translation_threshold)(initializing_rotation_threshold)(valid_gnss_status)(map_path)(local_range)(update_range)(pointcloud2_topic)(gnss_topic)(imu_topic)(initialposes)(points_interval)(icp))

struct MapInfo {
  std::string points_file;
  std::string normal_file;
  std::vector<double> utm_origin;
  std::string zone;
} map_info;
REFLCPP_METAINFO(MapInfo, , (points_file)(normal_file)(utm_origin)(zone))

ros::Subscriber cloud_sub;
ros::Subscriber gnss_sub;
ros::Subscriber imu_sub;
ros::Subscriber initialpose_sub;
ros::Subscriber speed_sub;
ros::Subscriber steer_sub;

ros::Publisher preset_initialposes_pub;
ros::Publisher registration_pub;
ros::Publisher global_map_pub;
ros::Publisher local_map_pub;
ros::Publisher odometry_pub;
ros::Publisher estimate_pub;

tf2_ros::TransformBroadcaster *tf_pub;

struct RegistrationResults {
  Sophus::SE3d pose;
  Sophus::SE3d pose_se3;
  int iterations;
  double fitness;
};

auto execution_policy = std::execution::par_unseq;

std::mutex map_mtx;

pcl::PointCloud<pcl::Normal>::Ptr p_normal;
pcl::PointCloud<pcl::PointXYZ>::Ptr p_map;
pcl::search::Search<pcl::PointXYZ>::Ptr p_global_search;
pcl::search::Search<pcl::PointXYZ>::Ptr p_local_search;

double stamp = -1;

pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud_latest;

Sophus::SE3d Tbl;
Sophus::SO3d Tbi;

Sophus::SE3d pose;

bool wait_heading = true;
bool wait_position = true;

void local_map_update(const std::atomic_bool &require_stop = false) {
  Eigen::Vector3d local_map_center{1e11, 1e11, 1e11};
  pcl::search::Search<pcl::PointXYZ>::Ptr p_new_local_search;
  while (!require_stop) {
    std::this_thread::sleep_for(1s);
    if (wait_heading || wait_position) {
      continue;
    }
    pcl::PointXYZ pt;
    std::unique_lock lock(map_mtx);
    pt.x = pose.translation().x();
    pt.y = pose.translation().y();
    pt.z = pose.translation().z();
    lock.unlock();

    if (p_local_search != nullptr) {
      double distance_to_local_map_center =
          (local_map_center - pt.getVector3fMap().cast<double>()).norm();
      if (distance_to_local_map_center < cfg.update_range) {
        continue;
      }
    }

    local_map_center = pt.getVector3fMap().cast<double>();

    ROS_INFO("updating local map...");
    boost::shared_ptr<std::vector<int>> p_indices(new std::vector<int>);
    std::vector<float> unused;
    p_global_search->radiusSearch(pt, cfg.local_range, *p_indices, unused);

    p_new_local_search.reset(new pcl::search::KdTree<pcl::PointXYZ>);
    p_new_local_search->setInputCloud(p_map, p_indices);
    p_new_local_search->setSortedResults(true);

    lock.lock();
    p_local_search = std::move(p_new_local_search);
    lock.unlock();

    ROS_INFO("local map updated! (points_num: %d)", (int)p_indices->size());

    if (local_map_pub.getNumSubscribers() > 0) {
      sensor_msgs::PointCloud2 map_msg;
      pcl::PointCloud<pcl::PointXYZ> local_map;
      pcl::copyPointCloud(*p_map, *p_indices, local_map);
      pcl::toROSMsg(local_map, map_msg);
      map_msg.header.frame_id = "iekf_map";
      local_map_pub.publish(map_msg);
    }
  }
}

RegistrationResults registration(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &p_cloud,
    const pcl::search::Search<pcl::PointXYZ>::Ptr &p_search,
    Sophus::SE3d current_pose) {
  RegistrationResults results;

  std::vector<int> par_indices(p_cloud->size());
  std::iota(par_indices.begin(), par_indices.end(), 0);

  Eigen::MatrixXd A_full(p_cloud->size(), 6);
  Eigen::MatrixXd b_full(p_cloud->size(), 1);

  int iter = 0;
  for (iter = 0; iter < cfg.icp.iterations; iter++) {
    std::atomic_int points_num = 0;
    std::atomic_int64_t fitness1e4 = 0;
    std::for_each(
        execution_policy, par_indices.begin(), par_indices.end(), [&](int i) {
          const pcl::PointXYZ &pt_b = p_cloud->points[i];
          pcl::PointXYZ pt_w;
          pt_w.getVector4fMap() =
              current_pose.cast<float>() * pt_b.getVector4fMap();

          std::vector<int> k_indices;
          std::vector<float> k_distances;
          p_search->nearestKSearch(pt_w, 1, k_indices, k_distances);
          if (k_distances.empty() || k_distances[0] > cfg.icp.distance_thres)
            return;
          Eigen::Vector4d plane = p_normal->points[k_indices[0]]
                                      .getNormalVector4fMap()
                                      .cast<double>();
          if (plane.hasNaN()) return;

          pt_w.data[3] = 1;
          double e = plane.transpose() * pt_w.getVector4fMap().cast<double>();

          int locked_num = points_num++;

          Eigen::Matrix<double, 1, 3> J_e_pw = plane.head<3>().transpose();
          Eigen::Matrix<double, 3, 6> J_pw_t =
              Eigen::Matrix<double, 3, 6>::Zero();
          // J_pw_t << -(pt_b.getVector4fMap().cast<double>()),
          // Eigen::Matrix3d::Identity();
          J_pw_t << current_pose.so3().matrix(),
              -current_pose.so3().matrix() *
                  Sophus::SO3d::hat(pt_b.getVector3fMap().cast<double>());
          A_full.row(locked_num) = J_e_pw * J_pw_t;
          b_full(locked_num, 0) = -e;

          fitness1e4 += 1e4 / (1 + 10 * e);
        });

    if (points_num < 10) {
      results.fitness = 0;
      return results;
    }

    auto A = A_full.topRows(points_num);
    auto b = b_full.topRows(points_num);
    Sophus::SE3d::Tangent x =
        A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    current_pose *= Sophus::SE3d::exp(x);

    results.pose_se3 = current_pose;
    results.iterations = iter;
    results.fitness = fitness1e4 / 1e4 / p_cloud->size();

    double rot = x.tail<3>().norm();
    double tran = x.head<3>().norm();

    if (rot < cfg.icp.rotation_epsilon && tran < cfg.icp.translation_epsilon)
      break;
  }

  return results;
}

bool load_pose(const std::vector<double> &params, Sophus::SE3d &pose) {
  pose = Sophus::SE3d{};
  if (params.size() == 6) {
    pose.translation().x() = params[0];
    pose.translation().y() = params[1];
    pose.translation().z() = params[2];
    pose.so3() =
        (Eigen::AngleAxisd(params[3] * M_PI / 180, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(params[4] * M_PI / 180, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(params[5] * M_PI / 180, Eigen::Vector3d::UnitZ()))
            .matrix();
    return true;
  }
  if (params.size() == 7) {
    pose.translation().x() = params[0];
    pose.translation().y() = params[1];
    pose.translation().z() = params[2];
    Eigen::Quaterniond q;
    q.coeffs().x() = params[3];
    q.coeffs().y() = params[4];
    q.coeffs().z() = params[5];
    q.coeffs().w() = params[6];
    if (std::abs(q.norm() - 1.0) > 1e-6) {
      ROS_WARN("unnormalized quaternion.");
    }
    q.normalize();
    pose.so3() = q.matrix();
    return true;
  } else {
    ROS_ERROR("invalid pose format.");
    return false;
  }
}

bool initialize_from_preset_pose(pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud) {
  int best_initialpose_idx = -1;
  double best_initialpose_fitness = 0;
  Sophus::SE3d best_initialpose;
  for (int i = 0; i < cfg.initialposes.size(); i++) {
    Sophus::SE3d initialpose;
    bool ok = load_pose(cfg.initialposes[i], initialpose);
    if (!ok) {
      ROS_WARN("load initialposes[%d] failed! skipping...", i);
      continue;
    }
    auto results = registration(p_cloud, p_global_search, initialpose);
    Sophus::SE3d delta = initialpose.inverse() * results.pose;
    double translation = delta.translation().norm();
    Eigen::Quaterniond q{delta.so3().unit_quaternion()};
    double rotation = std::abs(std::acos(q.w()) * 2) / M_PI * 180;
    ROS_INFO(
        "try initializing from initialposes[%d]: fitness = %f, "
        "translation = %f, rotation = %f",
        i, results.fitness, translation, rotation);
    if (results.fitness < cfg.initializing_fitness_threshold) continue;
    if (translation > cfg.initializing_translation_threshold) continue;
    if (rotation > cfg.initializing_rotation_threshold) continue;
    if (results.fitness > best_initialpose_fitness) {
      best_initialpose_fitness = results.fitness;
      best_initialpose_idx = i;
      best_initialpose = results.pose;
    }
  }
  if (best_initialpose_idx == -1 ||
      best_initialpose_fitness < cfg.initializing_fitness_threshold) {
    return false;
  }
  pose = best_initialpose;
  wait_heading = wait_position = false;
  return true;
}

using Mat12d = Eigen::Matrix<double, 12, 12>;
using Vec12d = Eigen::Matrix<double, 12, 1>;

struct State {
  Sophus::SE3d pose = Sophus::SE3d{};
  Sophus::SE3d::Tangent velocity = Sophus::SE3d::Tangent::Zero();
};

Vec12d operator-(const State &a, const State &b) {
  Vec12d c;
  c.middleRows<6>(0) = (b.pose.inverse() * a.pose).log();
  c.middleRows<6>(6) = a.velocity - b.velocity;
  return c;
}

State operator+(const State &b, const Vec12d &c) {
  State a;
  a.pose = b.pose * Sophus::SE3d::exp(c.middleRows<6>(0));
  a.velocity = b.velocity + c.middleRows<6>(6);
  return a;
}

void state_predict(double delta, State &state, Mat12d &F) {
  Sophus::SE3d::Tangent tksi = delta * state.velocity;
  Sophus::SE3d etksi = Sophus::SE3d::exp(tksi);
  state.pose *= etksi;

  F.setIdentity();
  // pose wrt. pose
  F.block<6, 6>(0, 0) = etksi.inverse().Adj();
  // pose wrt. velocity
  F.block<6, 6>(0, 6) = Sophus::SE3d::leftJacobian(-tksi) * delta;
}

int state_measure_icp(const pcl::search::Search<pcl::PointXYZ> *p_search,
                      const pcl::PointCloud<pcl::PointXYZ> *p_cloud,
                      const State &state, Eigen::VectorXd &measures,
                      Eigen::MatrixXd &H, double *p_fitness = nullptr) {
  if (measures.rows() < p_cloud->size()) {
    measures = Eigen::VectorXd::Zero(p_cloud->size());
  }
  if (H.rows() < p_cloud->size()) {
    H = Eigen::MatrixXd::Zero(p_cloud->size(), 12);
  }

  std::atomic_int fitness1e4 = 0;
  std::atomic_int measure_idx = 0;
  std::for_each(
      execution_policy, p_cloud->begin(), p_cloud->end(),
      [&](const pcl::PointXYZ &pt) {
        pcl::PointXYZ pt_b = pt;
        pt_b.data[3] = 1;

        pcl::PointXYZ pt_w;
        pt_w.getVector3fMap() =
            state.pose.cast<float>() * pt_b.getVector3fMap();
        pt_w.data[3] = 1;

        std::vector<int> k_indices;
        std::vector<float> k_distances;
        p_search->nearestKSearch(pt_w, 1, k_indices, k_distances);
        if (k_distances.empty() || k_distances[0] > cfg.iekf.distance_thres)
          return;
        Eigen::Vector4d plane = p_normal->points[k_indices[0]]
                                    .getNormalVector4fMap()
                                    .cast<double>();
        if (plane.hasNaN()) return;

        int measure_idx_locked = measure_idx++;

        double e = plane.transpose() * pt_w.getVector4fMap().cast<double>();

        if (e > 0) {
          measures[measure_idx_locked] =
              std::log(1 + cfg.iekf.robust_kernel * e) / cfg.iekf.robust_kernel;
        } else {
          measures[measure_idx_locked] =
              -std::log(1 - cfg.iekf.robust_kernel * e) /
              cfg.iekf.robust_kernel;
        }
        double J_m_e = 1 / (1 + cfg.iekf.robust_kernel * e);
        Eigen::Matrix<double, 1, 3> J_e_pw = plane.transpose().head<3>();
        Eigen::Matrix<double, 3, 6> J_pw_t;
        J_pw_t << state.pose.so3().matrix(),
            -state.pose.so3().matrix() *
                Sophus::SO3d::hat(pt_b.getVector3fMap().cast<double>());
        // measure wrt. pose
        H.block<1, 6>(measure_idx_locked, 0) = J_m_e * J_e_pw * J_pw_t;

        fitness1e4 += 1e4 / (1 + 10 * std::abs(e));
      });
  if (p_fitness != nullptr) {
    *p_fitness = fitness1e4 / 1e4 / p_cloud->size();
  }
  return measure_idx;
}

void state_measure_vel(const State &state, double speed, double &measure,
                       Eigen::Matrix<double, 1, 12> &H) {
  measure = state.velocity.topRows<3>().x();
  H.setZero();
  H(0, 6) = 1;
}

void state_measure_rot(const State &state, double rot, double &measure,
                       Eigen::Matrix<double, 1, 12> &H) {
  measure = state.velocity.bottomRows<3>().z();
  H.setZero();
  H(0, 11) = 1;
}

State X;
Mat12d P = Mat12d::Identity();
Mat12d Q = Mat12d::Zero();

void iekf_predict(double delta) {
  Mat12d F;
  state_predict(delta, X, F);
  P = F * P * F.transpose() + Q;
}

bool iekf_update_icp(const pcl::search::Search<pcl::PointXYZ> *p_search,
                     const pcl::PointCloud<pcl::PointXYZ> *p_cloud) {
  auto iekf_update_begin = std::chrono::steady_clock::now();

  Eigen::MatrixXd H_full(p_cloud->size(), 12);
  Eigen::VectorXd hx_full(p_cloud->size());
  Eigen::MatrixXd K_full(12, p_cloud->size());
  int measures_num;

  State Xi = X;
  Mat12d Pi = P;
  double fitness;
  int iterations;
  for (iterations = 0; iterations < cfg.iekf.max_iterations; iterations++) {
    H_full.setZero();
    hx_full.setZero();
    measures_num =
        state_measure_icp(p_search, p_cloud, Xi, hx_full, H_full, &fitness);
    if (measures_num < 10) return false;
    if (fitness < cfg.tracking_bad_fitness) return false;

    auto H = H_full.topRows(measures_num);
    auto hx = hx_full.topRows(measures_num);
    auto K = K_full.leftCols(measures_num);
    double r = cfg.iekf.icp_measure_std * cfg.iekf.icp_measure_std;
    K = (H.transpose() * H + Pi.inverse() * r).inverse() * H.transpose();
    Vec12d W = K * (-hx);
    Xi = Xi + W;

    Mat12d Jk = Mat12d::Identity();
    Jk.block<6, 6>(0, 0) =
        Sophus::SE3d::leftJacobianInverse(-W.middleRows<6>(0, 0));
    Pi = Jk * P * Jk.transpose();

    double translation = W.middleRows<3>(0).norm();
    double rotation = W.middleRows<3>(3).norm();
    double linear_velocity = W.middleRows<3>(6).norm();
    double angular_velocity = W.middleRows<3>(9).norm();
    // PRINTVAR(translation);
    // PRINTVAR(rotation);
    // PRINTVAR(linear_velocity);
    // PRINTVAR(angular_velocity);
    if (translation < cfg.iekf.translation_epsilon &&
        rotation < cfg.iekf.rotation_epsilon &&
        linear_velocity < cfg.iekf.linear_velocity_epsilon &&
        angular_velocity < cfg.iekf.angular_velocity_epsilon)
      break;

    auto iekf_update_curr = std::chrono::steady_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(
        iekf_update_curr - iekf_update_begin);
    double iekf_update_ms = us.count() / 1e3;
    if (iekf_update_ms > cfg.iekf.max_duration_ms) {
      ROS_WARN(
          "iekf update iteration beyond the time limit, breaking... "
          "This may cause an inefficient update.");
      break;
    }
  }
  auto H = H_full.topRows(measures_num);
  auto K = K_full.leftCols(measures_num);
  X = Xi;
  P = (Mat12d::Identity() - K * H) * Pi;
  IVAR(iterations);
  IVAR(fitness);
  return true;
}

bool iekf_update_vel(double speed) {
  double measure;
  Eigen::Matrix<double, 1, 12> H;
  state_measure_vel(X, speed, measure, H);
  double R = cfg.iekf.speed_measure_std * cfg.iekf.speed_measure_std;
  Vec12d K = P * H.transpose() / (H * P * H.transpose() + R);
  X = X + K * (speed - measure);
  P = (Mat12d::Identity() - K * H) * P;
  return true;
}

bool iekf_update_rot(double rot) {
  double measure;
  Eigen::Matrix<double, 1, 12> H;
  state_measure_rot(X, rot, measure, H);
  double R = cfg.iekf.imu_measure_std * cfg.iekf.imu_measure_std;
  Vec12d K = P * H.transpose() / (H * P * H.transpose() + R);
  X = X + K * (rot - measure);
  P = (Mat12d::Identity() - K * H) * P;
  return true;
}

void read_normalized_stamp(const sensor_msgs::PointCloud2Ptr &p_msg,
                           std::vector<double> &stamps) {
  int points_num = p_msg->width * p_msg->height;
  stamps.resize(points_num);
  double stamp_min = std::numeric_limits<double>::infinity();
  sensor_msgs::PointCloud2ConstIterator<double> stamp_it(*p_msg, "timestamp");
  for (int i = 0; i < points_num && stamp_it != stamp_it.end();
       i++, ++stamp_it) {
    stamps[i] = *stamp_it;
    if (stamps[i] < stamp_min) stamp_min = stamps[i];
  }
  for (int i = 0; i < points_num; i++) {
    stamps[i] -= stamp_min;
  }
}

void deskew_pointcloud(pcl::PointCloud<pcl::PointXYZ> &points,
                       const std::vector<double> &stamps,
                       const Sophus::SE3d::Tangent &velocity) {
  ROS_ASSERT(points.size() == stamps.size());
  int points_num = points.size();
  for (int i = 0; i < points_num; i++) {
    points[i].getVector3fMap() =
        Sophus::SE3d::exp(-velocity * stamps[i]).cast<float>() *
        points[i].getVector3fMap();
  }
}

void pointcloud2_callback(sensor_msgs::PointCloud2Ptr p_msg) {
  auto t1 = std::chrono::steady_clock::now();
  ROS_DEBUG("check deshew");
  bool enable_deskew = false;
  for (auto f : p_msg->fields) {
    if (f.name == "timestamp") {
      enable_deskew = true;
      break;
    }
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*p_msg, (*p_cloud));
  // deskew
  if (!enable_deskew) {
    ROS_WARN_ONCE("pointcloud2 has no field 'timestamp', deskew turn off");
  } else {
    std::vector<double> stamps;
    ROS_DEBUG("read_normalized_stamp");
    read_normalized_stamp(p_msg, stamps);
    ROS_DEBUG("deskew_pointcloud");
    deskew_pointcloud(*p_cloud, stamps, X.velocity);
  }
  // downsample
  ROS_DEBUG("downsample");
  pcl::UniformSampling<pcl::PointXYZ> voxel;
  voxel.setInputCloud(p_cloud);
  voxel.setRadiusSearch(cfg.voxel_leaf);
  voxel.filter(*p_cloud);
  // range filter
  ROS_DEBUG("range filter");
  int points_num = 0;
  for (auto pt : *p_cloud) {
    if (pt.getVector3fMap().norm() < cfg.range_filter_min) continue;
    if (pt.getVector3fMap().norm() > cfg.range_filter_max) continue;
    (*p_cloud)[points_num++] = pt;
  }
  p_cloud->resize(points_num);
  // apply extinct
  ROS_DEBUG("apply extinct");
  for (auto &pt : *p_cloud) {
    pt.getVector3fMap() = Tbl.cast<float>() * pt.getVector3fMap();
  }

  if (wait_heading || wait_position) {
    // std::cout << "waiting initialposes..." << std::endl;
    bool ok = initialize_from_preset_pose(p_cloud);
    if (ok) ROS_INFO("initialized by preset pose");
    // stamp = p_msg->header.stamp.toSec();
    X.pose = pose;
    return;
  }

  pcl::search::Search<pcl::PointXYZ>::Ptr p_local_search_locked;
  {
    std::unique_lock lock(map_mtx);
    p_local_search_locked = p_local_search;
  }
  if (p_local_search_locked == nullptr) {
    // std::cout << "p_local_search_locked is nullptr..." << std::endl;
    return;
  }

  double prev_stamp = stamp;
  stamp = p_msg->header.stamp.toSec();

  if (prev_stamp > 0) {
    double stamp_delta = stamp - prev_stamp;
    ROS_WARN_COND(
        stamp_delta > 0.2,
        "Timestamp between two pointcloud is larger than 0.2 seconds. "
        "This may cause a matching failure.");

    ROS_DEBUG("iekf predict");
    iekf_predict(stamp_delta);
    ROS_DEBUG("iekf update");
    bool ok = iekf_update_icp(p_local_search_locked.get(), p_cloud.get());

    if (!ok) {
      ROS_WARN("iekf update failed. resetting...");
      wait_heading = wait_position = true;
      p_local_search = nullptr;
      X.velocity.setZero();
      P.setIdentity();
      return;
    }
  }

  double linear_velocity = X.velocity.topRows<3>().norm();
  if (linear_velocity > cfg.iekf.max_possible_linear_velocity) {
    ROS_WARN("linear velocity too large. resetting...");
    wait_heading = wait_position = true;
    p_local_search = nullptr;
    X.velocity.setZero();
    P.setIdentity();
    return;
  }
  double angular_velocity = X.velocity.bottomRows<3>().norm();
  if (angular_velocity > cfg.iekf.max_possible_angular_velocity) {
    ROS_WARN("angular velocity too large. resetting...");
    wait_heading = wait_position = true;
    p_local_search = nullptr;
    X.velocity.setZero();
    P.setIdentity();
    return;
  }
  {
    std::unique_lock lock(map_mtx);
    pose = X.pose;
  }

  if (registration_pub.getNumSubscribers() > 0) {
    pcl::PointCloud<pcl::PointXYZ> cloud_out = *p_cloud;
    for (int i = 0; i < cloud_out.size(); i++) {
      pcl::PointXYZ &pt = cloud_out[i];
      pt.getVector3fMap() = X.pose.cast<float>() * pt.getVector3fMap();
    }
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(cloud_out, msg_out);
    msg_out.header.frame_id = "iekf_map";
    registration_pub.publish(msg_out);
    p_cloud_latest = p_cloud;
  }

  cyber_msgs::LocalizationEstimate le_msg;
  le_msg.header.stamp = p_msg->header.stamp;
  gps_common::UTMtoLL(X.pose.translation().y() + map_info.utm_origin[1],
                      X.pose.translation().x() + map_info.utm_origin[0],
                      map_info.zone, le_msg.latitude, le_msg.longitude);
  le_msg.altitude = X.pose.translation().z() + map_info.utm_origin[2];
  le_msg.pose.position.x = X.pose.translation().x();
  le_msg.pose.position.y = X.pose.translation().y();
  le_msg.pose.position.z = X.pose.translation().z();
  le_msg.pose.orientation.x = X.pose.so3().unit_quaternion().x();
  le_msg.pose.orientation.y = X.pose.so3().unit_quaternion().y();
  le_msg.pose.orientation.z = X.pose.so3().unit_quaternion().z();
  le_msg.pose.orientation.w = X.pose.so3().unit_quaternion().w();
  le_msg.velocity.linear.x = X.velocity[0];
  le_msg.velocity.linear.y = X.velocity[1];
  le_msg.velocity.linear.z = X.velocity[2];
  le_msg.velocity.angular.x = X.velocity[3];
  le_msg.velocity.angular.y = X.velocity[4];
  le_msg.velocity.angular.z = X.velocity[5];
  le_msg.acceleration.linear.x = 0;
  le_msg.acceleration.linear.y = 0;
  le_msg.acceleration.linear.z = 0;
  le_msg.acceleration.angular.x = 0;
  le_msg.acceleration.angular.y = 0;
  le_msg.acceleration.angular.z = 0;
  estimate_pub.publish(le_msg);

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.frame_id = "iekf_map";
  tf_msg.header.stamp = p_msg->header.stamp;
  tf_msg.child_frame_id = "iekf_odom";
  tf_msg.transform.translation.x = pose.translation().x();
  tf_msg.transform.translation.y = pose.translation().y();
  tf_msg.transform.translation.z = pose.translation().z();
  tf_msg.transform.rotation.x = pose.so3().unit_quaternion().x();
  tf_msg.transform.rotation.y = pose.so3().unit_quaternion().y();
  tf_msg.transform.rotation.z = pose.so3().unit_quaternion().z();
  tf_msg.transform.rotation.w = pose.so3().unit_quaternion().w();
  tf_pub->sendTransform(tf_msg);

  nav_msgs::Odometry odom;
  odom.header.frame_id = "iekf_map";
  odom.header.stamp = p_msg->header.stamp;
  odom.child_frame_id = "iekf_odom";
  odom.pose.pose.position.x = X.pose.translation().x();
  odom.pose.pose.position.y = X.pose.translation().y();
  odom.pose.pose.position.z = X.pose.translation().z();
  odom.pose.pose.orientation.x = X.pose.so3().unit_quaternion().x();
  odom.pose.pose.orientation.y = X.pose.so3().unit_quaternion().y();
  odom.pose.pose.orientation.z = X.pose.so3().unit_quaternion().z();
  odom.pose.pose.orientation.w = X.pose.so3().unit_quaternion().w();
  for (int i = 0; i < 36; i++) {
    odom.pose.covariance[i] = P(i % 6, i / 6);
  }
  odom.twist.twist.linear.x = X.velocity[0];
  odom.twist.twist.linear.y = X.velocity[1];
  odom.twist.twist.linear.z = X.velocity[2];
  odom.twist.twist.angular.x = X.velocity[3];
  odom.twist.twist.angular.y = X.velocity[4];
  odom.twist.twist.angular.z = X.velocity[5];
  for (int i = 0; i < 36; i++) {
    odom.twist.covariance[i] = P(6 + i % 6, 6 + i / 6);
  }
  odometry_pub.publish(odom);

  auto t2 = std::chrono::steady_clock::now();
  double duration_ms =
      std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() /
      1e3;
  ROS_INFO("total time consumption: %lfms", duration_ms);
}

void initialpose_callback(geometry_msgs::PoseWithCovarianceStamped msg) {
  Eigen::Quaterniond q;
  q.coeffs().x() = msg.pose.pose.orientation.x;
  q.coeffs().y() = msg.pose.pose.orientation.y;
  q.coeffs().z() = msg.pose.pose.orientation.z;
  q.coeffs().w() = msg.pose.pose.orientation.w;
  pose = Sophus::SE3d{};
  pose.so3() = q.matrix();
  pose.translation().x() = msg.pose.pose.position.x;
  pose.translation().y() = msg.pose.pose.position.y;

  ROS_INFO("computing initialpose...");
  std::vector<float> Z;
  float check_size = 1.0;
  for (int i = 0; i < p_map->size(); i++) {
    double dx = (*p_map)[i].x - pose.translation().x();
    double dy = (*p_map)[i].y - pose.translation().y();
    if ((dx * dx + dy * dy) < check_size * check_size) {
      Z.push_back((*p_map)[i].z);
    }
  }
  if (!Z.empty()) {
    // cluster 2 types
    int iter = 0, maxIter = 10;
    int low_size = 0, high_size = 0;
    float z_low = 0, z_high = 0, pre_z_low, pre_z_high;
    pre_z_low = *std::min_element(Z.begin(), Z.end());
    pre_z_high = *std::max_element(Z.begin(), Z.end());

    do {
      iter++;
      z_low = z_high = 0;
      for (int i = 0; i < Z.size(); i++) {
        if (std::abs(Z[i] - pre_z_low) <= std::abs(Z[i] - pre_z_high)) {
          z_low += Z[i];
          low_size++;
        } else {
          z_high += Z[i];
          high_size++;
        }
      }
      if (low_size != 0 && high_size != 0) {
        z_low /= low_size;
        pre_z_low = z_low;
        z_high /= high_size;
        pre_z_high = z_high;
      }
    } while ((std::abs(z_low - pre_z_low) > 0.1 ||
              std::abs(z_high - pre_z_high) > 0.1) &&
             iter < maxIter);
    printf("zlow = %f, zhigh = %f\n", pre_z_low, pre_z_high);
    pose.translation().z() = pre_z_low;
  }
  X.pose = pose;
  X.velocity.setZero();
  wait_heading = wait_position = false;
  ROS_INFO("initialpose set");
}

void gnss_callback(sensor_msgs::NavSatFix msg) {
  if (!wait_position) return;
  if (std::find(cfg.valid_gnss_status.begin(), cfg.valid_gnss_status.end(),
                msg.status.status) == cfg.valid_gnss_status.end()) {
    ROS_WARN_THROTTLE(1, "gnss status invalid");
    return;
  }
  double x, y, z;
  std::string zone;
  gps_common::LLtoUTM(msg.latitude, msg.longitude, y, x, zone);
  if (zone != map_info.zone) {
    ROS_FATAL("zone & northp miss match between GPS and Map!");
    std::abort();
  }
  x -= map_info.utm_origin[0];
  y -= map_info.utm_origin[1];
  z = msg.altitude - map_info.utm_origin[2];
  pose.translation().x() = x;
  pose.translation().y() = y;
  pose.translation().z() = z;
  X.pose.translation().x() = x;
  X.pose.translation().y() = y;
  X.pose.translation().z() = z;
  wait_position = false;
  ROS_INFO("position update by gps.");

  if (!wait_position && !wait_heading) {
    Eigen::Quaterniond q(pose.so3().unit_quaternion());
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.frame_id = "iekf_map";
    tf_msg.header.stamp = msg.header.stamp;
    tf_msg.child_frame_id = "iekf_odom";
    tf_msg.transform.translation.x = pose.translation().x();
    tf_msg.transform.translation.y = pose.translation().y();
    tf_msg.transform.translation.z = pose.translation().z();
    tf_msg.transform.rotation.x = q.coeffs().x();
    tf_msg.transform.rotation.y = q.coeffs().y();
    tf_msg.transform.rotation.z = q.coeffs().z();
    tf_msg.transform.rotation.w = q.coeffs().w();
    tf_pub->sendTransform(tf_msg);
  }
}

void imu_callback(sensor_msgs::Imu msg) {
  if (wait_heading) {
    Eigen::Quaterniond q;
    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;

    pose.so3() = Tbi.matrix() * q;
    X.pose.so3() = pose.so3();
    wait_heading = false;
    ROS_INFO("heading update by gps.");

    if (!wait_position && !wait_heading) {
      Eigen::Quaterniond q(pose.so3().unit_quaternion());
      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.frame_id = "iekf_map";
      tf_msg.header.stamp = msg.header.stamp;
      tf_msg.child_frame_id = "iekf_odom";
      tf_msg.transform.translation.x = pose.translation().x();
      tf_msg.transform.translation.y = pose.translation().y();
      tf_msg.transform.translation.z = pose.translation().z();
      tf_msg.transform.rotation.x = q.coeffs().x();
      tf_msg.transform.rotation.y = q.coeffs().y();
      tf_msg.transform.rotation.z = q.coeffs().z();
      tf_msg.transform.rotation.w = q.coeffs().w();
      tf_pub->sendTransform(tf_msg);
    }
  }
  if (wait_heading || wait_position) return;
  Eigen::Vector3d rot;
  rot.x() = msg.angular_velocity.x;
  rot.y() = msg.angular_velocity.y;
  rot.z() = msg.angular_velocity.z;
  rot = Tbi * rot;
  iekf_update_rot(rot.z());
}

void speed_callback(cyber_msgs::SpeedFeedback msg) {
  if (wait_heading || wait_position) return;
  double speed;
  if (msg.gear == 9) {
    speed = -msg.speed_cms / 1e2;
  } else if (msg.gear == 11) {
    speed = msg.speed_cms / 1e2;
  } else {
    ROS_WARN("unknown speed_feedback gear: %d", msg.gear);
    return;
  }
  iekf_update_vel(speed);
}

void steer_callback(cyber_msgs::SteerFeedback msg) {
  /// TODO:
}

namespace fs = std::filesystem;

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " CONFIG" << std::endl;
    return -1;
  }
  auto cfg_path = fs::path(argv[1]);
  YAML::Node cfg_file = YAML::LoadFile(cfg_path);
  cfg = cfg_file.as<ProgramConfigs>();
  if (fs::path(cfg.map_path).is_relative()) {
    cfg.map_path = cfg_path.parent_path() / cfg.map_path;
  }
  map_info = YAML::LoadFile(fs::path(cfg.map_path) / "info.yaml").as<MapInfo>();
  std::cout << "here";
  if (!cfg.lidar_extinct.empty()) {
    ROS_ASSERT(cfg.lidar_extinct.size() == 6);
    Tbl.translation().x() = cfg.lidar_extinct[0];
    Tbl.translation().y() = cfg.lidar_extinct[1];
    Tbl.translation().z() = cfg.lidar_extinct[2];
    Tbl.so3() =
        (Eigen::AngleAxisd(cfg.lidar_extinct[3], Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(cfg.lidar_extinct[4], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(cfg.lidar_extinct[5], Eigen::Vector3d::UnitZ()))
            .matrix();
  }
  if (!cfg.imu_extinct.empty()) {
    ROS_ASSERT(cfg.imu_extinct.size() == 3);
    Tbi = (Eigen::AngleAxisd(cfg.imu_extinct[0], Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(cfg.imu_extinct[1], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(cfg.imu_extinct[2], Eigen::Vector3d::UnitZ()))
              .matrix();
  }

  Q.diagonal().middleRows<3>(0).array() =
      cfg.iekf.position_std * cfg.iekf.position_std;
  Q.diagonal().middleRows<3>(3).array() =
      cfg.iekf.attitude_std * cfg.iekf.attitude_std;
  Q.diagonal().middleRows<3>(6).array() =
      cfg.iekf.linear_velocity_std * cfg.iekf.linear_velocity_std;
  Q.diagonal().middleRows<3>(9).array() =
      cfg.iekf.angular_velocity_std * cfg.iekf.angular_velocity_std;

  p_map.reset(new pcl::PointCloud<pcl::PointXYZ>);
  p_normal.reset(new pcl::PointCloud<pcl::Normal>);
  ROS_INFO("utm_origin: [%lf, %lf]", map_info.utm_origin[0],
           map_info.utm_origin[1]);
  ROS_INFO("loading map...");
  pcl::io::load(fs::path(cfg.map_path) / map_info.points_file, *p_map);
  IVAR(p_map->size());
  std::string map_normal_file = fs::path(cfg.map_path) / map_info.normal_file;
  if (fs::exists(map_normal_file)) {
    pcl::io::load(map_normal_file, *p_normal);
  } else {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(p_map);
    ne.setRadiusSearch(0.3);
    ne.compute(*p_normal);
    pcl::io::save(map_normal_file, *p_normal);
  }
  IVAR(p_normal->size());
  for (int i = 0; i < p_map->size(); i++) {
    auto &n = p_normal->points[i];
    const auto &p = p_map->points[i];
    n.getNormalVector3fMap() /= n.getNormalVector3fMap().norm();
    n.data_n[3] = -n.getNormalVector3fMap().cast<double>().dot(
        p.getVector3fMap().cast<double>());
  }
  p_global_search.reset(new pcl::search::KdTree<pcl::PointXYZ>);
  p_global_search->setInputCloud(p_map);
  ROS_INFO("map initialized!");

  ros::init(argc, argv, "iekf3d");
  ros::NodeHandle nh;

  cloud_sub = nh.subscribe(cfg.pointcloud2_topic, 1, &pointcloud2_callback);
  gnss_sub = nh.subscribe(cfg.gnss_topic, 1, &gnss_callback);
  imu_sub = nh.subscribe(cfg.imu_topic, 1, &imu_callback);
  initialpose_sub = nh.subscribe("/initialpose", 1, &initialpose_callback);
  speed_sub = nh.subscribe(cfg.speed_topic, 1, &speed_callback);
  steer_sub = nh.subscribe(cfg.steer_topic, 1, &steer_callback);

  preset_initialposes_pub = nh.advertise<geometry_msgs::PoseArray>(
      "/iekf3d/preset_initialposes", 1, true);
  registration_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/iekf3d/registrationd_cloud", 1);
  global_map_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/iekf3d/global_map", 1, true);
  local_map_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/iekf3d/local_map", 1, true);
  odometry_pub = nh.advertise<nav_msgs::Odometry>("/iekf3d/odometry", 1);
  estimate_pub = nh.advertise<cyber_msgs::LocalizationEstimate>(
      "/localization/estimation", 1);

  tf_pub = new tf2_ros::TransformBroadcaster;

  sensor_msgs::PointCloud2 map_msg;
  pcl::toROSMsg(*p_map, map_msg);
  map_msg.header.frame_id = "iekf_map";
  global_map_pub.publish(map_msg);

  geometry_msgs::PoseArray initialposes_arr;
  initialposes_arr.header.frame_id = "iekf_map";
  for (const auto &params : cfg.initialposes) {
    Sophus::SE3d initailpose;
    bool ok = load_pose(params, initailpose);
    if (!ok) continue;
    geometry_msgs::Pose initailpose_msg;
    initailpose_msg.position.x = initailpose.translation().x();
    initailpose_msg.position.y = initailpose.translation().y();
    initailpose_msg.position.z = initailpose.translation().z();
    initailpose_msg.orientation.x = initailpose.so3().unit_quaternion().x();
    initailpose_msg.orientation.y = initailpose.so3().unit_quaternion().y();
    initailpose_msg.orientation.z = initailpose.so3().unit_quaternion().z();
    initailpose_msg.orientation.w = initailpose.so3().unit_quaternion().w();
    initialposes_arr.poses.emplace_back(initailpose_msg);
  }
  preset_initialposes_pub.publish(initialposes_arr);

  std::atomic_bool local_map_thread_require_stop = false;
  std::thread local_map_thread(&local_map_update,
                               std::ref(local_map_thread_require_stop));

  ros::spin();

  local_map_thread_require_stop = true;
  local_map_thread.join();

  return 0;
}

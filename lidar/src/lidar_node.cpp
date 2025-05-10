#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <vector>
#include <thread>
#include <mutex>
#include <algorithm>
#include <array>
#include <unordered_map>
#include <map>

#include <Eigen/Dense>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/io/auto_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/search.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/uniform_sampling.hpp>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <ros/ros.h>
#include "ros/package.h"
#include "nav_msgs/Odometry.h"
#include "gimbal/TimestampFloat.h"

#include "reflcpp/core.h"
#include "reflcpp/yaml.h"


class Profiler
{
public:
  Profiler(std::string tag) : tag(std::move(tag)) {}

  ~Profiler()
  {
    std::cout << "[" << tag << "]: " << duration_ns / 1e6 << "ms" << std::endl; // Test Yang
    //ROS_INFO_STREAM("[" << tag << "]: " << duration_ns / 1e6 << "ms");
  }

  void start() { t_start = std::chrono::steady_clock::now(); }

  void stop()
  {
    auto t_stop = std::chrono::steady_clock::now();
    duration_ns +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(t_stop - t_start)
            .count();
  }

private:
  std::string tag;
  std::uint64_t duration_ns = 0;
  std::chrono::time_point<std::chrono::steady_clock> t_start;
};
#define PROFILER(tag) Profiler profiler_##tag(#tag)

class TimestampConverter{
  private:
    std::map<std::string, double> timestamp_offsets;

  public:
    TimestampConverter()
    {
      timestamp_offsets["ros_stamp"] = 0;
    }


    /**
     * @brief Get the offset between the given ROS time and the given timestamp.
     *        Suggestted to use this function at each callback function's beginning
     *        to get the timestamp offset.
     *
     * @param timestamp_name The name associated with the timestamp.
     * @param rostime The ROS time to compare against the given timestamp. Usually the ros now time.
     * @param timestamp The timestamp(double type) to be gotten offset.
     */
    void get_offset(const std::string& timestamp_name, const ros::Time& rostime, const double& timestamp){
      if (timestamp_offsets.find(timestamp_name) != timestamp_offsets.end()) {
        return;
      }
      double offset = rostime.toSec() - timestamp;
      timestamp_offsets[timestamp_name] = offset;
    }

    double convert(const std::string& from_timestamp, const std::string& to_timestamp, const double& value){
        double from_offset = timestamp_offsets[from_timestamp];
        double to_offset = timestamp_offsets[to_timestamp];
        return value + (from_offset - to_offset);
    }
};

struct AngV{
  double ang_v_x;
  double ang_v_y;
  double ang_v_z;
};

enum OverlapMode{
  STATIC_MODE,
  DYNAMIC_MODE
};

struct ProgramConfigs{
  int overlap_mode;
  double voxel_leaf;
  double total_time_s;
  double wait_time_s;
  int frame_process_num;
  bool is_save_cloud;

  std::vector<double> car2gimbal_transformation;
} cfg;
REFLCPP_METAINFO(ProgramConfigs, 
,(overlap_mode)(voxel_leaf)(total_time_s)(wait_time_s)(frame_process_num)
(is_save_cloud)(car2gimbal_transformation));

// ros::Subscriber gimbal_sub_h;
// ros::Subscriber gimbal_sub_v;
ros::Subscriber cloud_sub;
ros::Subscriber imu_sub;
ros::Subscriber gimbal_sub_pan;
ros::Subscriber gimbal_sub_tilt;
ros::Subscriber gnss_sub;
ros::Subscriber car_imu_sub;

ros::Publisher cloud_pub;

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;

std::mutex map_mtx;

pcl::PointCloud<pcl::Normal>::Ptr p_map_normal;
CloudType::Ptr p_map;
pcl::search::Search<PointType>::Ptr p_global_search;
pcl::search::Search<PointType>::Ptr p_local_search;
std::vector<std::vector<std::array<float, 2>>> slam_areas;

gimbal::TimestampFloat gimbal_horizontal_angle;
gimbal::TimestampFloat gimbal_vertical_angle;
bool gimbal_inited_h = false;
bool gimbal_inited_v = false;

std::map<double, Eigen::Vector3f> imu_ang_v_vec;
std::map<double, float> h_ang_map;
std::map<double, float> v_ang_map;
std::map<double, Eigen::Quaternionf> q_rot_c_map;

// const double start_time_172 = 1723625932.263;
// const double start_time_329 = 329099317000 / 1e9;
const float pitch_shift = (0 - (0)) * M_PI / 180.0;
const float yaw_shift = (0 - (0)) * M_PI / 180.0;
const Eigen::Matrix3f init_rotation = Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX()).toRotationMatrix();
const Eigen::Vector3f init_translation = Eigen::Vector3f(0.05, 0, 0);
const double Avia_dt = 4.0 / 960000;
const double frame_T = 0.1;
const int frame_point_num = 24000;
double start_timestamp = 0.0;  // Will be set to ros::Time::now().toSec() in main()
Eigen::Matrix3f car2gimbal_rot = Eigen::Matrix3f::Identity();
Eigen::Vector3f car2gimbal_trans = Eigen::Vector3f(0.0, 0.0, 0.0);
//const double end_time_329 = 366099597000 / 1e9;

CloudType::Ptr p_cloud_complete(new CloudType);
TimestampConverter t_cvter;
Eigen::Vector3f v_w, v_g;
Eigen::Vector3f omega_c, omega_g;
Eigen::Matrix3f rot_c, rot_g;
Eigen::Vector3f trans_c;

bool is_point_valid(const PointType& pt)
{
 return pt.x != 0 && pcl::isFinite(pt) && pt.getVector3fMap().norm() < 1000;
}

void rotation_integral(const std::map<double, Eigen::Vector3f>& imu_ang_v_vec, double start_time, double end_time, Eigen::Matrix3f& rot)
{
  if (start_time == end_time)
  {
    rot = Eigen::Matrix3f::Identity();
    return;
  }
  else if (start_time > end_time)
  {
    rotation_integral(imu_ang_v_vec, end_time, start_time, rot);
    return;
  }
  Eigen::Matrix3f rot_integral = Eigen::Matrix3f::Identity();
  auto it = imu_ang_v_vec.lower_bound(start_time);
  if (it->first > end_time)
  {
    double dt = end_time - start_time;
    Eigen::AngleAxisf angle_axis(dt * it->second.norm(), it->second.normalized());
    rot = angle_axis.toRotationMatrix();
    return;
  }
  else
  {
    double dt = it->first - start_time;
    Eigen::AngleAxisf angle_axis(dt * it->second.norm(), it->second.normalized());
    rot_integral = angle_axis.toRotationMatrix();
  }
  while (it != imu_ang_v_vec.end() && it->first <= end_time) {
    double dt = (std::next(it) != imu_ang_v_vec.end() && std::next(it)->first <= end_time) ? 
                std::next(it)->first - it->first : end_time - it->first;
    Eigen::AngleAxisf angle_axis(dt * it->second.norm(), it->second.normalized());
    rot_integral = rot_integral * angle_axis.toRotationMatrix();
    ++it;
  }
  rot = rot_integral;
}

void gnss_callback(sensor_msgs::NavSatFix gps_msg)
{
  static bool is_init = false;
  static double last_timestamp = 0.0, last_x_w = 0.0, last_y_w = 0.0;
  geographic_msgs::GeoPointStampedPtr geo_msg(new geographic_msgs::GeoPointStamped());
  geo_msg->header = gps_msg.header;
  geo_msg->position.latitude = gps_msg.latitude;
  geo_msg->position.longitude = gps_msg.longitude;
  geo_msg->position.altitude = gps_msg.altitude;

  geodesy::UTMPoint utm_point;
  geodesy::fromMsg(geo_msg->position, utm_point);
  double timestamp_now = gps_msg.header.stamp.toSec();
  double x_w_now = utm_point.easting - 351425.09269358893;
  double y_w_now = utm_point.northing - 3433830.3251591502;
  trans_c = Eigen::Vector3f(x_w_now, y_w_now, 0);
  double delta_t = timestamp_now - last_timestamp;
  if (is_init)
  {
    if (delta_t < 1e-2)
      return;
    v_w = Eigen::Vector3f((x_w_now - last_x_w)/delta_t, (y_w_now - last_y_w)/delta_t, 0);
    last_timestamp = timestamp_now;
    last_x_w = x_w_now;
    last_y_w = y_w_now;
  }
  is_init = true;
  if (v_w.norm() > 100)
  {
    std::cout << "x_w_now: " << x_w_now << ", y_w_now: " << y_w_now << std::endl;
    std::cout << "last_x_w: " << last_x_w << ", last_y_w: " << last_y_w << std::endl;
    std::cout << "last_timestamp: " << std::fixed << std::setprecision(9) << last_timestamp << std::endl;
    std::cout << "timestamp_now: " << std::fixed << std::setprecision(9) << timestamp_now << std::endl;
  }
}

void car_imu_callback(sensor_msgs::Imu imu_data)
{
  t_cvter.get_offset("car_stamp", ros::Time::now(), imu_data.header.stamp.toSec());
  Eigen::Quaternionf q_rot_c(imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z);
  q_rot_c_map[t_cvter.convert("car_stamp", "ros_stamp", imu_data.header.stamp.toSec())] = q_rot_c;
  // rot_c = Eigen::Quaternionf(imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z).toRotationMatrix();
  // rot_g = car2gimbal * rot_c;
  omega_c = Eigen::Vector3f(imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z);
}

void gimbal_pan_callback(std_msgs::Float32MultiArray msg)
{
  t_cvter.get_offset("gimbal_stamp", ros::Time::now(), msg.data[0]);
  gimbal_horizontal_angle.header.stamp.fromSec(t_cvter.convert("gimbal_stamp", "ros_stamp", msg.data[0]));
  // gimbal_horizontal_angle.header.stamp.fromSec(msg.data[0]);
  gimbal_horizontal_angle.value = -msg.data[1] * M_PIq / 180.0; // Notice the sign.
  gimbal_inited_h = true;
  h_ang_map[gimbal_horizontal_angle.header.stamp.toSec()] = gimbal_horizontal_angle.value;
}

void gimbal_tilt_callback(std_msgs::Float32MultiArray msg)
{
  t_cvter.get_offset("gimbal_stamp", ros::Time::now(), msg.data[0]);
  gimbal_vertical_angle.header.stamp.fromSec(t_cvter.convert("gimbal_stamp", "ros_stamp", msg.data[0]));
  // gimbal_vertical_angle.header.stamp.fromSec(msg.data[0]);
  // ROS_INFO_STREAM("msg_time:" << std::fixed << std::setprecision(9) << msg.data[0]);
  // ROS_INFO_STREAM("Gimbal time:" << std::fixed << std::setprecision(9) << gimbal_vertical_angle.header.stamp.toSec());
  gimbal_vertical_angle.value = (msg.data[1]-(-3)) * M_PIq / 180.0;
  gimbal_inited_v = true;
  v_ang_map[gimbal_vertical_angle.header.stamp.toSec()] = gimbal_vertical_angle.value;
}

void imu_callback(sensor_msgs::Imu imu_data)
{
  t_cvter.get_offset("Avia_stamp", ros::Time::now(), imu_data.header.stamp.toSec());
  Eigen::Vector3f ang_v(imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z);
  imu_ang_v_vec[t_cvter.convert("Avia_stamp", "ros_stamp", imu_data.header.stamp.toSec())] = ang_v;
}

void pointcloud2_callback(sensor_msgs::PointCloud2Ptr p_msg)
{
  t_cvter.get_offset("Avia_stamp", ros::Time::now(), p_msg->header.stamp.toSec());
  const static double start_timestamp_local = start_timestamp;
  if (ros::Time::now().toSec() - start_timestamp_local < cfg.wait_time_s || !gimbal_inited_h || !gimbal_inited_v)
  {
    //std::cout << "Frame skipped." << std::endl;
    return;
  }
  PROFILER(pointcloud2_callback);
  profiler_pointcloud2_callback.start();
  CloudType::Ptr p_cloud(new CloudType);
  pcl::moveFromROSMsg(*p_msg, (*p_cloud));

  // pcl::UniformSampling<PointType> voxel;
  // voxel.setInputCloud(p_cloud);
  // voxel.setRadiusSearch(cfg.voxel_leaf);
  // voxel.filter(*p_cloud);

  const static double point_time_start = ros::Time::now().toSec() - frame_T;

  double frame_time = t_cvter.convert("Avia_stamp", "ros_stamp", p_msg->header.stamp.toSec());
  int frame_point_idx = std::round((frame_time - point_time_start) / frame_T) * frame_point_num;
  int point_idx = frame_point_idx;
  double point_time = frame_time;
  int valid_point_count = 0;
  for (const auto& point : p_cloud->points) {
    if (is_point_valid(point)) {
      valid_point_count++;
    }
  }

  //std::cout << "Point time:" << point_time << std::endl;
  ROS_INFO_STREAM("Point time:" << std::fixed << std::setprecision(9) << point_time);
  std::cout << "Valid point count:" << valid_point_count << std::endl;

  //Get the frame's initial pose by feedback.
  if (h_ang_map.size() < 3 || v_ang_map.size() < 3)
  {
    ROS_INFO_STREAM("\033[91m" << "Not enough gimbal data." << "\033[0m");
    return;
  }
  auto it_h_ang = --h_ang_map.lower_bound(point_time);
  auto it_h_ang_prev = --(--h_ang_map.lower_bound(point_time));
  auto it_v_ang = --v_ang_map.lower_bound(point_time);
  auto it_v_ang_prev = --(--v_ang_map.lower_bound(point_time)); 
  double comp_head_time = it_h_ang->first;  
  float yaw_g_angle = it_h_ang->second - yaw_shift;
  float pitch_g_angle = it_v_ang->second - pitch_shift;
  pitch_g_angle += (comp_head_time - it_v_ang_prev->first) * (it_v_ang->second - it_v_ang_prev->second) / (it_v_ang->first - it_v_ang_prev->first);

  float &pitch_l_angle = yaw_g_angle; 
  float &minus_yaw_l_angle = pitch_g_angle;
  Eigen::Matrix3f yaw_l_fr = Eigen::AngleAxisf((-minus_yaw_l_angle), Eigen::Vector3f::UnitZ()).toRotationMatrix();
  Eigen::Matrix3f pitch_l_fr = Eigen::AngleAxisf((pitch_l_angle), Eigen::Vector3f::UnitY()).toRotationMatrix();
  Eigen::Matrix4f frame_init_pose = Eigen::Matrix4f::Identity();
  frame_init_pose.topLeftCorner<3, 3>() = init_rotation * pitch_l_fr * yaw_l_fr;

  CloudType::Ptr p_cloud_out(new CloudType);
  p_cloud_out->resize(p_cloud->size());
  int interval = std::floor(frame_point_num / cfg.frame_process_num);
  std::map<double, Eigen::Matrix3f> frame_imu_rot_map;
  auto it_imu = --imu_ang_v_vec.lower_bound(frame_time);
  switch (cfg.overlap_mode)
  {
    case STATIC_MODE:
      pcl::transformPointCloud(*p_cloud, *p_cloud_out, frame_init_pose);
      point_time += frame_T;
      break;
    case DYNAMIC_MODE:
      // Calculate the rotation in imu stamps.
      while (it_imu != imu_ang_v_vec.end() && it_imu->first <= frame_time + frame_T)
      {
        if (it_imu == imu_ang_v_vec.begin())
        {
          ++it_imu;
          continue;
        }
        Eigen::Matrix3f rot_imu = Eigen::Matrix3f::Identity();
        rotation_integral(imu_ang_v_vec, comp_head_time, it_imu->first, rot_imu);
        frame_imu_rot_map[it_imu->first] = rot_imu;
        ++it_imu;
      }

      // Calculate the rotation in lidar point stamps.
      #pragma omp parallel for schedule(static)
      for (int i = 0; i < interval * cfg.frame_process_num; i+=interval)
      {
        if (!is_point_valid(p_cloud->points[i]))
          continue;
        int point_idx = frame_point_idx + i;
        //Get the point's time.
        double point_time = point_time_start + point_idx * Avia_dt;    
        
        //Get the point's rotation.
        auto it_lower_imu_rot = --frame_imu_rot_map.lower_bound(point_time);
        Eigen::Matrix3f rot_delta = Eigen::Matrix3f::Identity();
        rotation_integral(imu_ang_v_vec, it_lower_imu_rot->first, point_time, rot_delta);
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.topLeftCorner<3, 3>() = frame_init_pose.topLeftCorner<3, 3>() * it_lower_imu_rot->second * rot_delta; 
        Eigen::Vector3f translation = pose.topLeftCorner<3, 3>() * init_translation;
        pose.topRightCorner<3, 1>() = translation;

        //Calculate the point's position.
        for (int j = 0; j < interval && i+j<interval*cfg.frame_process_num; j++)
        {
          int idx = i + j;
          if (!is_point_valid(p_cloud->points[idx]))
            continue;
          PointType pt;
          pt.getVector4fMap() = pose * p_cloud->points[idx].getVector4fMap(); 
          pt.intensity = p_cloud->points[idx].intensity;
          p_cloud_out->points[idx] = pt;
        }
      }
      break;
    default:
      ROS_INFO_STREAM("\033[91m" << "Invalid overlap mode." << "\033[0m");
      break;
  }

  sensor_msgs::PointCloud2 msg_out;
  Eigen::Matrix4f real_pose = Eigen::Matrix4f::Identity();
  // real_pose.topLeftCorner<3, 3>() = car2gimbal_rot *  q_rot_c_map.lower_bound(comp_head_time)->second.toRotationMatrix();
  // real_pose.topRightCorner<3, 1>() = trans_c + car2gimbal_trans;
  real_pose.topLeftCorner<3, 3>() = car2gimbal_rot.transpose();
  real_pose.topRightCorner<3, 1>() = -car2gimbal_trans;
  pcl::transformPointCloud(*p_cloud_out, *p_cloud_out, real_pose);
  pcl::toROSMsg(*p_cloud_out, msg_out);
  msg_out.header.frame_id = "iekf_map";
  msg_out.header.stamp = ros::Time(frame_time);
  cloud_pub.publish(msg_out);

  static bool is_save = !cfg.is_save_cloud;//the '!' is right, but need some time to understand.
  if (!is_save)
    *p_cloud_complete += *p_cloud_out;
  
  // std::stringstream ss;
  // ss << "data/pc" << static_cast<int>(frame_time)  << ".ply";
  // if (pcl::io::savePLYFile(ss.str(), *p_cloud_out)==0)
  // {
  //   ROS_INFO_STREAM("\033[92m" << "Successful to save point cloud." << "\033[0m");
  // }
  // else
  // {
  //   ROS_ERROR_STREAM("\033[91m" << "Failed to save point cloud." << "\033[0m");
  // }

  if (point_time > point_time_start + cfg.total_time_s && !is_save)
  {
    is_save = true;
    ROS_INFO_STREAM("\033[92m" << "Saving point cloud to pc.ply..." << "\033[0m");
    if (pcl::io::savePLYFile("data/pc.ply", *p_cloud_complete) == 0) {
        ROS_INFO_STREAM("\033[92m" << "Successful to save point cloud." << "\033[0m");
        //std::cout << "Successful to save point cloud." << std::endl;
    } else {
        ROS_ERROR_STREAM("\033[91m" << "Failed to save point cloud." << "\033[0m");
        //std::cerr << "Failed to save point cloud." << std::endl;
    }
  }

  // if (pcl::io::savePLYFile("pc_com/pc_compensate_" + std::to_string(p_cloud->header.stamp) + ".ply", *p_cloud_out) == 0) {
  //     std::cout << "Successful to save point cloud." << std::endl;
  // } else {
  //     std::cerr << "Failed to save point cloud." << std::endl;
  // }

  profiler_pointcloud2_callback.stop();
}

namespace fs = std::filesystem;

#define PRINT(var) std::cout << #var " = " << var << std::endl

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_node");
  ros::NodeHandle nh;
  ROS_INFO("Lidar node started");
  start_timestamp = ros::Time::now().toSec();

  std::string strSrcFolder = ros::package::getPath("lidar");

  cfg = YAML::LoadFile(strSrcFolder + "/configs/configs.yaml")
          .as<ProgramConfigs>();
  std::cout << "overlap_mode: " << cfg.overlap_mode << std::endl
            << "voxel_leaf: " << cfg.voxel_leaf << std::endl
            << "total_time_s: " << cfg.total_time_s << std::endl
            << "wait_time_s: " << cfg.wait_time_s << std::endl
            << "frame_process_num: " << cfg.frame_process_num << std::endl
            << "is_save_cloud: " << cfg.is_save_cloud << std::endl;
  // cfg.overlap_mode = 0;
  // cfg.voxel_leaf = 0.1;
  // cfg.total_time_s = 30.0;
  // cfg.wait_time_s = 10.0;
  ROS_INFO_STREAM("\033[92m" << "Configs Loaded" << "\033[0m");

  car2gimbal_trans = Eigen::Vector3f(
    cfg.car2gimbal_transformation[0], 
    cfg.car2gimbal_transformation[1], 
    cfg.car2gimbal_transformation[2]);
  car2gimbal_rot = Eigen::AngleAxisf(cfg.car2gimbal_transformation[3] / 180 * M_PI, Eigen::Vector3f::UnitZ()).toRotationMatrix()
                  * Eigen::AngleAxisf(cfg.car2gimbal_transformation[4] / 180 * M_PI, Eigen::Vector3f::UnitY()).toRotationMatrix()
                  * Eigen::AngleAxisf(cfg.car2gimbal_transformation[5] / 180 * M_PI, Eigen::Vector3f::UnitX()).toRotationMatrix();

  // Notice the queue sizes.
  imu_sub = nh.subscribe("/livox/imu", 25, &imu_callback);
  cloud_sub = nh.subscribe("/livox/lidar", 1, &pointcloud2_callback);
  // gimbal_sub_h = nh.subscribe("/horizontal_angle", 1, &gimbal_horizontal_callback);
  // gimbal_sub_v = nh.subscribe("/vertical_angle", 1, &gimbal_vertical_callback);
  gimbal_sub_pan = nh.subscribe("pan", 4, &gimbal_pan_callback);
  gimbal_sub_tilt = nh.subscribe("tilt", 4, &gimbal_tilt_callback);
  gnss_sub = nh.subscribe("/Inertial/gps/fix", 15, &gnss_callback);
  car_imu_sub = nh.subscribe("/Inertial/imu/data", 15, &car_imu_callback);

  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/registered_cloud", 1);

  ros::spin();

  return 0;
}

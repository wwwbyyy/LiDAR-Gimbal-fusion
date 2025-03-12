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

  std::string map_path;
  float map_voxel_leaf;
  float normal_estimate_radius;
  std::vector<float> initialpose;
  float local_range;
  float update_range;
  int points_interval;
  struct ICP
  {
    float voxel_leaf;
    int iterations;
    float distance_thres;
    float planefit_thres;
    float robust_kernel;
    float translation_epsilon;
    float rotation_epsilon;
    int min_points_per_frame;
  } icp;
} cfg;
REFLCPP_METAINFO(ProgramConfigs, 
,(overlap_mode)(voxel_leaf)(total_time_s)(wait_time_s)(frame_process_num)
(map_path)(map_voxel_leaf)(normal_estimate_radius)
(initialpose)(icp)(local_range)(update_range)(points_interval)(is_save_cloud));
REFLCPP_METAINFO(ProgramConfigs::ICP, 
,(voxel_leaf)(iterations)(distance_thres)(planefit_thres)(robust_kernel)
(translation_epsilon)(rotation_epsilon)(min_points_per_frame));

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

std::map<double, AngV> imu_ang_v;
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
const Eigen::Matrix3f car2gimbal = (Eigen::AngleAxisf(-M_PI * 145.5/180, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-M_PI*3.1/180, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(-M_PI*3.5/180, Eigen::Vector3f::UnitX())).toRotationMatrix();
//const double end_time_329 = 366099597000 / 1e9;

CloudType::Ptr p_cloud_complete(new CloudType);
TimestampConverter t_cvter;
Eigen::Vector3f v_w, v_g;
Eigen::Vector3f omega_c, omega_g;
Eigen::Matrix3f rot_c, rot_g;
Eigen::Vector3f trans_c;

bool is_point_valid(const PointType& pt)
{
  return pt.x != 0;
}

/**
 * Calculates the integral of angle change over a specified time range.
 *
 * @param imu_ang_v A map containing the angular velocities at different timestamps.
 * @param start_time The starting time of the range.
 * @param end_time The ending time of the range.
 * @param xyz_order The order of the XYZ axis (1 for X, 2 for Y, 3 for Z).
 * @return The total angle change over the specified time range.
 */
double angle_integral(const std::map<double, AngV>& imu_ang_v, double start_time, double end_time, int xyz_order)
{
  double angle_change = 0.0;

  // 获取在指定时间范围内的角速度值
  std::map<double, double> filtered_values;
  for (const auto& pair : imu_ang_v) {
      if (pair.first >= start_time && pair.first <= end_time) {
        switch (xyz_order)
        {
        case 1:
          filtered_values[pair.first] = pair.second.ang_v_x;
          break;
        case 2:
          filtered_values[pair.first] = pair.second.ang_v_y;
          break;        
        case 3:
          filtered_values[pair.first] = pair.second.ang_v_z;
          break;        
        default:
          break;
        }
      }
  }

  //std::cout << filtered_values.size() << std::endl;
  // 如果没有符合条件的数据点，则返回0
  if (filtered_values.empty()) {
      //std::cerr << "No data between two times:" << start_time << "," << end_time << std::endl;
      auto closest = --imu_ang_v.lower_bound(end_time);
      switch (xyz_order)
      {
        case 1:
          angle_change = closest->second.ang_v_x * (end_time - start_time);
          break;
        case 2:
          angle_change = closest->second.ang_v_y * (end_time - start_time);
          break;        
        case 3:
          angle_change = closest->second.ang_v_z * (end_time - start_time);
          break;        
        default:
          angle_change = 0.0;
          break;
      }
      return angle_change;
    }

  // 处理从 start_time 到 filtered_values 的第一个时间点的积分
  auto first_pair = filtered_values.begin();
  double first_time = first_pair->first;
  double first_velocity = first_pair->second; // 第一个时间点的角速度
  
  if (start_time < first_time) {
      // 积分从 start_time 到 first_time
      double dt_start = first_time - start_time;
      angle_change += first_velocity * dt_start; // 使用简单的矩形法
  }

  // 在 filtered_values 中进行积分（中间部分）
  auto it = filtered_values.begin();
  double last_time = it->first;
  double last_velocity = it->second; // 第一个时间点的角速度

  ++it; // 移动到下一个元素

  while (it != filtered_values.end()) {
      double current_time = it->first;
      double current_velocity = it->second; // 当前时间点的角速度
      
      // 使用梯形法计算在这个时间段内转过的角度
      double dt = current_time - last_time; // 时间差
      //std::cout << current_velocity << ", " << dt << std::endl;
      angle_change += (last_velocity + current_velocity) / 2.0 * dt; // 梯形法积分
      
      // 更新 last_time 和 last_velocity
      last_time = current_time;
      last_velocity = current_velocity;

      ++it; // 移动到下一个元素
  }

  // 处理从 filtered_values 的最后一个时间点到 end_time 的积分
  auto last_pair = --filtered_values.end(); // 最后一个元素
  double last_time_filtered = last_pair->first;
  double last_velocity_filtered = last_pair->second; // 最后一个时间点的角速度

  if (end_time > last_time_filtered) {
      // 积分从 last_time_filtered 到 end_time
      double dt_end = end_time - last_time_filtered;
      angle_change += last_velocity_filtered * dt_end; // 使用简单的矩形法
  }

  return angle_change; // 返回在时间区间内转过的总角度
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
  gimbal_horizontal_angle.value = -msg.data[1] * M_PIq / 180.0;
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
  double dot_product = imu_data.linear_acceleration.x * imu_data.angular_velocity.x
                     + imu_data.linear_acceleration.y * imu_data.angular_velocity.y;
  double ang_v_y = 0;
  if (dot_product > 0)
    ang_v_y = std::sqrt(imu_data.angular_velocity.x * imu_data.angular_velocity.x + imu_data.angular_velocity.y * imu_data.angular_velocity.y);
  else
    ang_v_y = -std::sqrt(imu_data.angular_velocity.x * imu_data.angular_velocity.x + imu_data.angular_velocity.y * imu_data.angular_velocity.y);
  AngV ang_v_tmp = {
    0,
    ang_v_y,
    imu_data.angular_velocity.z
  };
  imu_ang_v[t_cvter.convert("Avia_stamp", "ros_stamp", imu_data.header.stamp.toSec())] = ang_v_tmp;
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

  map_mtx.lock();
  //Get the frame's initial pose by feedback.
  auto it_h_ang = --h_ang_map.lower_bound(point_time);
  auto it_v_ang = --v_ang_map.lower_bound(point_time);
  float yaw_g_angle = it_h_ang->second - yaw_shift;
  float &pitch_l_angle = yaw_g_angle; 
  float pitch_g_angle = it_v_ang->second - pitch_shift;
  float &minus_yaw_l_angle = pitch_g_angle;
  Eigen::Matrix3f yaw_l_fr = Eigen::AngleAxisf((-minus_yaw_l_angle), Eigen::Vector3f::UnitZ()).toRotationMatrix();
  Eigen::Matrix3f pitch_l_fr = Eigen::AngleAxisf((pitch_l_angle), Eigen::Vector3f::UnitY()).toRotationMatrix();
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.topLeftCorner<3, 3>() = init_rotation * pitch_l_fr * yaw_l_fr;
  map_mtx.unlock();

  CloudType::Ptr p_cloud_out(new CloudType);
  p_cloud_out->resize(p_cloud->size());
  int interval = std::floor(frame_point_num / cfg.frame_process_num);
  switch (cfg.overlap_mode)
  {
    case STATIC_MODE:
      pcl::transformPointCloud(*p_cloud, *p_cloud_out, pose);
      point_time += frame_T;
      break;
    case DYNAMIC_MODE:
      #pragma omp parallel for
      for (int i = 0; i < interval * cfg.frame_process_num; i+=interval)
      {
        if (!is_point_valid(p_cloud->points[i]))
          continue;
        int point_idx = frame_point_idx + i;
        //Get the point's time.
        double point_time = point_time_start + point_idx * Avia_dt;    
        
        //Calculate the point's pose.
        double dyaw_l_angle = 0, dpitch_l_angle = 0;
        //If it's the first point in a frame, integral from the feedback.
        dyaw_l_angle = angle_integral(imu_ang_v, it_v_ang->first, point_time, 3);
        dpitch_l_angle = angle_integral(imu_ang_v, it_h_ang->first, point_time, 2) - omega_c.z() * (frame_time - it_h_ang->first);
        Eigen::Matrix3f yaw_l_pt = Eigen::AngleAxisf(-minus_yaw_l_angle + dyaw_l_angle, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        Eigen::Matrix3f pitch_l_pt = Eigen::AngleAxisf(pitch_l_angle + dpitch_l_angle, Eigen::Vector3f::UnitY()).toRotationMatrix();
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.topLeftCorner<3, 3>() = init_rotation * pitch_l_pt * yaw_l_pt;
        rot_c = q_rot_c_map.lower_bound(point_time)->second.toRotationMatrix();
        v_g = car2gimbal * rot_c * v_w;
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

  // Because the LiDAR is not aligned with the gimbal, 
  // we need to rotate the point cloud to align with the gimbal.
  // Eigen::Matrix4f align_pose = Eigen::Matrix4f::Identity();
  // align_pose.topLeftCorner<3, 3>() = Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX()).toRotationMatrix();
  // pcl::transformPointCloud(*p_cloud_out, *p_cloud_out, align_pose);

  sensor_msgs::PointCloud2 msg_out;
  Eigen::Matrix4f real_pose = Eigen::Matrix4f::Identity();
  real_pose.topLeftCorner<3, 3>() = car2gimbal *  q_rot_c_map.lower_bound(frame_time)->second.toRotationMatrix();
  real_pose.topRightCorner<3, 1>() = trans_c;
  pcl::transformPointCloud(*p_cloud_out, *p_cloud_out, real_pose);
  pcl::toROSMsg(*p_cloud_out, msg_out);
  msg_out.header.frame_id = "map";
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

  imu_sub = nh.subscribe("/livox/imu", 1, &imu_callback);
  cloud_sub = nh.subscribe("/livox/lidar", 1, &pointcloud2_callback);
  // gimbal_sub_h = nh.subscribe("/horizontal_angle", 1, &gimbal_horizontal_callback);
  // gimbal_sub_v = nh.subscribe("/vertical_angle", 1, &gimbal_vertical_callback);
  gimbal_sub_pan = nh.subscribe("pan", 1, &gimbal_pan_callback);
  gimbal_sub_tilt = nh.subscribe("tilt", 1, &gimbal_tilt_callback);
  gnss_sub = nh.subscribe("/Inertial/gps/fix", 1, &gnss_callback);
  car_imu_sub = nh.subscribe("/Inertial/imu/data", 1, &car_imu_callback);

  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/registered_cloud", 1);

  ros::spin();

  return 0;
}

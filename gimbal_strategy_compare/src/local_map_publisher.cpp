// Extracts a local sub-region of a global point-cloud map around each sample
// pose and publishes it in the LiDAR frame.

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>

static double g_voxel = 0.5;      // global downsample resolution (m)
static pcl::PointCloud<pcl::PointXYZ>::Ptr g_global(new pcl::PointCloud<pcl::PointXYZ>);
static pcl::KdTreeFLANN<pcl::PointXYZ> g_kdtree;
static bool g_ready = false;

static ros::Publisher g_pub;
static ros::Time g_last_sample(0.0);

// ---- params ----
static double g_period = 2.0;     // sample interval (s)
static double g_range  = 150.0;   // extraction radius (m)
static Eigen::Vector3d g_lidar_T(1.08, 0.0, 1.643);
static Eigen::Matrix3d g_R_lidar_to_vehicle = Eigen::Matrix3d::Identity();

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  if (!g_ready) return;

  ros::WallTime t0 = ros::WallTime::now();
  ros::Time now = msg->header.stamp;
  if (!g_last_sample.isZero() &&
      (now - g_last_sample).toSec() < g_period - 1e-3)
    return;
  g_last_sample = now;

  const auto& pos  = msg->pose.pose.position;
  const auto& quat = msg->pose.pose.orientation;
  Eigen::Vector3d veh_pos(pos.x, pos.y, pos.z);
  Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
  Eigen::Matrix3d R_vm = q.toRotationMatrix();  // vehicle → map

  // LiDAR origin in map frame
  Eigen::Vector3d origin_map = veh_pos + R_vm * g_lidar_T;
  // map → LiDAR rotation
  Eigen::Matrix3d R_m_lidar = (R_vm * g_R_lidar_to_vehicle).transpose();

  // kd-tree radius search
  pcl::PointXYZ search_pt;
  search_pt.x = static_cast<float>(origin_map.x());
  search_pt.y = static_cast<float>(origin_map.y());
  search_pt.z = static_cast<float>(origin_map.z());

  std::vector<int> indices;
  std::vector<float> dists;
  g_kdtree.radiusSearch(search_pt, static_cast<float>(g_range), indices, dists);

  // transform to LiDAR frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr local(new pcl::PointCloud<pcl::PointXYZ>);
  local->reserve(indices.size());
  for (int idx : indices) {
    const auto& pt = g_global->points[idx];
    Eigen::Vector3d vm(pt.x, pt.y, pt.z);
    Eigen::Vector3d vl = R_m_lidar * (vm - origin_map);
    local->points.emplace_back(
        static_cast<float>(vl.x()),
        static_cast<float>(vl.y()),
        static_cast<float>(vl.z()));
  }

  local->width  = local->points.size();
  local->height = 1;
  local->is_dense = false;

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*local, msg_out);
  msg_out.header.stamp = now;
  msg_out.header.frame_id = "lidar_frame";
  g_pub.publish(msg_out);

  ROS_INFO("[LocalMap] t=%.1f  %zu pts  %.0fms",
           now.toSec(), local->size(), (ros::WallTime::now() - t0).toSec() * 1000.0);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_map_publisher");
  ros::NodeHandle nh("~");

  std::string pcd_path;
  nh.param<std::string>("pcd_path", pcd_path,
                         "/home/loc/loc_ws/data/dianwanghesai/map-01new.ply");
  nh.param<double>("sample_period", g_period, 2.0);
  nh.param<double>("range", g_range, 150.0);

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
    }
  }

  // Load and downsample
  ROS_INFO("Loading map from %s ...", pcd_path.c_str());
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPLYFile(pcd_path, *raw) < 0) {
    ROS_ERROR("Failed to load %s", pcd_path.c_str());
    return 1;
  }
  ROS_INFO("Loaded %zu points, downsampling at %.2fm ...", raw->size(), g_voxel);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(raw);
  vg.setLeafSize(static_cast<float>(g_voxel), static_cast<float>(g_voxel), static_cast<float>(g_voxel));
  vg.filter(*g_global);
  raw.reset();
  ROS_INFO("Downsampled: %zu points. Building kd-tree ...", g_global->size());
  g_kdtree.setInputCloud(g_global);
  g_ready = true;
  ROS_INFO("Ready.");

  ros::NodeHandle nh_global;
  g_pub = nh_global.advertise<sensor_msgs::PointCloud2>("/local_map", 1);
  ros::Subscriber odom_sub =
      nh_global.subscribe("/iekf3d/odometry", 10, odomCallback);

  ROS_INFO("LocalMapPublisher started (period=%.1fs, range=%.0fm).", g_period, g_range);
  ros::spin();
  return 0;
}

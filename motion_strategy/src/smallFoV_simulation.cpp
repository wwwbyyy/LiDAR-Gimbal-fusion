#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

// Global variables for the FoV parameters
double fovMin = -M_PI * 70 / 180.0 / 2; // Minimum angle of the FoV
double fovMax = M_PI* 70 / 180.0 / 2; // Maximum angle of the FoV
double currentOrientation = 0.0; // Current orientation

ros::Publisher smallFovPointCloudPub;

// Callback function for the point cloud message
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // Create a new point cloud message for the small FoV simulation
  sensor_msgs::PointCloud2 smallFovPointCloud;

  // Calculate the indices of the point cloud data within the FoV
  pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr p_smallFovCloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *p_cloud);
  for (const auto& point : p_cloud->points)
  {
    double angle = std::atan2(point.y, point.x);
    double anglediff = std::fmod(angle - currentOrientation +  3 * M_PI, 2 * M_PI) - M_PI; // Normalize the angle to [-pi, pi]
    if (anglediff >= fovMin && anglediff <= fovMax)
    {
      // Add the point to the small FoV point cloud
      p_smallFovCloud->points.push_back(point);
    }
  }

  // Publish the small FoV point cloud message
  pcl::toROSMsg(*p_smallFovCloud, smallFovPointCloud);
  smallFovPointCloud.header = msg->header;
  //smallFovPointCloud.fields = msg->fields;
  smallFovPointCloudPub.publish(smallFovPointCloud);
}

// Callback function for the orientation message
void orientationCallback(const std_msgs::Float64::ConstPtr& msg)
{
  currentOrientation = msg->data * M_PI / 180.0; // Convert degrees to radians
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "small_fov_simulation");
  ros::NodeHandle nh;

  // Subscribe to the full point cloud topic
  ros::Subscriber pointCloudSub = nh.subscribe<sensor_msgs::PointCloud2>("/driver/hesai/pandar", 1, pointCloudCallback);

  // Subscribe to the orientation topic
  ros::Subscriber orientationSub = nh.subscribe<std_msgs::Float64>("/control_yaw_ang", 1, orientationCallback);

  // Publisher for the small FoV point cloud
  smallFovPointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/hesai/simulated_smallfov_cloud", 1);

  ros::spin();

  return 0;
}

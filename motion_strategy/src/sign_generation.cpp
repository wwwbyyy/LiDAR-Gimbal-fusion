#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <map>

#include <Eigen/Dense>
#include <pcl/io/auto_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>

#include "point_type.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr p_map_origin(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr p_map(new pcl::PointCloud<pcl::PointXYZ>);

const double FoV_range_rad = M_PI * 70 / 180.0;

std::vector<TrajectoryPoint> readCSV(const std::string& filePath) {
  std::vector<TrajectoryPoint> trajectory;
  std::ifstream file(filePath);

  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << filePath << std::endl;
    return trajectory;
  }

  std::string line;
  // 跳过第一行（表头）
  std::getline(file, line);
  // 解析表头
  std::map<int, std::string> header;
  std::stringstream headerStream(line);
  std::string headerValue;
  int columnIndex = 0;
  while (std::getline(headerStream, headerValue, ',')) {
    header[columnIndex] = headerValue;
    columnIndex++;
  }
  // 打印表头
  std::cout << "Header: ";
  for (const auto& pair : header) {
    std::cout << pair.second << " ";
  }

  // 读取数据行
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string value;
    TrajectoryPoint point;
    int columnIndex = 0;

    while (std::getline(ss, value, ',')) {
      if (header[columnIndex] == "field.header.stamp") {
        point.timestamp = std::stod(value)/1e9;
      } else if (header[columnIndex] == "field.pose.position.x") {
        point.position.x() = std::stod(value);
      } else if (header[columnIndex] == "field.pose.position.y") {
        point.position.y() = std::stod(value);
      } else if (header[columnIndex] == "field.pose.position.z") {
        point.position.z() = std::stod(value);
      }
      columnIndex++;
    }
    trajectory.push_back(point);
  }

  file.close();
  std::cout << "Read " << trajectory.size() << " trajectory points from " << filePath << std::endl;
  return trajectory;
}

// 根据轨迹生成路标的函数
std::vector<SignPoint> generate_SignPoints(const std::vector<TrajectoryPoint>& trajectory, double interval, double angle_step_ang, double map_range) {
  std::vector<SignPoint> signpoints;

  if (trajectory.empty()) {
    std::cerr << "Trajectory is empty!" << std::endl;
    return signpoints;
  }
  
  // Take the first point of the trajectory
  const TrajectoryPoint& firstPoint = trajectory.front();
  signpoints.push_back({firstPoint.position, 0.0, 0.0});

  // Generate signpoints with the given interval
  for (size_t i = 1; i < trajectory.size(); i++) {
    const TrajectoryPoint& currentPoint = trajectory[i];
    const Eigen::Vector3d& currentPosition = currentPoint.position;

    // Check if there are existing signpoints within the interval range
    bool hasExistingSignpoints = false;
    for (const SignPoint& signpoint : signpoints) {
      const Eigen::Vector3d& signpointPosition = signpoint.position;
      double distance = (currentPosition - signpointPosition).norm();
      if (distance <= interval) {
        hasExistingSignpoints = true;
        break;
      }
    }

    // If no existing signpoints within the interval, generate a new signpoint
    if (!hasExistingSignpoints) {
      signpoints.push_back({currentPosition, 0.0, 0.0});
    }
  }
  std::cout << "Generated " << signpoints.size() << " signpoints." << std::endl;

  // Calculate the best yaw and pitch angles for each signpoint
  for (SignPoint& signpoint : signpoints) {
    const Eigen::Vector3d& signpointPosition = signpoint.position;

    // Define the range and step size for angles
    double angle_step_rad = angle_step_ang * M_PI / 180.0; // Convert to radians

    // Initialize variables for best yaw and maximum point count
    double bestYaw = 0.0;
    int maxPointCount = 0;

    // Iterate over the range of angles
    for (double yaw = 0; yaw <= M_PI * 2; yaw += angle_step_rad) {
      double pitch = 0.0; // Initial pitch

      // Calculate the number of points within the FoV range
      int pointCount = 0;
      for (const pcl::PointXYZ& point : p_map->points) {
        // Convert the point to Eigen::Vector3d
        Eigen::Vector3d pointPosition(point.x, point.y, point.z);
        if ((pointPosition - signpointPosition).norm() > map_range) {
          continue; // Skip points outside the specified range
        }

        // Calculate the angle between the signpoint and the current point
        double angle = std::atan2(pointPosition.y() - signpointPosition.y(), pointPosition.x() - signpointPosition.x());

        // Calculate the cyclic difference between the angles
        double angleDiff = std::fmod(yaw - angle + 3 * M_PI, 2 * M_PI) - M_PI;

        // Check if the point is within the FoV range
        if (std::abs(angleDiff) <= FoV_range_rad / 2.0) {
          pointCount++;
        }
      }
      std::cout << "Yaw: " << yaw * 180.0 / M_PI << ", Point Count: " << pointCount << std::endl;

      // Update the best yaw and maximum point count if necessary
      if (pointCount > maxPointCount) {
        bestYaw = yaw;
        maxPointCount = pointCount;
      }
    }

    // Update the signpoint's best_yaw and best_pitch
    signpoint.best_yaw_ang = bestYaw * 180.0 / M_PI; // Convert to degrees
    signpoint.best_pitch_ang = 0.0;
    
    std::cout << "Signpoint position: " << signpointPosition.transpose() << ", Best Yaw: " << signpoint.best_yaw_ang
              << ", Best Pitch: " << signpoint.best_pitch_ang << std::endl;
  }
  
  return signpoints;
}

// 主函数
int main(int argc, char** argv) {
  // 设置 CSV 文件路径
  std::string csv_filepath = "data/sjtu-200-traj.csv";
  std::string map_filepath = "data/SJTU1031/map1031-10cm.ply";

  // 读取轨迹数据
  std::vector<TrajectoryPoint> trajectory = readCSV(csv_filepath);
  // 读入点云数据
  if (pcl::io::loadPLYFile<pcl::PointXYZ>(map_filepath, *p_map_origin) == -1) {
    PCL_ERROR("Failed to read PCD file\n");
    return -1;
  }
  std::cout << "Loaded map with " << p_map_origin->points.size() << " points." << std::endl;
  
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(p_map_origin);
  sor.setLeafSize(0.3, 0.3, 0.3); // Set the desired leaf size for downsampling
  sor.filter(*p_map);
  std::cout << "Downsampled map with " << p_map->points.size() << " points." << std::endl;

  if (trajectory.empty()) {
    std::cerr << "No trajectory data found!" << std::endl;
    return -1;
  }

  double signpointInterval = 5.0; // 距离间隔
  std::vector<SignPoint> signpoints = generate_SignPoints(trajectory, signpointInterval, 30.0, 20.0);

  std::ofstream outfile("data/signpoints.csv");
  if (outfile.is_open()) {
    outfile << "x,y,z,best_yaw_ang,best_pitch_ang" << std::endl;
    for (const auto& signpoint : signpoints) {
      outfile << signpoint.position.x() << "," << signpoint.position.y() << ","
              << signpoint.position.z() << "," << signpoint.best_yaw_ang << ","
              << signpoint.best_pitch_ang << std::endl;
    }
    outfile.close();
    std::cout << "Signpoints saved to signpoints.csv" << std::endl;
  } else {
    std::cerr << "Failed to open signpoints.csv for writing" << std::endl;
  }

  return 0;
}
#include <fstream>
#include <iostream>
#include <cmath>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <cyber_msgs/LocalizationEstimate.h>
#include <std_msgs/Float64.h>

#include "point_type.h"
#include "GimbalCommand.h"

enum Stategy{
  NONE,
  UNIFORM,
  MOST_COUNT
};

std::vector<SignPoint> signpoints;

ros::Publisher control_yaw_pub;
ros::Publisher control_msg_pub;

//6 elements: Translation(m) + Euler[x,y,z]
std::vector<double> lidar_extinct = {1.15, 0, 1.96, 0, 0, 1.5315};

Stategy motion_strategy = UNIFORM;

void none_motion_strategy(const cyber_msgs::LocalizationEstimate::ConstPtr& msg) {
  // No motion strategy applied
  std_msgs::Float64 yawMsg;
  yawMsg.data = 0.0; // Default value
  control_yaw_pub.publish(yawMsg);
  ROS_INFO("No motion strategy applied, yaw: %f", yawMsg.data);
}

void uniform_motion_strategy(const cyber_msgs::LocalizationEstimate::ConstPtr& msg) {
  // Apply uniform motion strategy
  static double last_yaw = 0.0;
  static ros::Time last_time = ros::Time::now();
  std_msgs::Float64 yawMsg;
  yawMsg.data = last_yaw + 30 * (ros::Time::now() - last_time).toSec(); // Increment yaw by 150 degrees per second
  if (yawMsg.data > 180) {
    yawMsg.data -= 360; // Wrap around
  } else if (yawMsg.data < -180) {
    yawMsg.data += 360; // Wrap around
  }
  last_yaw = yawMsg.data;
  control_yaw_pub.publish(yawMsg);
  last_time = ros::Time::now();
  ROS_INFO("Uniform motion strategy applied, yaw: %f", yawMsg.data);
}

void most_count_motion_strategy(const cyber_msgs::LocalizationEstimate::ConstPtr& msg) {
  // Apply most count motion strategy
  double minDistance = std::numeric_limits<double>::max();
  SignPoint nearestSignpoint1, nearestSignpoint2;
  Eigen::Vector3d localizationPosition(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  for (const auto& signpoint : signpoints) {
    double distance = (localizationPosition - signpoint.position).norm();
    if (distance < minDistance) {
      minDistance = distance;
      nearestSignpoint2 = nearestSignpoint1;
      nearestSignpoint1 = signpoint;
    }
  }

  // Calculate the weighted best yaw
  double smallerDistance = (nearestSignpoint1.position - localizationPosition).norm();
  double largerDistance = (nearestSignpoint2.position - localizationPosition).norm();
  double distanceRatio = smallerDistance / largerDistance; // Calculate the distance ratio
  double weight = 1/2-(distanceRatio*distanceRatio-1)/((distanceRatio*distanceRatio+1)*2); 

  double angleDiff = std::fmod(nearestSignpoint1.best_yaw_ang - nearestSignpoint2.best_yaw_ang + 3 * 180, 2 * 180) - 180;
  double weightedYaw = angleDiff * weight + nearestSignpoint2.best_yaw_ang;

  Eigen::Quaternion q_rot_car2map(
    msg->pose.orientation.w, 
    msg->pose.orientation.x, 
    msg->pose.orientation.y, 
    msg->pose.orientation.z);
  Eigen::Matrix3d rot_car2map = q_rot_car2map.toRotationMatrix();
  Eigen::Matrix3d rot_lidar2car = Eigen::AngleAxisd(lidar_extinct[3], Eigen::Vector3d::UnitX()).toRotationMatrix() *
    Eigen::AngleAxisd(lidar_extinct[4], Eigen::Vector3d::UnitY()).toRotationMatrix() *
    Eigen::AngleAxisd(lidar_extinct[5], Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Matrix3d rot_map2lidar = (rot_car2map * rot_lidar2car).transpose();
  Eigen::Vector3d w_yaw_vec(std::cos(weightedYaw * M_PI / 180.0), std::sin(weightedYaw * M_PI / 180.0), 0);
  Eigen::Vector3d w_yaw_vec_car = rot_map2lidar * w_yaw_vec;
  double vehicleYaw = std::atan2(w_yaw_vec_car.y(), w_yaw_vec_car.x());

  // Publish the yaw in degrees
  static double last_yaw = 0.0;
  std_msgs::Float64 yawMsg;
  yawMsg.data = vehicleYaw * 180.0 / M_PI;
  double yawDiff = std::fmod(yawMsg.data - last_yaw + 3 * 180, 2 * 180) - 180;
  if (yawDiff > 10) {
    yawMsg.data = last_yaw + 10; // Increment by 10 degrees
  } else if (yawDiff < -10) {
    yawMsg.data = last_yaw - 10; // Decrement by 10 degrees
  }
  if (yawMsg.data > 180) {
    yawMsg.data -= 360; // Wrap around
  } else if (yawMsg.data < -180) {
    yawMsg.data += 360; // Wrap around
  }
  last_yaw = yawMsg.data;
  control_yaw_pub.publish(yawMsg);

  cyber_msgs::GimbalCommand cmdMsg;
  cmdMsg.header.stamp = ros::Time::now();
  cmdMsg.cmd = 0x4B;
  cmdMsg.data = yawMsg.data;
  control_msg_pub.publish(cmdMsg);

  ROS_INFO("Most count motion strategy applied, yaw: %f", yawMsg.data);
}

void estimate_callback(const cyber_msgs::LocalizationEstimate::ConstPtr& msg) {
  // Process the localization estimate message
  // Find the two nearest signpoints based on position

  void (*motion_strategy_func)(const cyber_msgs::LocalizationEstimate::ConstPtr& msg) = nullptr;

  switch(motion_strategy) {
    case NONE:
      motion_strategy_func = &none_motion_strategy;
      break;
    case UNIFORM:
      motion_strategy_func = &uniform_motion_strategy;
      break;
    case MOST_COUNT:
      motion_strategy_func = &most_count_motion_strategy;
      break;
    default:
      ROS_ERROR("Invalid motion strategy");
      return;
  }

  motion_strategy_func(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_strategy");
  ros::NodeHandle nh;

  // Load landmarks from the signpoints.csv file
  std::ifstream file("data/signpoints.csv");
  if (file.is_open()) {
    std::string line;
    std::getline(file, line); // Skip header
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      SignPoint signpoint;
      char delimiter = ',';
      std::string x, y, z, yaw, pitch;
      if (std::getline(iss, x, delimiter) && std::getline(iss, y, delimiter) && std::getline(iss, z, delimiter) &&
          std::getline(iss, yaw, delimiter) && std::getline(iss, pitch, delimiter)) {
        signpoint.position.x() = std::stod(x);
        signpoint.position.y() = std::stod(y);
        signpoint.position.z() = std::stod(z);
        signpoint.best_yaw_ang = std::stod(yaw);
        signpoint.best_pitch_ang = std::stod(pitch);
        signpoints.push_back(signpoint);
      } else {
        std::cerr << "Error reading line: " << line << std::endl;
      }
    }
    file.close();
  } else {
    ROS_ERROR("Failed to open signpoints.csv file");
    return 1;
  }

  // Create a subscriber for the /localization/estimation topic
  ros::Subscriber sub = nh.subscribe<cyber_msgs::LocalizationEstimate>("/localization/estimation", 10, 
    estimate_callback);

  control_yaw_pub = nh.advertise<std_msgs::Float64>("/control_yaw_ang", 10);
  control_msg_pub = nh.advertise<cyber_msgs::GimbalCommand>("/gimbal_cmd", 10);

  ros::spin();

  return 0;
}

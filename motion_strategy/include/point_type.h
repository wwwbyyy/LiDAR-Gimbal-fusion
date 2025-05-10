#pragma once

#include <Eigen/Dense>

struct TrajectoryPoint {
  double timestamp;
  Eigen::Vector3d position; 
};

struct SignPoint{
  Eigen::Vector3d position;
  double best_yaw_ang;
  double best_pitch_ang;
};
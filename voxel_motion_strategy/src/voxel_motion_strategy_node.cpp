#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxel_motion_strategy_node");
  ros::NodeHandle nh;

  ROS_INFO("voxel_motion_strategy_node started (dummy).");

  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

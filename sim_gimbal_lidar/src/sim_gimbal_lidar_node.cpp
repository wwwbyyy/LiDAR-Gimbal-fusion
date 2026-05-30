#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <cyber_msgs/GimbalCommand.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include "ros/package.h"

#include "reflcpp/core.h"
#include "reflcpp/yaml.h"

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;

ros::Subscriber gimbal_cmd_sub;
ros::Subscriber cloud_sub;

ros::Publisher pan_pub;         // /pan — same interface as pelco_control
ros::Publisher tilt_pub;        // /tilt
ros::Publisher cloud_pub;

struct ProgramConfigs{
  std::string lidar_topic;
  std::string gimbal_cmd_topic;
} cfg;
REFLCPP_METAINFO(ProgramConfigs, ,(lidar_topic)(gimbal_cmd_topic));

class SimGimbalLidar{
  private:
    float yaw_deg;
    float pitch_deg;
    float yaw_fov_deg = 70.4f;

  public:
    float get_yaw_deg() { return yaw_deg; }
    float get_pitch_deg() { return pitch_deg; }
    void set_yaw_deg(float v) {
      yaw_deg = v;
      while (yaw_deg > 180.0f) yaw_deg -= 360.0f;
      while (yaw_deg < -180.0f) yaw_deg += 360.0f;
    }
    void set_pitch_deg(float v) { pitch_deg = v; }

    SimGimbalLidar(float init_yaw_deg, float init_pitch_deg):
        yaw_deg(init_yaw_deg), pitch_deg(init_pitch_deg) {}

    CloudType::Ptr get_pointcloud(CloudType::Ptr p_cloud_in)
    {
        CloudType::Ptr p_cloud_out(new CloudType);
        p_cloud_out->reserve(p_cloud_in->size());
        float half_fov = yaw_fov_deg / 2.0f;
        for (const auto& pt : p_cloud_in->points)
        {
            if (!pcl::isFinite(pt)) continue;
            float yaw = std::atan2(pt.y, pt.x) * 180.0f / M_PI;
            float diff = yaw - yaw_deg;
            while (diff > 180.0f) diff -= 360.0f;
            while (diff < -180.0f) diff += 360.0f;
            if (std::abs(diff) <= half_fov)
            {
                p_cloud_out->points.push_back(pt);
            }
        }
        p_cloud_out->width = p_cloud_out->points.size();
        p_cloud_out->height = 1;
        p_cloud_out->is_dense = false;
        return p_cloud_out;
    }
}
sim_gimbal_lidar(0.0, 0.0);

void gimbalCmdCallback(const cyber_msgs::GimbalCommand::ConstPtr& msg)
{
  // PAN  = 0x4B: set absolute pan angle (degrees)
  // TILT = 0x4D: set absolute tilt angle (degrees)
  if (msg->cmd == 0x4B) {
    sim_gimbal_lidar.set_yaw_deg(static_cast<float>(msg->data));
  } else if (msg->cmd == 0x4D) {
    sim_gimbal_lidar.set_pitch_deg(static_cast<float>(msg->data));
  }
}

void pointcloud2_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  CloudType::Ptr p_cloud_in(new CloudType);
  pcl::fromROSMsg(*msg, *p_cloud_in);

  CloudType::Ptr p_cloud_out = sim_gimbal_lidar.get_pointcloud(p_cloud_in);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*p_cloud_out, msg_out);
  msg_out.header = msg->header;
  cloud_pub.publish(msg_out);

  // Publish feedback — same format as pelco_control
  std_msgs::Float64MultiArray pan_msg;
  pan_msg.data.resize(2);
  pan_msg.data[0] = msg->header.stamp.toSec();
  pan_msg.data[1] = sim_gimbal_lidar.get_yaw_deg();
  pan_pub.publish(pan_msg);

  std_msgs::Float64MultiArray tilt_msg;
  tilt_msg.data.resize(2);
  tilt_msg.data[0] = msg->header.stamp.toSec();
  tilt_msg.data[1] = sim_gimbal_lidar.get_pitch_deg();
  tilt_pub.publish(tilt_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_gimbal_lidar_node");
  ros::NodeHandle nh;

  std::string strSrcFolder = ros::package::getPath("sim_gimbal_lidar");
  cfg = YAML::LoadFile(strSrcFolder + "/configs/configs.yaml")
          .as<ProgramConfigs>();

  std::cout << "lidar_topic: " << cfg.lidar_topic << std::endl
            << "gimbal_cmd_topic: " << cfg.gimbal_cmd_topic << std::endl;

  gimbal_cmd_sub = nh.subscribe<cyber_msgs::GimbalCommand>(
      cfg.gimbal_cmd_topic, 10, gimbalCmdCallback);
  cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      cfg.lidar_topic, 10, pointcloud2_callback);

  pan_pub   = nh.advertise<std_msgs::Float64MultiArray>("/pan", 10);
  tilt_pub  = nh.advertise<std_msgs::Float64MultiArray>("/tilt", 10);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(
      "/sim_gimbal_lidar/pointcloud", 10);

  ros::spin();
  return 0;
}

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include "ros/package.h"

#include "reflcpp/core.h"
#include "reflcpp/yaml.h"

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;

ros::Subscriber yaw_deg_sub;
ros::Subscriber yaw_deg_angular_velocity_sub;
ros::Subscriber cloud_sub;

ros::Publisher yaw_deg_pub;
ros::Publisher cloud_pub;

struct ProgramConfigs{
  std::string lidar_topic;
  std::string yaw_deg_topic;
  std::string yaw_deg_angular_velocity_topic;
  // float dt; // in second
} cfg;
REFLCPP_METAINFO(ProgramConfigs, 
,(lidar_topic)(yaw_deg_topic)(yaw_deg_angular_velocity_topic));

class SimGimbalLidar{
  private:
    float yaw_deg; // in degree
    float yaw_deg_angular_velocity; // in deg/s
    float pitch_deg; // in degree
    float pitch_deg_angular_velocity; // in deg/s

    float yaw_fov_deg = 70.4f; // in degree

  public:
    float get_yaw_deg() { return yaw_deg; }
    float get_pitch_deg() { return pitch_deg; }
    void set_yaw_deg(float v) { yaw_deg = v; }
    void set_pitch_deg(float v) { pitch_deg = v; }
    void set_yaw_deg_angular_velocity(float v) { yaw_deg_angular_velocity = v; }
    void set_pitch_deg_angular_velocity(float v) { pitch_deg_angular_velocity = v; }
    void update_yaw_deg(float dt)
    {
        yaw_deg += yaw_deg_angular_velocity * dt;
        while (yaw_deg > 180.0) yaw_deg -= 360.0;
        while (yaw_deg < -180.0) yaw_deg += 360.0;
    }
    void update_pitch_deg(float dt)
    {
        pitch_deg += pitch_deg_angular_velocity * dt;
        if (pitch_deg > 90.0) pitch_deg = 90.0;
        if (pitch_deg < -90.0) pitch_deg = -90.0;
    }
    void set_yaw_fov_deg(float v) { yaw_fov_deg = v; }
    float get_yaw_fov_deg() { return yaw_fov_deg; }

    SimGimbalLidar(float init_yaw_deg, float init_pitch_deg):
        yaw_deg(init_yaw_deg), pitch_deg(init_pitch_deg),
        yaw_deg_angular_velocity(0.0), pitch_deg_angular_velocity(0.0)
    {}

    CloudType::Ptr get_pointcloud(CloudType::Ptr p_cloud_in)
    {
        CloudType::Ptr p_cloud_out(new CloudType);
        p_cloud_out->reserve(p_cloud_in->size());
        float half_fov = yaw_fov_deg / 2.0f;
        for (const auto& pt : p_cloud_in->points)
        {
            if (!pcl::isFinite(pt)) continue;
            float yaw = std::atan2(pt.y, pt.x) * 180.0f / M_PI; // in degree
            //float pitch = std::atan2(pt.z, std::sqrt(pt.x * pt.x + pt.y * pt.y)) * 180.0f / M_PI; // in degree
            if (yaw >= yaw_deg - half_fov && yaw <= yaw_deg + half_fov)
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

void yaw_deg_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  sim_gimbal_lidar.set_yaw_deg(msg->data[1]);
}

void yaw_deg_angular_velocity_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  sim_gimbal_lidar.set_yaw_deg_angular_velocity(msg->data[1]);
}

void pointcloud2_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  static double last_time = msg->header.stamp.toSec();
  double current_time = msg->header.stamp.toSec();
  double dt = current_time - last_time;
  last_time = current_time;

  sim_gimbal_lidar.update_yaw_deg(dt);
  //sim_gimbal_lidar.update_pitch_deg(dt);

  CloudType::Ptr p_cloud_in(new CloudType);
  pcl::fromROSMsg(*msg, *p_cloud_in);

  CloudType::Ptr p_cloud_out = sim_gimbal_lidar.get_pointcloud(p_cloud_in);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*p_cloud_out, msg_out);
  msg_out.header = msg->header;
  cloud_pub.publish(msg_out);

  std_msgs::Float64MultiArray yaw_msg;
  yaw_msg.data.resize(2);
  yaw_msg.data[0] = msg->header.stamp.toSec();
  yaw_msg.data[1] = sim_gimbal_lidar.get_yaw_deg();
  yaw_deg_pub.publish(yaw_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_gimbal_lidar_node");
  ros::NodeHandle nh;

  std::string strSrcFolder = ros::package::getPath("sim_gimbal_lidar");
  cfg = YAML::LoadFile(strSrcFolder + "/configs/configs.yaml")
          .as<ProgramConfigs>();
  
  std::cout << "lidar_topic: " << cfg.lidar_topic << std::endl
            << "yaw_deg_topic: " << cfg.yaw_deg_topic << std::endl
            << "yaw_deg_angular_velocity_topic: " << cfg.yaw_deg_angular_velocity_topic << std::endl;
  
  yaw_deg_sub = nh.subscribe<std_msgs::Float64MultiArray>(cfg.yaw_deg_topic, 10, yaw_deg_callback);
  yaw_deg_angular_velocity_sub = nh.subscribe<std_msgs::Float64MultiArray>(cfg.yaw_deg_angular_velocity_topic, 10, yaw_deg_angular_velocity_callback);
  cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(cfg.lidar_topic, 10, pointcloud2_callback);

  yaw_deg_pub = nh.advertise<std_msgs::Float64MultiArray>("sim_gimbal_lidar/yaw_deg", 10);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("sim_gimbal_lidar/pointcloud", 10);

  ros::spin();

  return 0;
}
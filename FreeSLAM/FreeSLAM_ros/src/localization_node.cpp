// clang-format off
#include <filesystem>
#include <string>
#include <chrono>
// clang-format on

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gps_common/conversions.h>
#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_broadcaster.h>

#include "FreeSLAM/Localization.hpp"
#include "reflcpp/core.hpp"
#include "reflcpp/yaml.hpp"
#include "tools/rosbag.hpp"

namespace fs = std::filesystem;

/**
 * @brief program configs
 *
 */
struct ProgramConfigs {
    std::string bag_file;  // path to rosbag file. leave empty for online appliction.
    float bag_rate;        // rosbag play rate

    int gps_status_valid;

    std::string lidar_type;
    std::string lidar_topic;
    std::string gps_topic;
    std::string imu_topic;
} prog_cfg;
REFLCPP_METAINFO(
    ProgramConfigs, ,
    (bag_file)(bag_rate)(gps_status_valid)(lidar_type)(lidar_topic)(gps_topic)(imu_topic))
REFLCPP_YAML(ProgramConfigs)

/**
 * @brief relative path to absolute path.
 *
 * @param path
 * @param prefix
 */
void as_absolute_path(std::string &path, const std::string &prefix) {
    if (!path.empty() && fs::path(path).is_relative()) {
        path = fs::path(prefix) / path;
    }
}

/**
 * @brief user appliction
 *
 */
std::unique_ptr<FreeSLAM::Localization> app;

/**
 * @brief ros subscriber and publisher
 *
 */
ros::Subscriber lidar_sub;
ros::Subscriber gps_sub;
ros::Subscriber imu_sub;
ros::Subscriber ori_sub;

ros::Publisher registered_pub;
ros::Publisher global_map_pub;
ros::Publisher odom_pub;

ros::Publisher slam_utm_pub;
ros::Publisher slam_heading_pub;

// utm zone
std::string utm_zone;

/**
 * @brief ros sensor callback function
 *
 */
void lidar_process(std_msgs::Header header, FreeSLAM::PointVec frame) {
    using namespace std::chrono;

    auto t1 = steady_clock::now();
    app->FeedPointCloud(header.stamp.toSec(), std::move(frame));
    auto t2 = steady_clock::now();
    std::cout << "total time consumption: " << duration_cast<milliseconds>(t2 - t1).count() << "ms"
              << std::endl;

    auto pose = app->GetCurrentPose();
    Eigen::Quaternionf q(pose.topLeftCorner<3, 3>());

    if (registered_pub.getNumSubscribers() > 0) {
        sensor_msgs::PointCloud2 registered_msg;
        app->GetRegisteredPoints(frame);
        pcl::toROSMsg(frame, registered_msg);
        registered_msg.header.seq = header.seq;
        registered_msg.header.stamp = header.stamp;
        registered_msg.header.frame_id = "map";
        registered_pub.publish(registered_msg);
    }

    if (odom_pub.getNumSubscribers() > 0) {
        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = "map";
        odom_msg.header.stamp = header.stamp;
        odom_msg.child_frame_id = header.frame_id;
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.position.x = pose(0, 3);
        odom_msg.pose.pose.position.y = pose(1, 3);
        odom_msg.pose.pose.position.z = pose(2, 3);
        odom_pub.publish(odom_msg);
    }

    Eigen::Vector3d utm_origin = app->GetUtmOrigin();

    if(app->waiting_position || app->waiting_orientation) return;

    sensor_msgs::NavSatFix slam_utm_msg;
    gps_common::UTMtoLL(pose(1, 3) + utm_origin[1], pose(0, 3) + utm_origin[0], utm_zone,
                        slam_utm_msg.latitude, slam_utm_msg.longitude);
    slam_utm_msg.header.stamp = header.stamp;
    slam_utm_pub.publish(slam_utm_msg);

    sensor_msgs::Imu slam_heading_msg;
    slam_heading_msg.orientation.x = q.coeffs().x();
    slam_heading_msg.orientation.y = q.coeffs().y();
    slam_heading_msg.orientation.z = q.coeffs().z();
    slam_heading_msg.orientation.w = q.coeffs().w();
    slam_heading_msg.header.stamp = header.stamp;
    slam_heading_pub.publish(slam_heading_msg);
}

void pointcloud2_callback(sensor_msgs::PointCloud2Ptr p_msg) {
    // convert msg
    FreeSLAM::PointVec frame;
    pcl::fromROSMsg(*p_msg, frame);
    // find possible timestamp field
    std::string stamp_name;
    for (auto f : p_msg->fields) {
        if (f.name == "t" || f.name == "time" || f.name == "timestamp") {
            stamp_name = f.name;
            ROS_INFO_ONCE("assume '%s' as point timestamp field", stamp_name.c_str());
            if (f.datatype != sensor_msgs::PointField::FLOAT64) {
                ROS_WARN_ONCE(
                    "point timestamp field '%s' datatype is not float64. "
                    "timestamp will be ignored!",
                    stamp_name.c_str());
            }
            break;
        }
    }
    // set timestamp
    if (!stamp_name.empty()) {
        int points_num = p_msg->width * p_msg->height;
        std::vector<double> stamps(points_num);
        double stamp_min = std::numeric_limits<double>::infinity();
        sensor_msgs::PointCloud2ConstIterator<double> stamp_it(*p_msg, stamp_name);
        for (int i = 0; i < points_num; i++, ++stamp_it) {
            stamps[i] = *stamp_it;
            if (stamps[i] < stamp_min) stamp_min = stamps[i];
        }
        for (int i = 0; i < points_num; i++) {
            frame[i].timestamp = stamps[i] - stamp_min + p_msg->header.stamp.toSec();
        }
    }
    // fix data[3]
    for (auto &pt : frame) pt.data[3] = 1;
    // process data
    lidar_process(p_msg->header, std::move(frame));
}

void livox_callback(livox_ros_driver::CustomMsgPtr p_msg) {
    // convert msg
    FreeSLAM::PointVec frame;
    frame.resize(p_msg->point_num);
    for (int i = 0; i < (int)p_msg->point_num; i++) {
        frame[i].timestamp = (p_msg->timebase + p_msg->points[i].offset_time) / 1e9;
        frame[i].x = p_msg->points[i].x;
        frame[i].y = p_msg->points[i].y;
        frame[i].z = p_msg->points[i].z;
    }
    // process data
    lidar_process(p_msg->header, std::move(frame));
}

void gps_callback(sensor_msgs::NavSatFixPtr p_msg) {
    if (p_msg->status.status != prog_cfg.gps_status_valid) return;
    Eigen::Vector3d utm;
    gps_common::LLtoUTM(p_msg->latitude, p_msg->longitude, utm.y(), utm.x(), utm_zone);
    utm.z() = p_msg->altitude;
    app->FeedGPS(p_msg->header.stamp.toSec(), utm);
    // std::cout << utm_zone << std::endl;
}

void imu_callback(sensor_msgs::ImuPtr p_msg) {
    Eigen::Quaternionf q(p_msg->orientation.w, p_msg->orientation.x, p_msg->orientation.y,
                         p_msg->orientation.z);
    app->FeedOrientation(p_msg->header.stamp.toSec(), q);
}

void pose_callback(geometry_msgs::PoseWithCovarianceStampedPtr p_msg) {
    Eigen::Quaternionf q(p_msg->pose.pose.orientation.w, p_msg->pose.pose.orientation.x, p_msg->pose.pose.orientation.y,
                         p_msg->pose.pose.orientation.z);
    Eigen::Vector3f p(p_msg->pose.pose.position.x, p_msg->pose.pose.position.y, p_msg->pose.pose.position.z);
    app->FeedPose(p_msg->header.stamp.toSec(), p, q);
}

/**
 * @brief main function
 *
 */
int main(int argc, char *argv[]) {
    /****************************************/
    /*      configure from command line     */
    /****************************************/
    if (argc != 2) {
        ROS_FATAL_STREAM("usage: " << argv[0] << " CONFIG_FILE.");
        return -1;
    }
    ROS_INFO_STREAM("configuring from " << argv[1]);
    YAML::Node cfg = YAML::LoadFile(argv[1]);
    prog_cfg = cfg["prog_cfg"].as<ProgramConfigs>();
    auto prefix_path = fs::path(argv[1]).parent_path();
    as_absolute_path(prog_cfg.bag_file, prefix_path);

    /****************************************/
    /*            configure app             */
    /****************************************/
    auto app_cfg = cfg["app_cfg"].as<FreeSLAM::LocalizationConfigs>();
    as_absolute_path(app_cfg.map_info, prefix_path);
    app = std::make_unique<FreeSLAM::Localization>(app_cfg);

    utm_zone = app->zone;
    std::cout << utm_zone << std::endl;

    /****************************************/
    /*            configure ros             */
    /****************************************/
    ros::init(argc, argv, "FreeSLAM_localization_node");
    ros::NodeHandle nh;
    // subscriber here
    if (prog_cfg.lidar_type == "pointcloud2") {
        lidar_sub = nh.subscribe(prog_cfg.lidar_topic, 1, &pointcloud2_callback,
                                 ros::TransportHints().tcpNoDelay());
    } else if (prog_cfg.lidar_type == "livox") {
        lidar_sub = nh.subscribe(prog_cfg.lidar_topic, 1, &livox_callback,
                                 ros::TransportHints().tcpNoDelay());
    }
    gps_sub = nh.subscribe(prog_cfg.gps_topic, 10, &gps_callback);
    imu_sub = nh.subscribe(prog_cfg.imu_topic, 10, &imu_callback);
    ori_sub = nh.subscribe("/initialpose", 10, &pose_callback);
    // publisher here
    registered_pub = nh.advertise<sensor_msgs::PointCloud2>("registered", 1);
    global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("global_map", 1, true);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);

    slam_utm_pub = nh.advertise<sensor_msgs::NavSatFix>("/slam/gps/fix", 1);
    slam_heading_pub = nh.advertise<sensor_msgs::Imu>("/slam/gps/heading", 1);

    // publish global map
    {
        FreeSLAM::PointVec global_map;
        sensor_msgs::PointCloud2 global_map_msg;
        app->GetGlobalMapPoints(global_map);
        pcl::toROSMsg(global_map, global_map_msg);
        global_map_msg.header.frame_id = "map";
        global_map_pub.publish(global_map_msg);
    }

    /****************************************/
    /*          configure rosbag            */
    /****************************************/
    BagPlayer bag_player(nh);
    if (!prog_cfg.bag_file.empty()) {
        bag_player.open(prog_cfg.bag_file, {prog_cfg.lidar_topic, prog_cfg.gps_topic});
        bag_player.set_queue_size(10);
        bag_player.set_rate(prog_cfg.bag_rate);  // set to 0 for full speed (not recommend)
    }

    /****************************************/
    /*              main loop               */
    /****************************************/
    ROS_INFO_STREAM("begin loop.");
    auto callbacks = ros::getGlobalCallbackQueue();
    while (ros::ok()) {
        // play rosbag
        if (bag_player.is_open()) {
            // end loop if bag end
            if (bag_player.eof()) ros::shutdown();
            bag_player.play_once();
        }
        // process ros message callback (sleep forever if no callbacks available)
        if (bag_player.is_open()) {
            callbacks->callAvailable();
        } else {
            callbacks->callAvailable(ros::WallDuration(999));
        }
    }

    /****************************************/
    /*    (optional): some post process     */
    /****************************************/

    return 0;
}
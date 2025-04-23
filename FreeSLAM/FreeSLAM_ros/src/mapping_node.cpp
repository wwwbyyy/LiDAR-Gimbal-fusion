// clang-format off
#include <filesystem>
#include <string>
#include <chrono>
// clang-format on

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <gps_common/conversions.h>
#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_broadcaster.h>

#include "FreeSLAM/Mapping.hpp"
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
    int queue_size;        // queue size for pub & sub

    int gps_status_valid;

    std::string lidar_type;
    std::string lidar_topic;
    std::string gps_topic;
    std::string imu_topic;
} prog_cfg;
REFLCPP_METAINFO(
    ProgramConfigs, ,
    (bag_file)(bag_rate)(queue_size)(gps_status_valid)(lidar_type)(lidar_topic)(gps_topic)(imu_topic))
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
std::unique_ptr<FreeSLAM::Mapping> app;

/**
 * @brief ros subscriber and publisher
 *
 */
ros::Subscriber lidar_sub;
ros::Subscriber gps_sub;
ros::Subscriber imu_sub;

ros::Publisher registered_pub;
ros::Publisher global_map_pub;
ros::Publisher loop_pub;
ros::Publisher odom_pub;
ros::Publisher trajectory_pub;

ros::Publisher history_frame_pub;

std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub;

/**
 * @brief ros sensor callback function
 *
 */
void lidar_process(std_msgs::Header header, FreeSLAM::PointVec frame) {
    using namespace std::chrono;

    auto t1 = steady_clock::now();
    app->FeedPointCloud(header.stamp.toSec(), std::move(frame));
    auto pose = app->GetCurrentPose();
    Eigen::Quaternionf q(pose.topLeftCorner<3, 3>());
    auto t2 = steady_clock::now();

    if (registered_pub.getNumSubscribers() > 0) {
        sensor_msgs::PointCloud2 registered_msg;
        app->GetRegisteredPoints(frame);
        for (auto &pt : frame) {
            pt.getVector4fMap() = pose * pt.getVector4fMap();
        }
        pcl::toROSMsg(frame, registered_msg);
        registered_msg.header.seq = header.seq;
        registered_msg.header.stamp = header.stamp;
        registered_msg.header.frame_id = "map";
        registered_pub.publish(registered_msg);
    }

    auto t3 = steady_clock::now();
    if (global_map_pub.getNumSubscribers() > 0) {
        sensor_msgs::PointCloud2 local_map_msg;
        app->GetLocalMapPoints(frame);
        pcl::toROSMsg(frame, local_map_msg);
        local_map_msg.header.seq = header.seq;
        local_map_msg.header.stamp = header.stamp;
        local_map_msg.header.frame_id = "map";
        global_map_pub.publish(local_map_msg);
    }
    auto t4 = steady_clock::now();

    if (loop_pub.getNumSubscribers() > 0) {
        app->GetLoopPoints(frame);
        if (!frame.empty()) {
            sensor_msgs::PointCloud2 loop_msg;
            pcl::toROSMsg(frame, loop_msg);
            loop_msg.header.seq = header.seq;
            loop_msg.header.stamp = header.stamp;
            loop_msg.header.frame_id = "map";
            loop_pub.publish(loop_msg);
        }
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

    if (trajectory_pub.getNumSubscribers() > 0) {
        std::map<double, Eigen::Matrix4f> trajectory;
        app->GetTrajectory(trajectory);
        geometry_msgs::PoseArray trajectory_msg;
        trajectory_msg.header.frame_id = "map";
        for (const auto &[t, pose] : trajectory) {
            Eigen::Quaternionf q(pose.topLeftCorner<3, 3>());
            geometry_msgs::Pose pose_msg;
            pose_msg.position.x = pose(0, 3);
            pose_msg.position.y = pose(1, 3);
            pose_msg.position.z = pose(2, 3);
            pose_msg.orientation.w = q.w();
            pose_msg.orientation.x = q.x();
            pose_msg.orientation.y = q.y();
            pose_msg.orientation.z = q.z();
            trajectory_msg.poses.emplace_back(pose_msg);
        }
        trajectory_pub.publish(trajectory_msg);
    }

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.frame_id = "map";
    tf_msg.header.stamp = header.stamp;
    tf_msg.child_frame_id = header.frame_id;
    tf_msg.transform.rotation.w = q.w();
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.translation.x = pose(0, 3);
    tf_msg.transform.translation.y = pose(1, 3);
    tf_msg.transform.translation.z = pose(2, 3);
    tf_pub->sendTransform(tf_msg);

    // std::cout << "algorithm: " << duration_cast<milliseconds>(t2 - t1).count() << "ms"
    //           << ", local map pub: " << duration_cast<milliseconds>(t4 - t3).count() << "ms"
    //           << std::endl;
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

std::map<double, Eigen::Vector3d> gps_map;
void gps_callback(sensor_msgs::NavSatFixPtr p_msg) {
    static double last_gps = 0;
    if (p_msg->status.status != prog_cfg.gps_status_valid) return;
    // check the position_covariance of the gps message
    if (p_msg->position_covariance[0] > 0.1 &&
        p_msg->position_covariance[4] > 0.1 &&
        p_msg->position_covariance[8] > 0.1 ) return;
        
    char zone[13];
    Eigen::Vector3d utm;
    gps_common::LLtoUTM(p_msg->latitude, p_msg->longitude, utm.y(), utm.x(), zone);
    utm.z() = p_msg->altitude;

    gps_map[p_msg->header.stamp.toSec()] = utm;

    if (p_msg->header.stamp.toSec() - last_gps < 5) return;
    last_gps = p_msg->header.stamp.toSec();
    app->FeedGPS(p_msg->header.stamp.toSec(), utm);
}

void imu_callback(sensor_msgs::ImuPtr msg) {
    Eigen::Quaternionf imu_q(
        msg->orientation.w, 
        msg->orientation.x, 
        msg->orientation.y, 
        msg->orientation.z);
    app->FeedImu(imu_q);
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
    app = std::make_unique<FreeSLAM::Mapping>(cfg["app_cfg"].as<FreeSLAM::MappingConfigs>());

    /****************************************/
    /*            configure ros             */
    /****************************************/
    ros::init(argc, argv, "FreeSLAM_mapping_node");
    ros::NodeHandle nh;
    // subscriber here
    if (prog_cfg.lidar_type == "pointcloud2") {
        lidar_sub = nh.subscribe(prog_cfg.lidar_topic, prog_cfg.queue_size, &pointcloud2_callback,
                                 ros::TransportHints().tcpNoDelay());
    } else if (prog_cfg.lidar_type == "livox") {
        lidar_sub = nh.subscribe(prog_cfg.lidar_topic, prog_cfg.queue_size, &livox_callback,
                                 ros::TransportHints().tcpNoDelay());
    }
    gps_sub = nh.subscribe(prog_cfg.gps_topic, prog_cfg.queue_size, &gps_callback);
    imu_sub = nh.subscribe(prog_cfg.imu_topic, prog_cfg.queue_size, &imu_callback);
    // publisher here
    registered_pub = nh.advertise<sensor_msgs::PointCloud2>("registered", 1);
    global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("local_map", 1, true);
    history_frame_pub = nh.advertise<sensor_msgs::PointCloud2>("history_frame", 1, true);
    loop_pub = nh.advertise<sensor_msgs::PointCloud2>("loopclosure", 1);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", prog_cfg.queue_size);
    trajectory_pub = nh.advertise<geometry_msgs::PoseArray>("trajectory", 1, true);
    tf_pub = std::make_unique<tf2_ros::TransformBroadcaster>();

    /****************************************/
    /*          configure rosbag            */
    /****************************************/
    BagPlayer bag_player(nh);
    if (!prog_cfg.bag_file.empty()) {
        bag_player.open(prog_cfg.bag_file, {prog_cfg.lidar_topic, prog_cfg.gps_topic, prog_cfg.imu_topic});
        bag_player.set_queue_size(prog_cfg.queue_size);
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
        sensor_msgs::PointCloud2 history_frame_msg;
        pcl::toROSMsg(app->GetHistoryFrame(), history_frame_msg);
        // local_map_msg.header.seq = header.seq;
        // local_map_msg.header.stamp = header.stamp;
        history_frame_msg.header.frame_id = "map";
        history_frame_pub.publish(history_frame_msg);
    }

    /****************************************/
    /*    (optional): some post process     */
    /****************************************/
    app->Shutdown();

    FreeSLAM::PointVec global_map;
    app->GetGlobalMapPoints(global_map);
    pcl::io::savePCDFileBinary("global.pcd", global_map);

    std::map<double, Eigen::Matrix4f> trajectory;
    app->GetTrajectory(trajectory);
    std::ofstream ofs("slam.txt");
    for (auto [stamp, pose] : trajectory) {
        Eigen::Quaternionf q(pose.topLeftCorner<3, 3>());
        Eigen::Vector3f t(pose.topRightCorner<3, 1>());
        ofs << std::setprecision(std::numeric_limits<double>::max_digits10) << stamp << " " << t.x()
            << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
            << q.w() << std::endl;
    }
    ofs.close();

    // std::ofstream ofs;
    ofs.open("gps.txt");
    for (auto [stamp, pos] : gps_map) {
        ofs << std::setprecision(std::numeric_limits<double>::max_digits10) << stamp << " "
            << pos.x() << " " << pos.y() << " " << pos.z() << " 0 0 0 1" << std::endl;
    }
    ofs.close();

    std::cout << "utm origin: " << std::setprecision(std::numeric_limits<double>::max_digits10)
              << app->GetUTMOrigin() << std::endl;

    return 0;
}
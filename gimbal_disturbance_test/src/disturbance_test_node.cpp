#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include "cyber_msgs/GimbalCommand.h"

struct Config {
    double home_pan_deg = 0.0;
    double home_tilt_deg = 0.0;
    double hold_duration_s = 30.0;
    double loop_rate_hz = 50.0;
};

Config loadConfig(const std::string& path) {
    Config cfg;
    YAML::Node yaml = YAML::LoadFile(path);
    cfg.home_pan_deg = yaml["home_pan_deg"].as<double>();
    cfg.home_tilt_deg = yaml["home_tilt_deg"].as<double>();
    cfg.hold_duration_s = yaml["hold_duration_s"].as<double>();
    cfg.loop_rate_hz = yaml["loop_rate_hz"].as<double>();
    return cfg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "disturbance_test_node");
    ros::NodeHandle nh;

    std::string config_path = ros::package::getPath("gimbal_disturbance_test")
                              + "/configs/disturbance_test.yaml";
    Config cfg = loadConfig(config_path);

    ROS_INFO("Disturbance test: hold PAN=%.1f TILT=%.1f for %.0fs",
             cfg.home_pan_deg, cfg.home_tilt_deg, cfg.hold_duration_s);
    ROS_INFO("Push the gimbal body now. Ctrl+C to end early.");

    ros::Publisher cmd_pub = nh.advertise<cyber_msgs::GimbalCommand>("/gimbal_cmd", 10);

    ros::Time t_start = ros::Time::now();

    double tilt_cmd = std::max(-80.0, std::min(80.0, cfg.home_tilt_deg));

    // Publish initial hold command once
    {
        ros::Time now = ros::Time::now();
        cyber_msgs::GimbalCommand pan_msg;
        pan_msg.header.stamp = now;
        pan_msg.cmd = 0x4B;
        pan_msg.data = cfg.home_pan_deg;
        cmd_pub.publish(pan_msg);

        cyber_msgs::GimbalCommand tilt_msg;
        tilt_msg.header.stamp = now;
        tilt_msg.cmd = 0x4D;
        tilt_msg.data = tilt_cmd;
        cmd_pub.publish(tilt_msg);
    }
    ROS_INFO("Hold command sent.");

    // Low-rate keep-alive and countdown timer
    ros::Timer timer = nh.createTimer(ros::Duration(5.0),
        [&](const ros::TimerEvent&) {
            double elapsed = (ros::Time::now() - t_start).toSec();

            if (elapsed > cfg.hold_duration_s) {
                ROS_INFO("Hold duration reached. Shutting down.");
                ros::shutdown();
                return;
            }

            int remaining = (int)(cfg.hold_duration_s - elapsed);
            ROS_INFO("  %ds remaining...", remaining);

            // Keep-alive: re-send hold command every 5s
            ros::Time now = ros::Time::now();
            cyber_msgs::GimbalCommand pan_msg;
            pan_msg.header.stamp = now;
            pan_msg.cmd = 0x4B;
            pan_msg.data = cfg.home_pan_deg;
            cmd_pub.publish(pan_msg);

            cyber_msgs::GimbalCommand tilt_msg;
            tilt_msg.header.stamp = now;
            tilt_msg.cmd = 0x4D;
            tilt_msg.data = tilt_cmd;
            cmd_pub.publish(tilt_msg);
        });

    ros::spin();

    cyber_msgs::GimbalCommand stop_msg;
    stop_msg.header.stamp = ros::Time::now();
    stop_msg.cmd = 0x00;
    stop_msg.data = 0;
    cmd_pub.publish(stop_msg);
    ROS_INFO("STOP sent.");

    return 0;
}

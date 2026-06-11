#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <vector>
#include "cyber_msgs/GimbalCommand.h"

enum TestPhase {
    PHASE_IDLE,
    PHASE_HOME,
    PHASE_STEP_PAN,
    PHASE_STEP_TILT,
    PHASE_DONE
};

struct TestConfig {
    double loop_rate_hz = 50.0;
    double home_pan_deg = 0.0;
    double home_tilt_deg = 0.0;
    bool post_test_home = true;
    double hold_time_s = 3.0;
    std::vector<double> pan_angles_deg;
    std::vector<double> tilt_angles_deg;
};

TestConfig loadConfig(const std::string& path) {
    TestConfig cfg;
    YAML::Node yaml = YAML::LoadFile(path);
    cfg.loop_rate_hz = yaml["loop_rate_hz"].as<double>();
    cfg.home_pan_deg = yaml["home_pan_deg"].as<double>();
    cfg.home_tilt_deg = yaml["home_tilt_deg"].as<double>();
    cfg.post_test_home = yaml["post_test_home"].as<bool>();
    cfg.hold_time_s = yaml["hold_time_s"].as<double>();
    cfg.pan_angles_deg = yaml["pan_angles_deg"].as<std::vector<double>>();
    cfg.tilt_angles_deg = yaml["tilt_angles_deg"].as<std::vector<double>>();
    return cfg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "step_response_test_node");
    ros::NodeHandle nh;

    std::string config_path = ros::package::getPath("gimbal_response_test") + "/configs/test_config.yaml";
    TestConfig cfg = loadConfig(config_path);

    ROS_INFO("Step response test loaded:");
    ROS_INFO("  PAN steps: %zu, TILT steps: %zu", cfg.pan_angles_deg.size(), cfg.tilt_angles_deg.size());
    ROS_INFO("  Hold time: %.1fs, Rate: %.0fHz", cfg.hold_time_s, cfg.loop_rate_hz);

    ros::Publisher cmd_pub = nh.advertise<cyber_msgs::GimbalCommand>("/gimbal_cmd", 10);

    TestPhase phase = PHASE_IDLE;
    ros::Time phase_start;
    int step_idx = 0;

    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / cfg.loop_rate_hz),
        [&](const ros::TimerEvent&) {
            ros::Time now = ros::Time::now();
            double elapsed = (now - phase_start).toSec();

            double pan_cmd = cfg.home_pan_deg;
            double tilt_cmd = cfg.home_tilt_deg;

            switch (phase) {
                case PHASE_IDLE:
                    phase = PHASE_HOME;
                    phase_start = now;
                    ROS_INFO("Phase: HOME");
                    return;

                case PHASE_HOME:
                    if (elapsed > 1.5) {
                        phase = PHASE_STEP_PAN;
                        phase_start = now;
                        step_idx = 0;
                        ROS_INFO("Phase: STEP_PAN (%zu steps)", cfg.pan_angles_deg.size());
                        return;
                    }
                    break;

                case PHASE_STEP_PAN:
                    if (step_idx >= (int)cfg.pan_angles_deg.size()) {
                        phase = PHASE_STEP_TILT;
                        phase_start = now;
                        step_idx = 0;
                        ROS_INFO("Phase: STEP_TILT (%zu steps)", cfg.tilt_angles_deg.size());
                        return;
                    }
                    pan_cmd = cfg.pan_angles_deg[step_idx];
                    if (elapsed > cfg.hold_time_s) {
                        step_idx++;
                        phase_start = now;
                        if (step_idx < (int)cfg.pan_angles_deg.size()) {
                            ROS_INFO("  PAN step %d: %.1f deg", step_idx, cfg.pan_angles_deg[step_idx]);
                        }
                    }
                    break;

                case PHASE_STEP_TILT:
                    if (step_idx >= (int)cfg.tilt_angles_deg.size()) {
                        phase = PHASE_DONE;
                        phase_start = now;
                        ROS_INFO("Phase: DONE");
                        return;
                    }
                    tilt_cmd = cfg.tilt_angles_deg[step_idx];
                    if (elapsed > cfg.hold_time_s) {
                        step_idx++;
                        phase_start = now;
                        if (step_idx < (int)cfg.tilt_angles_deg.size()) {
                            ROS_INFO("  TILT step %d: %.1f deg", step_idx, cfg.tilt_angles_deg[step_idx]);
                        }
                    }
                    break;

                case PHASE_DONE:
                    if (cfg.post_test_home) {
                        pan_cmd = cfg.home_pan_deg;
                        tilt_cmd = cfg.home_tilt_deg;
                        if (elapsed > 2.0) {
                            ros::shutdown();
                            return;
                        }
                    } else {
                        ros::shutdown();
                        return;
                    }
                    break;
            }

            tilt_cmd = std::max(-80.0, std::min(80.0, tilt_cmd));

            cyber_msgs::GimbalCommand pan_msg;
            pan_msg.header.stamp = now;
            pan_msg.cmd = 0x4B;  // PAN
            pan_msg.data = pan_cmd;
            cmd_pub.publish(pan_msg);

            cyber_msgs::GimbalCommand tilt_msg;
            tilt_msg.header.stamp = now;
            tilt_msg.cmd = 0x4D;  // TILT
            tilt_msg.data = tilt_cmd;
            cmd_pub.publish(tilt_msg);
        });

    ros::spin();

    // STOP on exit
    cyber_msgs::GimbalCommand stop_msg;
    stop_msg.header.stamp = ros::Time::now();
    stop_msg.cmd = 0x00;
    stop_msg.data = 0;
    cmd_pub.publish(stop_msg);
    ROS_INFO("STOP sent.");

    return 0;
}

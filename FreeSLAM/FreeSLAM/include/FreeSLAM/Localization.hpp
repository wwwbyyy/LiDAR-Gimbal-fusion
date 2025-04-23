#pragma once

#include <string>

#include "FreeSLAM/HashVoxel.hpp"
#include "FreeSLAM/Registration.hpp"
#include "reflcpp/core.hpp"
#include "reflcpp/yaml.hpp"

namespace FreeSLAM {
/**
 * @brief the config parameters for FreeSLAM::Localiztion
 *
 */
struct LocalizationConfigs {
    // path to map info file
    std::string map_info;
    // voxel configs
    HashVoxelConfigs voxel_cfg;
    // registration configs
    RegistrationConfigs reg_cfg;
    // enable deskew
    bool enable_deskew = false;
    // input points filter
    double range_min, range_max, z_min, z_max;
    // input points voxel downsample
    double tracking_voxel;
    // extinct lidar -> body.
    // 0: [TODO] auto extinct
    // 3: translation only
    // 6: translation + euler angle
    // 7: translation + quaternion
    std::vector<float> extinct;
    // extince imu -> body
    // 3: euler angle
    std::vector<float> imu_extinct;
    // initial pose
    // 0: initialize from gps
    // 6: translation + euler angle
    // 7: translation + quaternion
    std::vector<float> initial;
};
/**
 * @brief map info
 *
 */
struct MapInfo {
    // path to map file
    std::string map_file;
    // coordinate origin for the map
    std::vector<double> utm_origin;
    
    std::string zone;
};
}  // namespace FreeSLAM

REFLCPP_METAINFO(
    FreeSLAM::LocalizationConfigs, ,
    (map_info)(voxel_cfg)(reg_cfg)(enable_deskew)(range_min)(range_max)(z_min)(z_max)(tracking_voxel)(extinct)(imu_extinct))
REFLCPP_YAML(FreeSLAM::LocalizationConfigs)

REFLCPP_METAINFO(FreeSLAM::MapInfo, , (map_file)(utm_origin)(zone))
REFLCPP_YAML(FreeSLAM::MapInfo)

namespace FreeSLAM {
/**
 * @brief FreeSLAM::Localiztion
 *
 */
class Localization : public LocalizationConfigs, public MapInfo {
   public:
    /**
     * @brief construct from LocalizationConfigs struct
     *
     * @param configs
     */
    explicit Localization(const LocalizationConfigs &configs);

    /**
     * @brief feed pointcloud data
     *
     * @param stamp the data timestamp [unit: s]
     * @param frame the pointcloud frame
     */
    void FeedPointCloud(double stamp, PointVec frame);

    /**
     * @brief feed gps data in utm coordinate
     *
     * @param stamp the data timestamp [unit: s]
     * @param utm   gps data in utm coordinate
     */
    void FeedGPS(double stamp, const Eigen::Vector3d &utm);

    /**
     * @brief feed orientation data
     *
     * @param stamp
     * @param q
     */
    void FeedOrientation(double stamp, const Eigen::Quaternionf &q);

    /**
     * @brief feed pose data from rviz (mannual)
     *
     * @param stamp the data timestamp [unit: s]
     * @param p     position
     * @param q     quaternion
     */
    void FeedPose(double stamp, const Eigen::Vector3f &p, const Eigen::Quaternionf &q);

    /**
     * @brief Get the Current Pose object
     *
     * @return pose in utm coordinate
     */
    Eigen::Matrix4f GetCurrentPose() const { return curr_pose; }

    /**
     * @brief Get the Utm Origin object
     *
     * @return Eigen::Vector3d
     */
    Eigen::Vector3d GetUtmOrigin() const {
        return Eigen::Map<const Eigen::Vector3d>(utm_origin.data());
    }

    /**
     * @brief Get the Registered Points object
     *
     * @param points
     */
    void GetRegisteredPoints(PointVec &points) const { points = std::move(registered_frame); }

    /**
     * @brief Get the Global Map Points object
     *
     * @param points
     */
    void GetGlobalMapPoints(PointVec &points) const { map.GetAllPoints(points); }

   public:
    bool waiting_position = true;
    bool waiting_orientation = true;
    bool flag_new_mannual_pose = false;

    Eigen::Isometry3f extinct_transform = Eigen::Isometry3f::Identity();
    Eigen::Vector6f velocity = Eigen::Vector6f::Zero();
    Eigen::Matrix4f curr_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f initial_pose = Eigen::Matrix4f::Identity();
    double curr_stamp = -1;
    PointVec registered_frame;
    PointVec map_points;
    HashVoxel map;
};
}  // namespace FreeSLAM

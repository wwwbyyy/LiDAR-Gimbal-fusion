#pragma once

#include <gtsam/nonlinear/ISAM2.h>

#include <Eigen/Geometry>
#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <fstream>
#include <iostream>

#include "FreeSLAM/HashVoxel.hpp"
#include "FreeSLAM/Registration.hpp"
#include "FreeSLAM/types.hpp"
#include "reflcpp/core.hpp"
#include "reflcpp/yaml.hpp"

namespace FreeSLAM {

/**
 * @brief the config parameters for FreeSLAM::Mapping
 *
 */
struct MappingConfigs {
    // voxel configs
    HashVoxelConfigs voxel_cfg;
    // configs during tracking
    RegistrationConfigs reg_cfg_track;
    // configs during loop-closure
    RegistrationConfigs reg_cfg_loop;
    // enable deskew
    bool enable_deskew = false;
    // enable loop-closure
    bool enable_loopclosure = false;
    // whether update local map after gps and loop-closure
    bool update_local_map = true;
    // enable incremental-mapping 
    bool enable_incremental_mapping = true;
    // extince lidar -> gps
    std::vector<float> extinct;
    // extince imu -> gps
    std::vector<float> imu_extinct;
    // assume the first 'n' frames to be static
    int static_frames = 1;
    // input pointcloud filter
    double range_min = 2, range_max = 200;
    // pointcloud voxel leaf size while tracking
    double tracking_voxel = 0.2;
    // pointcloud voxel leaf size while mapping
    double mapping_voxel = 0.2;
    //
    int loop_frame_skip = 500;
    //
    int loop_trials_num = 10;
    //
    float loop_distance_threshold = 50;
    //
    int loop_interval_ms = 1;
    //
    float loop_overlap_threshold = 0.9;
    //
    float loop_fitness_threshold = 0.1;
    //
    int loop_local_frames = 400;
};
}  // namespace FreeSLAM

REFLCPP_METAINFO(
    FreeSLAM::MappingConfigs, ,
    (voxel_cfg)(reg_cfg_track)(reg_cfg_loop)(enable_deskew)(enable_loopclosure)(update_local_map)(enable_incremental_mapping)(extinct)(imu_extinct)(static_frames)(range_min)(range_max)(tracking_voxel)(mapping_voxel)(loop_frame_skip)(loop_trials_num)(loop_distance_threshold)(loop_interval_ms)(loop_overlap_threshold)(loop_fitness_threshold)(loop_local_frames))
REFLCPP_YAML(FreeSLAM::MappingConfigs)

namespace FreeSLAM {
/**
 * @brief FreeSLAM::Mapping
 *
 */
class Mapping : private MappingConfigs {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    /**
     * @brief construct from FreeSLAMConfigs struct
     *
     * @param configs the config parameters
     */
    explicit Mapping(const MappingConfigs &configs);

    /**
     * @brief Destroy the FreeSLAM object
     *
     */
    ~Mapping();

    /**
     * @brief
     *
     */
    void Shutdown();

    /**
     * @brief [For incremental mapping] Save all intermediate data of mapping process
     *
     * @param filename the saved file's name
     */
    void SaveAllFrames(const std::string &filename);

    /**
     * @brief [For incremental mapping] Load all intermediate data of the former mapping process
     *
     * @param filename the data file's name
     */
    void LoadAllFrames(const std::string &filename);

    /**
     * @brief [For incremental mapping] Save the factor graph of the former mapping process
     *
     * @param graph the factor graph
     * @param values the estimate values
     * @param graph_filename the graph file's name
     */
    void SaveGraphAndValues(const gtsam::NonlinearFactorGraph& graph, 
                        const gtsam::Values& values, 
                        const std::string& graph_filename);
    /**
     * @brief [For incremental mapping] Load the factor graph of the former mapping process
     *
     * @param graph_filename the graph file's name
     */
    void LoadGraphAndValues(const std::string& graph_filename);

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
     * @brief feed imu orientation for incrementa mapping initialization
     *
     * @param imu_q   imu orientation quaternion
     */
    void FeedImu(const Eigen::Quaternionf &imu_q);

    /**
     * @brief Get the Current Pose object
     *
     * @return Eigen::Matrix4f
     */
    Eigen::Matrix4f GetCurrentPose() const {
        return pose_vec.empty() ? Eigen::Matrix4f::Identity() : pose_vec.back();
    }

    /**
     * @brief Get the Local Map object
     *
     * @return std::shared_ptr<HashVoxel>
     */
    std::shared_ptr<HashVoxel> GetLocalMap() const {
        // std::unique_lock lock(mtx);
        return p_voxel;
    }

    /**
     * @brief Get the Local Map object
     *
     * @param points output points
     */
    void GetLocalMapPoints(PointVec &points) const { GetLocalMap()->GetAllPoints(points); }

    /**
     * @brief Get the Global Map Points object
     *
     * @param points
     */
    void GetGlobalMapPoints(PointVec &points) const;

    /**
     * @brief Get the Registered Points object
     *
     * @param points output points
     */
    void GetRegisteredPoints(PointVec &points) const { points = tracking_vec.back(); }

    /**
     * @brief Get the Loop Points object
     *
     * @param points output points
     */
    void GetLoopPoints(PointVec &points) const {
        // std::unique_lock lock(mtx);
        points = std::move(loop_frame);
    }

    /**
     * @brief Get the Trajectory object
     *
     * @param trajectory
     */
    void GetTrajectory(std::map<double, Eigen::Matrix4f> &trajectory) const {
        trajectory.clear();
        // std::unique_lock lock(mtx);
        int frame_num = pose_vec.size();
        for (int i = 0; i < frame_num; i++) trajectory[stamp_vec[i]] = pose_vec[i];
    }

    /**
     * @brief
     *
     * @return Eigen::Vector3d
     */
    Eigen::Vector3d GetUTMOrigin() const { return *p_utm_origin; }

    PointVec GetHistoryFrame() const {return history_frame;}

   private:
    /**
     * @brief Loop Closure. Run in another thread.
     *
     */
    void LoopClosure();

    std::thread loop_thread;
    std::atomic_bool loop_running = false;

    mutable PointVec loop_frame;
    PointVec history_frame;

    mutable std::mutex mtx;
    std::shared_ptr<HashVoxel> p_voxel;

    std::unique_ptr<Eigen::Vector3d> p_utm_origin;

    std::vector<double> stamp_vec;
    std::vector<PointVec> tracking_vec;
    std::vector<PointVec> mapping_vec;
    EigenVec<Eigen::Matrix4f> pose_vec;
    std::map<double, Eigen::Vector3d> gps_map;

    // Eigen::Matrix4f prev_pose = Eigen::Matrix4f::Identity();
    Eigen::Vector6f velocity = Eigen::Vector6f::Zero();

    // for incremental mapping
    int former_frame_num;
    bool relocalization_waiting_position = true;
    bool relocalization_waiting_heading = true;
    Eigen::Matrix4f relocalization_initialpose = Eigen::Matrix4f::Identity();

    std::unique_ptr<gtsam::ISAM2> p_isam2;
};
}  // namespace FreeSLAM

#include "FreeSLAM/Mapping.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/Key.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include <pcl/common/io.h>
#include <pcl/io/auto_io.h>
#include <pcl/common/transforms.h>

#include <chrono>

#include "FreeSLAM/Deskew.hpp"
#include "FreeSLAM/Downsample.hpp"
#include "FreeSLAM/Registration.hpp"

namespace FreeSLAM {

Mapping::Mapping(const MappingConfigs &configs)
    : MappingConfigs(configs), p_voxel(std::make_shared<HashVoxel>(voxel_cfg)) {

    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.1;
    params.relinearizeSkip = 1;
    p_isam2 = std::make_unique<gtsam::ISAM2>(params);

    mapping_vec.reserve(1000000);
    tracking_vec.reserve(1000000);
    pose_vec.reserve(1000000);
    stamp_vec.reserve(1000000);

    if (enable_incremental_mapping){
        // load former mapping status
        std::cout << "Loading status data" << std::endl;
        LoadAllFrames("status.bin");
        // // load former map
        // PointVec map_points;
        // pcl::io::load("global.pcd", map_points);
        // p_voxel->AddPoints(map_points);
        // load former factor graph
        LoadGraphAndValues("factor_graph.g2o");
        std::cout << "Loading done" << std::endl;
    }

    if (enable_loopclosure) {
        loop_running = true;
        loop_thread = std::thread(&Mapping::LoopClosure, this);
    }

}

Mapping::~Mapping() { Shutdown(); }

void Mapping::SaveAllFrames(const std::string &filename) {
    std::ofstream file(filename, std::ios::binary);

    // Save UTM origin
    if (p_utm_origin != nullptr) {
        double utm_origin[3] = {(*p_utm_origin)(0), (*p_utm_origin)(1), (*p_utm_origin)(2)};
        file.write(reinterpret_cast<const char*>(utm_origin), sizeof(double) * 3);
    } else {
        // Save a zero vector if no UTM origin is available
        double utm_origin[3] = {0.0, 0.0, 0.0};
        file.write(reinterpret_cast<const char*>(utm_origin), sizeof(double) * 3);
    }

    // write frame count
    int frame_num = pose_vec.size();
    file.write(reinterpret_cast<const char*>(&frame_num), sizeof(int));
    std::cout << "framenum: " << frame_num << std::endl;
    std::cout << "mappingsize: " << mapping_vec.size() << "trackingsize: " << tracking_vec.size() << std::endl;

    for (int i = 0; i < frame_num; i++) {
        // write pose_vec
        file.write(reinterpret_cast<const char*>(pose_vec[i].data()), sizeof(float) * 16);

        // write mapping_vec
        int mapping_cloud_size = mapping_vec[i].size();
        file.write(reinterpret_cast<const char*>(&mapping_cloud_size), sizeof(int));
        for (const auto &pt : mapping_vec[i]) {
            float x = pt.x, y = pt.y, z = pt.z, i = pt.intensity;
            double t = pt.timestamp;
            file.write(reinterpret_cast<const char*>(&x), sizeof(float));
            file.write(reinterpret_cast<const char*>(&y), sizeof(float));
            file.write(reinterpret_cast<const char*>(&z), sizeof(float));
            file.write(reinterpret_cast<const char*>(&i), sizeof(float));
            file.write(reinterpret_cast<const char*>(&t), sizeof(double));
        }
        // if (i == 10)
        // {
        //     FreeSLAM::PointVec frame_10 = mapping_vec[i];
        //     pcl::io::savePCDFileBinary("frame_10_save.pcd", frame_10);
        //     std::cout << "pose_10: " << pose_vec[i] << std::endl;
        // }


        // write tracking_vec
        int tracking_cloud_size = tracking_vec[i].size();
        file.write(reinterpret_cast<const char*>(&tracking_cloud_size), sizeof(int));
        for (const auto &pt : tracking_vec[i]) {
            float x = pt.x, y = pt.y, z = pt.z, i = pt.intensity;
            double t = pt.timestamp;
            file.write(reinterpret_cast<const char*>(&x), sizeof(float));
            file.write(reinterpret_cast<const char*>(&y), sizeof(float));
            file.write(reinterpret_cast<const char*>(&z), sizeof(float));
            file.write(reinterpret_cast<const char*>(&i), sizeof(float));
            file.write(reinterpret_cast<const char*>(&t), sizeof(double));
        }

        // write stamp_vec
        file.write(reinterpret_cast<const char*>(&stamp_vec[i]), sizeof(double));

    }
    file.close();

    // FreeSLAM::PointVec global_map;
    // global_map.clear();
    // for (int i = 0; i < frame_num; i++) {
    //     PointVec tmp = mapping_vec[i];
    //     for (auto &pt : tmp) {
    //         pt.getVector4fMap() = pose_vec[i] * pt.getVector4fMap();
    //     }
    //     global_map += tmp;
    // }

    // // FreeSLAM::PointVec global_map;
    // // GetGlobalMapPoints(global_map);
    // pcl::io::savePCDFileBinary("global_save.pcd", global_map);
}

void Mapping::LoadAllFrames(const std::string &filename) {
    std::ifstream file(filename, std::ios::binary);

    // read UTM origin
    double utm_origin[3];
    file.read(reinterpret_cast<char*>(utm_origin), sizeof(double) * 3);
    p_utm_origin = std::make_unique<Eigen::Vector3d>(utm_origin[0], utm_origin[1], utm_origin[2]);
    
    // read frame num
    int frame_count;
    file.read(reinterpret_cast<char*>(&frame_count), sizeof(int));
    std::cout << "framenum: " << frame_count << std::endl;
    former_frame_num = frame_count;

    // mapping_vec.resize(frame_count);
    // tracking_vec.resize(frame_count);
    pose_vec.resize(frame_count);
    stamp_vec.resize(frame_count);

    for (int i = 0; i < frame_count; i++) {
        // read pose_vec
        pose_vec[i] = Eigen::Matrix4f::Zero();
        file.read(reinterpret_cast<char*>(pose_vec[i].data()), sizeof(float) * 16);

        // read mapping_vec
        int mapping_cloud_size = 0;
        PointVec map_frame;
        file.read(reinterpret_cast<char*>(&mapping_cloud_size), sizeof(int));
        for (int i = 0; i < mapping_cloud_size; i++) {
            PointType pt;
            file.read(reinterpret_cast<char*>(&pt.x), sizeof(float));
            file.read(reinterpret_cast<char*>(&pt.y), sizeof(float));
            file.read(reinterpret_cast<char*>(&pt.z), sizeof(float));
            file.read(reinterpret_cast<char*>(&pt.intensity), sizeof(float));
            file.read(reinterpret_cast<char*>(&pt.timestamp), sizeof(double));
            map_frame.emplace_back(pt);
        }
        mapping_vec.emplace_back(map_frame);
        // if (i == 10)
        // {
        //     FreeSLAM::PointVec frame_10 = mapping_vec[i];
        //     pcl::io::savePCDFileBinary("frame_10_load.pcd", frame_10);
        //     std::cout << "pose_10: " << pose_vec[i] << std::endl;
        // }

        // read tracking_vec
        int tracking_cloud_size = 0;
        PointVec track_frame;
        file.read(reinterpret_cast<char*>(&tracking_cloud_size), sizeof(int));
        for (int i = 0; i < tracking_cloud_size; i++) {
            PointType pt;
            file.read(reinterpret_cast<char*>(&pt.x), sizeof(float));
            file.read(reinterpret_cast<char*>(&pt.y), sizeof(float));
            file.read(reinterpret_cast<char*>(&pt.z), sizeof(float));
            file.read(reinterpret_cast<char*>(&pt.intensity), sizeof(float));
            file.read(reinterpret_cast<char*>(&pt.timestamp), sizeof(double));
            track_frame.emplace_back(pt);
        }
        tracking_vec.emplace_back(track_frame);
        // read stamp_vec
        file.read(reinterpret_cast<char*>(&stamp_vec[i]), sizeof(double));
    }
    file.close();

    // FreeSLAM::PointVec global_map;
    // global_map.clear();
    // // // for (int i = 0; i < frame_count; i++) {
    // // //     PointVec tmp = mapping_vec[i];
    // // //     for (auto &pt : tmp) {
    // // //         pt.getVector4fMap() = pose_vec[i] * pt.getVector4fMap();
    // // //     }
    // // //     global_map += tmp;
    // // // }

    // for (int i = 0; i < frame_count; ++i) {
    //     pcl::PointCloud<PointType> transformed_cloud;
    //     pcl::transformPointCloud(mapping_vec[i], transformed_cloud, pose_vec[i]); // 使用对应的位姿矩阵
    //     global_map += transformed_cloud; // 全局点云累加
    // }

    // // // FreeSLAM::PointVec global_map;
    // // // GetGlobalMapPoints(global_map);
    // pcl::io::savePCDFileBinary("global_load.pcd", global_map);
}

void Mapping::SaveGraphAndValues(const gtsam::NonlinearFactorGraph& graph, 
                        const gtsam::Values& values, 
                        const std::string& graph_filename) {
    // save factor graph (between factors)
    gtsam::writeG2o(graph, values, graph_filename); // save in g2o format

    // save PriorFactor and GPSFactor
    std::ofstream additional_file("additional_graph.g2o");

    for (size_t i = 0; i < graph.size(); ++i) {
        auto factor = graph[i];

        // PriorFactor
        if (auto prior = boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(factor)) {
            gtsam::Key key = prior->key();
            gtsam::Pose3 pose = prior->prior();
            gtsam::Vector sigmas = prior->noiseModel()->sigmas();

            additional_file << "PRIOR_FACTOR " << key << " "
                            << pose.x() << " " << pose.y() << " " << pose.z() << " "
                            << pose.rotation().quaternion().x() << " "
                            << pose.rotation().quaternion().y() << " "
                            << pose.rotation().quaternion().z() << " "
                            << pose.rotation().quaternion().w() << " "
                            << sigmas(0) << " " << sigmas(1) << " " << sigmas(2) << ""
                            << sigmas(3) << " " << sigmas(4) << " " << sigmas(5) << "\n";
        }

        // GPSFactor
        if (auto gps = boost::dynamic_pointer_cast<gtsam::GPSFactor>(factor)) {
            gtsam::Key key = gps->key();
            gtsam::Point3 gps_point = gps->measurementIn();
            gtsam::Vector sigmas = gps->noiseModel()->sigmas();

            additional_file << "GPS_FACTOR " << key << " "
                            << gps_point.x() << " " << gps_point.y() << " " << gps_point.z() << " "
                            << sigmas(0) << " " << sigmas(1) << " " << sigmas(2) << "\n";
        }
    }

    additional_file.close();

}

void Mapping::LoadGraphAndValues(const std::string& graph_filename) {

    // 从文件中加载因子图和初值
    gtsam::GraphAndValues graph_and_values = gtsam::readG2o("factor_graph.g2o", true);
    auto graph = graph_and_values.first;
    auto values = graph_and_values.second;

    int frame_num = pose_vec.size();

    using namespace gtsam::symbol_shorthand;

    // // 添加第一帧的 PriorFactor
    // {
    //     auto noise = gtsam::noiseModel::Diagonal::Sigmas(
    //         (gtsam::Vector(6) << M_PI, M_PI, M_PI, 1e2, 1e2, 1e2).finished());
    //     gtsam::Pose3 pose3(Eigen::Matrix4f::Identity().cast<double>());
    //     gtsam::PriorFactor<gtsam::Pose3> factor(P(0), pose3, noise);
    //     // global_graph.add(factor);
    //     graph->add(factor);
    //     // global_initialEstimate.insert(P(0), pose3);
    //     // p_isam2->update(global_graph, global_initialEstimate);
    // }


    std::ifstream file("additional_graph.g2o");
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        gtsam::Key key;

        if (line.find("PRIOR_FACTOR") != std::string::npos) {
            double x, y, z, qx, qy, qz, qw, sigma_0, sigma_1, sigma_2, sigma_3, sigma_4, sigma_5;
            iss >> type >> key >> x >> y >> z >> qx >> qy >> qz >> qw >> sigma_0 >> sigma_1 >> sigma_2 >> sigma_3 >> sigma_4 >> sigma_5;

            gtsam::Pose3 pose(gtsam::Rot3::Quaternion(qw, qx, qy, qz), gtsam::Point3(x, y, z));
            auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << sigma_0, sigma_1, sigma_2, sigma_3, sigma_4, sigma_5).finished());
            graph->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(key, pose, noise_model);
        }

        if (line.find("GPS_FACTOR") != std::string::npos) {
            double x, y, z, sigma_x, sigma_y, sigma_z;
            iss >> type >> key >> x >> y >> z >> sigma_x >> sigma_y >> sigma_z;

            gtsam::Point3 gps_point(x, y, z);
            auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(3) << sigma_x, sigma_y, sigma_z).finished());
            graph->emplace_shared<gtsam::GPSFactor>(key, gps_point, noise_model);
        }
    }

    file.close();

    // 一次性更新 ISAM2
    p_isam2->update(*graph, *values);
    // p_isam2->update(global_graph, global_initialEstimate);

}

void Mapping::Shutdown() {
    loop_running = false;
    if (loop_thread.joinable()) loop_thread.join();

    using namespace gtsam::symbol_shorthand;
    gtsam::Values result = p_isam2->calculateEstimate();
    int frame_num = pose_vec.size();
    for (int i = 0; i < frame_num; i++) {
        pose_vec[i] = result.at<gtsam::Pose3>(P(i)).matrix().cast<float>();
    }

    static bool saved = false;

    if (!enable_incremental_mapping && !saved){
        std::cout << "Saving status..." << std::endl;
        SaveAllFrames("status.bin");
        gtsam::NonlinearFactorGraph graph = p_isam2->getFactorsUnsafe();
        SaveGraphAndValues(graph, result, "factor_graph.g2o");
        std::cout << "Saving done" << std::endl;
        saved = true;
    }
}

void Mapping::FeedPointCloud(double stamp, PointVec frame) {
    std::vector<int> frame_indices;
    // filter points by range
    RangeSample(frame, frame_indices, range_min, range_max);
    // downsample points
    std::vector<int> tracking_indices = frame_indices;
    RandomVoxelSample(frame, tracking_indices, tracking_voxel);

    static bool new_bag_initializing = false;
    int frame_num = pose_vec.size();
    // incremental mapping - new bag reset velocity for deskew
    if (enable_incremental_mapping){
        if (relocalization_waiting_heading || relocalization_waiting_position){
            return;
        }
        if (!relocalization_waiting_heading && !relocalization_waiting_position && 
            frame_num - former_frame_num < static_frames){
            // ? velocity = (read from imu)
            velocity = Eigen::Vector6f::Zero();
            new_bag_initializing = true;
        }
    }

    // deskew frame by constant velocity assumption
    if (enable_deskew) {
        double stamp_max =
            std::max_element(frame.begin(), frame.end(), [](const auto &p0, const auto &p1) {
                return p0.timestamp < p1.timestamp;
            })->timestamp;
        Eigen::Vector6f deskew_delta = velocity * (stamp_max - stamp);
        Deskew(frame, stamp, stamp_max, frame_indices, deskew_delta, false);
        stamp = stamp_max;
    }
    // apply extinct
    if (extinct.size() == 3) {
        for (auto &pt : frame) {
            pt.getVector3fMap() -= Eigen::Map<const Eigen::Vector3f>(extinct.data());
        }
    }
    // copy input shared data which need lock
    std::unique_lock lock(mtx);
    // int frame_num = pose_vec.size();
    Eigen::Matrix4f prev_pose;
    Eigen::Matrix4f predicted = Eigen::Matrix4f::Identity();
    std::shared_ptr<HashVoxel> p_voxel_locked;
    {
        // std::unique_lock lock(mtx);
        p_voxel_locked = p_voxel;
        if (frame_num >= 1) {
            prev_pose = pose_vec.back();
            predicted = prev_pose * SE3Exp(velocity * (stamp - stamp_vec.back()));
        }
        // for incremental mapping init
        if (new_bag_initializing){
            predicted = relocalization_initialpose;
            // velocity = Eigen::Vector6f::Zero();
        }
    }
    // frame to map registration
    Eigen::Matrix4f curr_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f curr_delta = Eigen::Matrix4f::Identity();
    Eigen::Matrix6f information = Eigen::Matrix6f::Identity() * 1e8;
    if (!new_bag_initializing && frame_num >= static_frames) {
        Eigen::Matrix4f initial = predicted;
        // std::cout << "initial: " << initial << std::endl;
        // std::cout << "map size: " << p_voxel_locked->GetVoxelNum() << std::endl;
        auto result =
            Registration(*p_voxel_locked, frame, tracking_indices, initial, reg_cfg_track);
        double overlap = result.matches / (double)tracking_indices.size();
        // std::cout << "[Track]: matches = [" << result.matches << "/" << tracking_indices.size()
        //           << "]"
        //           << ", overlap = " << overlap << ", iterations = " << result.iterations
        //           << ", fitness = " << result.fitness << std::endl;
        curr_pose = initial * result.delta;
        curr_delta = prev_pose.inverse() * curr_pose;
        information = result.information_matrix;
        velocity = SE3Log(curr_delta) / (stamp - stamp_vec.back());
    }
    // prepare mapping indices
    std::vector<int> mapping_indices;
    if (tracking_voxel != mapping_voxel) {
        mapping_indices = frame_indices;
        RandomVoxelSample(frame, mapping_indices, mapping_voxel);
    } else {
        mapping_indices = tracking_indices;
    }
    // prepare mapping frame
    PointVec mapping_frame;
    pcl::copyPointCloud(frame, mapping_indices, mapping_frame);
    // prepare tracking frame
    PointVec tracking_frame;
    pcl::copyPointCloud(frame, tracking_indices, tracking_frame);
    // prepare update isam2
    using namespace gtsam::symbol_shorthand;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate;
    if (frame_num == 0) {
        auto noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << M_PI, M_PI, M_PI, 1e2, 1e2, 1e2).finished());
        // auto noise = gtsam::noiseModel::Constrained::Sigmas(
        //     (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());
        gtsam::Pose3 pose3(curr_pose.cast<double>());
        gtsam::PriorFactor<gtsam::Pose3> factor(P(0), pose3, noise);
        graph.add(factor);
        initialEstimate.insert(P(0), pose3);
    } else {
        if (new_bag_initializing){
            auto noise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << M_PI, M_PI, M_PI, 1e2, 1e2, 1e2).finished());
            if (frame_num - former_frame_num < static_frames){
                gtsam::Pose3 pose3(predicted.cast<double>());
                gtsam::PriorFactor<gtsam::Pose3> factor(P(frame_num), pose3, noise);
                graph.add(factor);
                initialEstimate.insert(P(frame_num), pose3);
            }
        } else {
            auto noise = gtsam::noiseModel::Gaussian::Information(information.cast<double>());
            gtsam::Pose3 between(curr_delta.cast<double>());
            gtsam::BetweenFactor<gtsam::Pose3> factor(P(frame_num - 1), P(frame_num), between, noise);
            gtsam::Pose3 pose3(curr_pose.cast<double>());
            graph.add(factor);
            initialEstimate.insert(P(frame_num), pose3);
        }
    }
    // save output shared data which need lock
    {
        // std::unique_lock lock(mtx);
        // update isam2
        p_isam2->update(graph, initialEstimate);
        // update curr_pose
        if (!pose_vec.empty()) curr_pose = pose_vec.back() * curr_delta;
        if (new_bag_initializing){
            static int cnt = 0;
            curr_pose = predicted;
            cnt++;
            if (cnt == static_frames) new_bag_initializing = false;
        }
        // save results
        stamp_vec.emplace_back(stamp);
        pose_vec.emplace_back(curr_pose);
        tracking_vec.emplace_back(tracking_frame);
        mapping_vec.emplace_back(mapping_frame);

        p_voxel_locked->AddPoints(mapping_frame, {}, curr_pose);
    }
}

void Mapping::FeedImu(const Eigen::Quaternionf &imu_q){
    // use imu for relocalization of incremental mapping
    if (enable_incremental_mapping && relocalization_waiting_heading){
        Eigen::Matrix3f imu_extinct_rot = 
            (Eigen::AngleAxisf(imu_extinct[0], Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(imu_extinct[1], Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(imu_extinct[2], Eigen::Vector3f::UnitZ()))
              .matrix(); 
        relocalization_initialpose.topLeftCorner<3, 3>() = 
            imu_extinct_rot * imu_q.toRotationMatrix();
        // // test
        relocalization_initialpose.topLeftCorner<3, 3>() = Eigen::Matrix3f::Identity();
        std::cout << "heading:" << relocalization_initialpose.topLeftCorner<3, 3>() << std::endl;
        relocalization_waiting_heading = false;
    }
}

void Mapping::FeedGPS(double stamp, const Eigen::Vector3d &utm) {
    if (pose_vec.size() < 100) return;
    if (p_utm_origin == nullptr) {
        p_utm_origin = std::make_unique<Eigen::Vector3d>(utm);
    }
    Eigen::Vector3d local_utm = utm - *p_utm_origin;
    gps_map[stamp] = local_utm;

    // for incremental mapping init
    if (enable_incremental_mapping && relocalization_waiting_position){
        // test
        // local_utm << -0.118665, 3.16225, -0.0888985;
        relocalization_initialpose.topRightCorner<3, 1>() = local_utm.cast<float>();
        std::cout << "position:" << relocalization_initialpose.topRightCorner<3, 1>() << std::endl;
        relocalization_waiting_position = false;
    }
    if (enable_incremental_mapping)
    {
        if (pose_vec.size() - former_frame_num < 100)
            return;
    }

    if (gps_map.size() < 2) return;
    auto it = gps_map.end();
    --it;
    --it;

    std::unique_lock lock(mtx);

    auto upper_it = std::upper_bound(stamp_vec.begin(), stamp_vec.end(), it->first);
    auto lower_it = upper_it;
    lower_it--;

    auto closest_it = (*upper_it - it->first < it->first - *lower_it) ? upper_it : lower_it;
    int closest_idx = closest_it - stamp_vec.begin();

    if (!((*upper_it >= it->first) && (*lower_it <= it->first))) {
        exit(-1);
    }

    using namespace gtsam::symbol_shorthand;
    gtsam::NonlinearFactorGraph graph;
    auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 1, 1, 1).finished());
    gtsam::GPSFactor factor(P(closest_idx), it->second, noise);
    graph.add(factor);

    // save output shared data which need lock
    int frame_num;
    {
        //
        frame_num = pose_vec.size();
        // update isam2
        p_isam2->update(graph);
        for (int i = 0; i < 2; i++) p_isam2->update();
        // save poses
        gtsam::Values result = p_isam2->calculateEstimate();
        for (int i = 0; i < frame_num; i++) {
            pose_vec[i] = result.at<gtsam::Pose3>(P(i)).matrix().cast<float>();
        }
        if (update_local_map) {
            // build new voxel map
            auto p_new_voxel = std::make_shared<HashVoxel>(voxel_cfg);
            for (int i = frame_num - 1; i >= std::max(0, frame_num - loop_local_frames); i--) {
                p_new_voxel->AddPoints(mapping_vec[i], {}, pose_vec[i]);
                if ((int)p_new_voxel->GetVoxelNum() >= voxel_cfg.max_num_voxels) break;
            }
            // update new voxel map
            p_voxel = p_new_voxel;
        }
    }
}

void Mapping::LoopClosure() {
    std::cout << "[Loop]: loop-closure enable." << std::endl;
    std::random_device rd;
    std::default_random_engine rng(rd());
    std::shared_ptr<HashVoxel> p_loop_voxel;
    while (loop_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(loop_interval_ms));
        if ((int)pose_vec.size() < loop_frame_skip || 
        (enable_incremental_mapping && (int)pose_vec.size() - former_frame_num < loop_frame_skip)) continue;

        std::unique_lock lock(mtx);

        // int current_id = pose_vec.size() - loop_frame_skip / 2;
        int current_id = pose_vec.size() - 1;
        Eigen::Matrix4f current_pose = pose_vec[current_id];

        std::vector<int> candidates;
        for (int i = 2; i < (int)pose_vec.size() - loop_frame_skip; i++) {
            float distance = (pose_vec[i].inverse() * current_pose).topRightCorner<3, 1>().norm();
            if (distance < loop_distance_threshold) candidates.emplace_back(i);
        }
        if (candidates.empty()) continue;
        std::shuffle(candidates.begin(), candidates.end(), rng);

        {
            // std::unique_lock lock(mtx);
            p_loop_voxel = std::make_shared<HashVoxel>(*p_voxel);
        }
        int trials = std::min((int)candidates.size(), loop_trials_num);
        int history_id = -1;
        Eigen::Matrix4f closed_pose;
        Eigen::Matrix6f information;
        PointVec *p_history_frame = nullptr;
        bool ok = false;
        for (int i = 0; i < trials; i++) {
            history_id = candidates[i];
            const auto &history_pose = pose_vec[history_id];
            p_history_frame = tracking_vec[history_id].size() > mapping_vec[history_id].size()
                                  ? &tracking_vec[history_id]
                                  : &mapping_vec[history_id];
            auto result =
                Registration(*p_loop_voxel, *p_history_frame, {}, history_pose, reg_cfg_loop);
            double overlap = result.matches / (double)p_history_frame->size();
            if (history_id < 4000)
            {
                history_frame.clear();
                pcl::transformPointCloud(*p_history_frame, history_frame, history_pose); // 使用对应的位姿矩阵

                std::cout << "[Loop]: matches = [" << result.matches << "/" << p_history_frame->size()
                        << "]"
                        << ", overlap = " << overlap << ", iterations = " << result.iterations
                        << ", fitness = " << result.fitness << std::endl;
                std::cout << "[Loop]: delta = " << std::endl << result.delta << std::endl;
                std::cout << "[Loop]: try: current-" << current_id << ", history-" << history_id << std::endl;
            }
            if (result.fitness > loop_fitness_threshold) continue;
            if (overlap < loop_overlap_threshold) continue;
            closed_pose = history_pose * result.delta;
            information = result.information_matrix;
            ok = true;
            break;
        }
        if (!ok) continue;

        std::cout << "[Loop]: loop found: current-" << current_id << ", history-" << history_id << std::endl;

        using namespace gtsam::symbol_shorthand;
        gtsam::NonlinearFactorGraph graph;
        auto noise = gtsam::noiseModel::Gaussian::Information(information.cast<double>());
        // auto noise = gtsam::noiseModel::Diagonal::Sigmas(
        //     (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2).finished());
        gtsam::Pose3 between((closed_pose.inverse() * current_pose).cast<double>());
        gtsam::BetweenFactor<gtsam::Pose3> factor(P(history_id), P(current_id), between, noise);
        graph.add(factor);

        // save output shared data which need lock
        int frame_num;
        {
            // std::unique_lock lock(mtx);
            //
            frame_num = pose_vec.size();
            // update isam2
            p_isam2->update(graph);
            for (int i = 0; i < 10; i++) p_isam2->update();
            // save poses
            gtsam::Values result = p_isam2->calculateEstimate();
            for (int i = 0; i < frame_num; i++) {
                pose_vec[i] = result.at<gtsam::Pose3>(P(i)).matrix().cast<float>();
            }

            if (update_local_map) {
                // build new voxel map
                p_loop_voxel->Reset();
                for (int i = frame_num - 1; i >= std::max(0, frame_num - loop_local_frames); i--) {
                    p_loop_voxel->AddPoints(mapping_vec[i], {}, pose_vec[i]);
                    if ((int)p_loop_voxel->GetVoxelNum() >= voxel_cfg.max_num_voxels) break;
                }
                // update new voxel map
                p_voxel = p_loop_voxel;
            }

            // save loop frame points
            loop_frame = *p_history_frame;
            for (auto &pt : loop_frame) {
                pt.getVector4fMap() = closed_pose * pt.getVector4fMap();
            }
        }
    }
}

void Mapping::GetGlobalMapPoints(PointVec &points) const {
    points.clear();
    // // std::unique_lock lock(mtx);
    // PointVec tmp;
    // int frame_num = pose_vec.size();
    // for (int i = 0; i < frame_num; i++) {
    //     tmp = mapping_vec[i];
    //     for (auto &pt : tmp) {
    //         pt.getVector4fMap() = pose_vec[i] * pt.getVector4fMap();
    //     }
    //     points += tmp;
    // }

    points.clear();
    int frame_num = pose_vec.size();
    for (int i = 0; i < frame_num; ++i) {
        pcl::PointCloud<PointType> transformed_cloud;
        pcl::transformPointCloud(mapping_vec[i], transformed_cloud, pose_vec[i]); // 使用对应的位姿矩阵
        points += transformed_cloud; // 全局点云累加
    }
}

}  // namespace FreeSLAM
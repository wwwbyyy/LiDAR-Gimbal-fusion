#include "FreeSLAM/Localization.hpp"

#include <pcl/io/auto_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <filesystem>

#include "FreeSLAM/Deskew.hpp"
#include "FreeSLAM/Downsample.hpp"
#include "FreeSLAM/Registration.hpp"

namespace fs = std::filesystem;

namespace FreeSLAM {

Localization::Localization(const LocalizationConfigs &configs)
    : LocalizationConfigs(configs),
      MapInfo(YAML::LoadFile(map_info).as<MapInfo>()),
      map(voxel_cfg) {
    if (fs::path(map_file).is_relative()) {
        map_file = fs::path(map_info).parent_path() / map_file;
    }
    // PointVec map_points;
    pcl::io::load(map_file, map_points);
    map.AddPoints(map_points);

    if (extinct.size() == 3) {
        extinct_transform(0, 3) = extinct[0];
        extinct_transform(1, 3) = extinct[1];
        extinct_transform(2, 3) = extinct[2];
    } else if (extinct.size() == 6) {
        extinct_transform(0, 3) = extinct[0];
        extinct_transform(1, 3) = extinct[1];
        extinct_transform(2, 3) = extinct[2];
        extinct_transform.matrix().topLeftCorner<3, 3>() =
            (Eigen::AngleAxisf(extinct[3], Eigen::Vector3f::UnitX()) *
             Eigen::AngleAxisf(extinct[4], Eigen::Vector3f::UnitY()) *
             Eigen::AngleAxisf(extinct[5], Eigen::Vector3f::UnitZ()))
                .matrix();
    } else if (extinct.size() == 7) {
        extinct_transform(0, 3) = extinct[0];
        extinct_transform(1, 3) = extinct[1];
        extinct_transform(2, 3) = extinct[2];
        extinct_transform.matrix().topLeftCorner<3, 3>() =
            Eigen::Quaternionf(extinct[3], extinct[4], extinct[5], extinct[6]).matrix();
    }

    if (initial.size() == 6) {
        curr_pose(0, 3) = initial[0];
        curr_pose(1, 3) = initial[1];
        curr_pose(2, 3) = initial[2];
        curr_pose.matrix().topLeftCorner<3, 3>() =
            (Eigen::AngleAxisf(initial[3], Eigen::Vector3f::UnitX()) *
             Eigen::AngleAxisf(initial[4], Eigen::Vector3f::UnitY()) *
             Eigen::AngleAxisf(initial[5], Eigen::Vector3f::UnitZ()))
                .matrix();
        waiting_position = waiting_orientation = false;
    } else if (initial.size() == 7) {
        curr_pose(0, 3) = initial[0];
        curr_pose(1, 3) = initial[1];
        curr_pose(2, 3) = initial[2];
        curr_pose.matrix().topLeftCorner<3, 3>() =
            Eigen::Quaternionf(initial[3], initial[4], initial[5], initial[6]).matrix();
        waiting_position = waiting_orientation = false;
    }
}

void Localization::FeedPointCloud(double stamp, PointVec frame) {
    // waiting initial
    if (waiting_position || waiting_orientation) return;
    
    if (flag_new_mannual_pose)
    {
        printf("Here\n");
        std::vector<float> Z;
        Z.clear();
        float check_size = 1.0;
        for (int i = 0; i < map_points.size(); i++)
        {
            if (std::abs(map_points[i].x - initial_pose(0, 3)) < check_size && \
                std::abs(map_points[i].y - initial_pose(1, 3)) < check_size)
            {
                Z.push_back(map_points[i].z);
            }
            else continue;
        }
        if (!Z.empty())
        {
            //cluster 2 types
            int iter = 0, maxIter = 10;
            int low_size = 0, high_size = 0;
            float z_low = 0, z_high = 0, pre_z_low, pre_z_high;
            pre_z_low = *std::min_element(Z.begin(), Z.end());
            pre_z_high = *std::max_element(Z.begin(), Z.end());

            do
            {
                iter++;
                z_low = z_high = 0;
                for (int i = 0; i < Z.size(); i++)
                {
                    if (std::abs(Z[i] - pre_z_low) <= std::abs(Z[i] - pre_z_high))
                    {
                        z_low += Z[i];
                        low_size++;
                    }
                    else    
                    {
                        z_high += Z[i];
                        high_size++;
                    }
                }
                if (low_size != 0 && high_size != 0)
                {
                    z_low /= low_size;
                    pre_z_low = z_low;
                    z_high /= high_size;
                    pre_z_high = z_high;
                }
            } while ((std::abs(z_low - pre_z_low) > 0.1 || std::abs(z_high - pre_z_high) > 0.1) && iter < maxIter);
            printf("zlow = %f, zhigh = %f\n", pre_z_low, pre_z_high); 
            initial_pose(2, 3) = pre_z_low;

            // float average_z = 0;
            // for (int i = 0; i < Z.size(); i++)
            // {
            //     average_z += Z[i];
            // }
            // average_z = average_z / Z.size() + 1.8;
            // printf("z = %f\n", average_z);
            // initial_pose(2,3) = average_z;
            curr_pose = initial_pose;
            velocity = Eigen::Vector6f::Zero();
            printf("New Mannual Pose\n");
        }
        else printf("Invalid Pose! Please re-input 2D Pose Estimation.\n");

        flag_new_mannual_pose = false;
    }

    //
    std::vector<int> frame_indices;
    // downsample points
    PROFILE("RandomVoxelSample", RandomVoxelSample(frame, frame_indices, tracking_voxel));
    // filter points by range
    PROFILE("RangeSample", RangeSample(frame, frame_indices, range_min, range_max, z_min, z_max));
    // apply extinct
    PROFILE("Extinct", {
        for (int i : frame_indices) {
            frame[i].getVector3fMap() = extinct_transform * frame[i].getVector3fMap();
        }
    });
    // deskew frame by constant velocity assumption
    double prev_stamp = curr_stamp;
    curr_stamp = stamp;
    PROFILE("Deskew", {
        if (enable_deskew) {
            curr_stamp =
                std::max_element(frame.begin(), frame.end(), [](const auto &p0, const auto &p1) {
                    return p0.timestamp < p1.timestamp;
                })->timestamp;
            Eigen::Vector6f deskew_delta = velocity * (curr_stamp - stamp);
            Deskew(frame, stamp, curr_stamp, frame_indices, deskew_delta, false);
        }
    });
    // compute predicted pose
    Eigen::Matrix4f prev_pose = curr_pose;
    Eigen::Matrix4f predicted_pose = curr_pose;
    if (prev_stamp != -1) {
        predicted_pose = curr_pose * SE3Exp(velocity * (curr_stamp - prev_stamp));
    }
    // frame to map registration
    using namespace std::chrono;
    RegistrationResults result;
    PROFILE("Registration",
            { result = Registration(map, frame, frame_indices, predicted_pose, reg_cfg); });
    curr_pose = predicted_pose * result.delta;
    if (prev_stamp != -1) {
        velocity = SE3Log(prev_pose.inverse() * curr_pose) / (curr_stamp - prev_stamp);
    }
    double overlap = (double)result.matches / frame_indices.size();
    std::cout << "[registration] iteration=" << result.iterations
            << ", points=" << frame_indices.size() << ", fitness=" << result.fitness
            << ", overlap=" << overlap << std::endl;
    // save registered points
    PROFILE("Save registered", {
        registered_frame.resize(frame_indices.size());
        for (int i = 0; i < (int)frame_indices.size(); i++) {
            registered_frame[i].getVector4fMap().noalias() =
                curr_pose * frame[frame_indices[i]].getVector4fMap();
        }
    });
    
}

void Localization::FeedGPS(double stamp, const Eigen::Vector3d &utm) {
    if (!waiting_position) return;
    initial_pose.topRightCorner<3, 1>() = curr_pose.topRightCorner<3, 1>() =
        (utm - Eigen::Map<Eigen::Vector3d>(utm_origin.data())).cast<float>();
    std::cout << utm << "\n" << utm_origin.data() << "\n" << initial_pose << std::endl;
    waiting_position = false;
}

void Localization::FeedOrientation(double stamp, const Eigen::Quaternionf &q) {
    // if (!waiting_orientation) return;
    flag_new_mannual_pose = true;
    initial_pose.topLeftCorner<3, 3>() = \
    curr_pose.topLeftCorner<3, 3>() = Eigen::AngleAxisf(imu_extinct[0], Eigen::Vector3f::UnitX()) *
                                      Eigen::AngleAxisf(imu_extinct[1], Eigen::Vector3f::UnitY()) *
                                      Eigen::AngleAxisf(imu_extinct[2], Eigen::Vector3f::UnitZ()) *
                                      q.matrix();
    waiting_orientation = false;
}

void Localization::FeedPose(double stamp, const Eigen::Vector3f &p, const Eigen::Quaternionf &q) {
    // if (!waiting_orientation) return;
    flag_new_mannual_pose = true;

    initial_pose.topRightCorner<3, 1>() = curr_pose.topRightCorner<3, 1>() = p;
    initial_pose.topLeftCorner<3, 3>()  = curr_pose.topLeftCorner<3, 3>()  = q.matrix();

    waiting_position = waiting_orientation = false;
}


}  // namespace FreeSLAM

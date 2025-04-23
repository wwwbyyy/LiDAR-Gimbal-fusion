#include "FreeSLAM/Registration.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <execution>

#include "FreeSLAM/Deskew.hpp"

namespace FreeSLAM {

template <int N, typename T>
static T fit_plane(Eigen::Matrix<T, 4, 1> &pca_result, const PointVec &points) {
    if (points.size() != N) return false;

    Eigen::Matrix<T, 3, 1> normvec;

    Eigen::Matrix<T, N, 3> A;
    Eigen::Matrix<T, N, 1> b;

    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < (int)points.size(); j++) {
        A(j, 0) = points[j].x;
        A(j, 1) = points[j].y;
        A(j, 2) = points[j].z;
    }

    normvec = A.colPivHouseholderQr().solve(b);

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    T error = 0;
    for (const auto &p : points) {
        Eigen::Matrix<T, 4, 1> temp = p.getVector4fMap();
        error += fabs(pca_result.dot(temp));
    }
    return error / N;
}

RegistrationResults Registration(const HashVoxel &voxel, const PointVec &frame,
                                 const std::vector<int> &indices, const Eigen::Matrix4f &initial,
                                 const RegistrationConfigs &configs) {
    bool no_indices = indices.empty();
    RegistrationResults results;
    int points_num = no_indices ? frame.size() : indices.size();
    Eigen::MatrixXf A_full(points_num, 6);
    Eigen::VectorXf B_full(points_num, 1);

    std::vector<int> par_indices(points_num);
    std::iota(par_indices.begin(), par_indices.end(), 0);

    results.iterations = 1;
    for (int _ = 0; _ < configs.max_iterations; _++, results.iterations++) {
        std::atomic_int matches = 0;
        std::atomic_int fitness1e6 = 0;

        auto func = [&](int i) {
            int idx = no_indices ? i : indices[i];
            PointType pb;
            pb.getVector4fMap() = results.delta * frame[idx].getVector4fMap();
            PointType pw;
            pw.getVector4fMap() = initial * pb.getVector4fMap();
            PointVec points_to_fit = voxel.GetNearestPoints(pw, 5);
            if (points_to_fit.size() != 5) return;
            Eigen::Vector4f nw;
            fit_plane<5>(nw, points_to_fit);
            if (nw.hasNaN()) return;
            Eigen::Vector4f nb = initial.transpose() * nw;

            double e = pb.getVector4fMap().dot(nb);
            double b = 1.0 / (1.0 + configs.robust_kernel * e);
            double a = b;

            int locked_matches = matches++;
            A_full(locked_matches, 0) = (pb.y * nb.z() - pb.z * nb.y()) * a;
            A_full(locked_matches, 1) = (pb.z * nb.x() - pb.x * nb.z()) * a;
            A_full(locked_matches, 2) = (pb.x * nb.y() - pb.y * nb.x()) * a;
            A_full(locked_matches, 3) = nb.x() * a;
            A_full(locked_matches, 4) = nb.y() * a;
            A_full(locked_matches, 5) = nb.z() * a;
            B_full(locked_matches, 0) = -e * b;

            fitness1e6 += std::abs(e) * 1e6;
        };
        std::for_each(std::execution::par_unseq, par_indices.begin(), par_indices.end(), func);

        results.matches = matches;
        results.fitness = fitness1e6 / 1e6 / (double)matches;

        auto A = A_full.topRows(matches);
        auto B = B_full.topRows(matches);
        auto H = A.transpose() * A;
        auto G = A.transpose() * B;
        Eigen::Vector6f X = H.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(G);
        // Eigen::Vector6f X = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
        results.delta = SE3Exp(X) * results.delta;

        results.information_matrix = H;

        float rotation = X.topRows<3>().norm();
        float translation = X.bottomRows<3>().norm();
        if (rotation < configs.rotation_epsilon && translation < configs.translation_epsilon) break;
    }
    return results;
}

}  // namespace FreeSLAM
#include "jomon/height_map.hpp"
#include <Eigen/Eigenvalues>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace jomon {

HeightMap build_height_map(const MeshData&        mesh,
                            const std::vector<int>& selected,
                            const Config&           cfg) {
    if (selected.empty())
        throw std::runtime_error("No vertices selected for height map");

    const int K = static_cast<int>(selected.size());

    // 1. Centroid
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (int idx : selected) centroid += mesh.V.row(idx).transpose();
    centroid /= static_cast<float>(K);

    // 2. PCA: build 3x3 covariance matrix
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (int idx : selected) {
        Eigen::Vector3f d = mesh.V.row(idx).transpose() - centroid;
        cov += d * d.transpose();
    }
    cov /= static_cast<float>(K);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
    // Eigenvalues in ascending order → smallest = plane normal direction
    Eigen::Vector3f plane_normal = es.eigenvectors().col(0);
    Eigen::Vector3f u_axis       = es.eigenvectors().col(2);  // largest variance
    Eigen::Vector3f v_axis       = es.eigenvectors().col(1);

    // 3. Project all selected vertices to (u, v, signed_distance)
    // Determine grid bounds
    float u_min =  std::numeric_limits<float>::max();
    float u_max = -std::numeric_limits<float>::max();
    float v_min =  std::numeric_limits<float>::max();
    float v_max = -std::numeric_limits<float>::max();

    for (int idx : selected) {
        Eigen::Vector3f d = mesh.V.row(idx).transpose() - centroid;
        float u = d.dot(u_axis);
        float v = d.dot(v_axis);
        if (u < u_min) u_min = u;
        if (u > u_max) u_max = u;
        if (v < v_min) v_min = v;
        if (v > v_max) v_max = v;
    }

    const float p = cfg.grid_pitch_mm;
    int cols = static_cast<int>(std::ceil((u_max - u_min) / p)) + 1;
    int rows = static_cast<int>(std::ceil((v_max - v_min) / p)) + 1;
    if (cols <= 0 || rows <= 0)
        throw std::runtime_error("Degenerate grid dimensions");

    // 4. Accumulate heights per cell (mean)
    Eigen::MatrixXf acc   = Eigen::MatrixXf::Zero(rows, cols);
    Eigen::MatrixXi count = Eigen::MatrixXi::Zero(rows, cols);

    for (int idx : selected) {
        Eigen::Vector3f d = mesh.V.row(idx).transpose() - centroid;
        float u   = d.dot(u_axis);
        float v   = d.dot(v_axis);
        float h   = d.dot(plane_normal);

        int ci = static_cast<int>((u - u_min) / p);
        int ri = static_cast<int>((v - v_min) / p);
        ci = std::max(0, std::min(cols - 1, ci));
        ri = std::max(0, std::min(rows - 1, ri));

        acc(ri, ci)   += h;
        count(ri, ci) += 1;
    }

    // 5. Build HeightMap
    HeightMap hmap;
    hmap.grid_pitch = p;
    hmap.origin     = centroid + u_min * u_axis + v_min * v_axis;
    hmap.u_axis     = u_axis;
    hmap.v_axis     = v_axis;

    const float kNaN = std::numeric_limits<float>::quiet_NaN();
    hmap.H.resize(rows, cols);
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            hmap.H(r, c) = (count(r, c) > 0)
                           ? acc(r, c) / static_cast<float>(count(r, c))
                           : kNaN;

    hmap.H_detrended.resize(rows, cols);
    hmap.H_detrended.fill(kNaN);

    return hmap;
}

} // namespace jomon

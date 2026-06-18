#include "jomon/face_extract.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace jomon {

namespace {

// Weighted Union-Find with path compression + union by rank
struct UF {
    std::vector<int> parent, rank_;

    explicit UF(int n) : parent(n), rank_(n, 0) {
        std::iota(parent.begin(), parent.end(), 0);
    }

    int find(int x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];  // path halving
            x = parent[x];
        }
        return x;
    }

    void unite(int a, int b) {
        a = find(a); b = find(b);
        if (a == b) return;
        if (rank_[a] < rank_[b]) std::swap(a, b);
        parent[b] = a;
        if (rank_[a] == rank_[b]) ++rank_[a];
    }
};

} // anonymous namespace

std::vector<int> extract_target_face(const MeshData& mesh, const Config& cfg) {
    const int N = static_cast<int>(mesh.V.rows());
    const int M = static_cast<int>(mesh.F.rows());

    if (N == 0 || M == 0)
        throw std::runtime_error("Mesh has no vertices or faces");

    // 1. Per-face normals and areas
    Eigen::MatrixX3f face_normals(M, 3);
    Eigen::VectorXf  face_areas(M);

    for (int f = 0; f < M; ++f) {
        int a = mesh.F(f, 0), b = mesh.F(f, 1), c = mesh.F(f, 2);
        Eigen::Vector3f e1 = mesh.V.row(b) - mesh.V.row(a);
        Eigen::Vector3f e2 = mesh.V.row(c) - mesh.V.row(a);
        Eigen::Vector3f cross = e1.cross(e2);
        face_areas(f)     = cross.norm() * 0.5f;
        float len = cross.norm();
        face_normals.row(f) = (len > 1e-12f) ? Eigen::Vector3f(cross / len) : Eigen::Vector3f::Zero();
    }

    // 2. Per-vertex normals (area-weighted average)
    Eigen::MatrixX3f vtx_normals = Eigen::MatrixX3f::Zero(N, 3);
    for (int f = 0; f < M; ++f) {
        float w = face_areas(f);
        for (int k = 0; k < 3; ++k)
            vtx_normals.row(mesh.F(f, k)) += face_normals.row(f) * w;
    }
    for (int i = 0; i < N; ++i) {
        float len = vtx_normals.row(i).norm();
        if (len > 1e-12f) vtx_normals.row(i) /= len;
    }

    // 3. Dominant direction = mean of all vertex normals, normalised
    Eigen::Vector3f dominant = vtx_normals.colwise().mean();
    float dom_len = dominant.norm();
    if (dom_len < 1e-8f)
        throw std::runtime_error("Cannot determine dominant normal (degenerate mesh?)");
    dominant /= dom_len;

    // 4. Mark vertices within angle threshold
    const float cos_thresh = std::cos(cfg.normal_angle_threshold_deg * (float)M_PI / 180.0f);
    std::vector<bool> selected(N, false);
    for (int i = 0; i < N; ++i)
        selected[i] = (vtx_normals.row(i).dot(dominant) >= cos_thresh);

    // 5. Union-Find over faces: unite adjacent selected vertices
    UF uf(N);
    for (int f = 0; f < M; ++f) {
        int a = mesh.F(f, 0), b = mesh.F(f, 1), c = mesh.F(f, 2);
        if (selected[a] && selected[b]) uf.unite(a, b);
        if (selected[b] && selected[c]) uf.unite(b, c);
        if (selected[a] && selected[c]) uf.unite(a, c);
    }

    // 6. Count component sizes; pick largest
    std::vector<int> comp_size(N, 0);
    for (int i = 0; i < N; ++i)
        if (selected[i]) ++comp_size[uf.find(i)];

    int best_root = -1, best_count = 0;
    for (int i = 0; i < N; ++i) {
        if (comp_size[i] > best_count) {
            best_count = comp_size[i];
            best_root  = i;
        }
    }

    if (best_root < 0)
        throw std::runtime_error("No vertices passed the normal angle threshold");

    // 7. Collect indices of the largest component
    std::vector<int> result;
    result.reserve(best_count);
    for (int i = 0; i < N; ++i)
        if (selected[i] && uf.find(i) == best_root)
            result.push_back(i);

    return result;
}

} // namespace jomon

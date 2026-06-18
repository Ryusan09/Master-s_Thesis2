#include "jomon/ply_io.hpp"
#include <happly.h>
#include <stdexcept>
#include <string>

namespace jomon {

MeshData load_ply(const std::string& path) {
    happly::PLYData ply(path);

    auto& velem = ply.getElement("vertex");
    auto x = velem.getProperty<float>("x");
    auto y = velem.getProperty<float>("y");
    auto z = velem.getProperty<float>("z");

    if (x.size() != y.size() || x.size() != z.size())
        throw std::runtime_error("PLY vertex property size mismatch");

    MeshData mesh;
    Eigen::Index n = static_cast<Eigen::Index>(x.size());
    mesh.V.resize(n, 3);
    mesh.V.col(0) = Eigen::Map<Eigen::VectorXf>(x.data(), n);
    mesh.V.col(1) = Eigen::Map<Eigen::VectorXf>(y.data(), n);
    mesh.V.col(2) = Eigen::Map<Eigen::VectorXf>(z.data(), n);

    auto& felem = ply.getElement("face");
    auto faces = felem.getListProperty<int>("vertex_indices");
    Eigen::Index m = static_cast<Eigen::Index>(faces.size());
    mesh.F.resize(m, 3);
    for (Eigen::Index i = 0; i < m; ++i) {
        if (faces[i].size() != 3)
            throw std::runtime_error("Non-triangle face at index " + std::to_string(i));
        mesh.F(i, 0) = faces[i][0];
        mesh.F(i, 1) = faces[i][1];
        mesh.F(i, 2) = faces[i][2];
    }

    return mesh;
}

} // namespace jomon

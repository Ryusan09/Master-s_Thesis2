#pragma once
#include "mesh_types.hpp"
#include "config.hpp"
#include <vector>

namespace jomon {

// Select the largest connected region whose per-vertex normals are close to
// the mesh's dominant normal direction.
// Returns indices into V of the selected vertices.
std::vector<int> extract_target_face(const MeshData& mesh, const Config& cfg);

} // namespace jomon

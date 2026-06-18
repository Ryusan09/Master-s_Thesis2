#pragma once
#include "mesh_types.hpp"
#include <string>

namespace jomon {

// Read a binary PLY mesh (float32 x/y/z, triangle face list).
// Throws std::runtime_error on failure.
MeshData load_ply(const std::string& path);

} // namespace jomon

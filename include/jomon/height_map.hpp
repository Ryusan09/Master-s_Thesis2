#pragma once
#include "mesh_types.hpp"
#include "config.hpp"
#include <vector>

namespace jomon {

// Project the selected vertices onto a PCA-fitted plane and sample them
// onto a regular 2-D grid.  Returns a HeightMap with H filled and
// H_detrended left empty (filled by detrend()).
HeightMap build_height_map(const MeshData&       mesh,
                            const std::vector<int>& selected,
                            const Config&           cfg);

} // namespace jomon

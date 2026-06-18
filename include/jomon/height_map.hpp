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

// Structure-tensor analysis of hmap.H_detrended.
// Returns the dominant gradient direction (degrees from +u axis, in [0, 180)),
// which is perpendicular to the grooves and serves as a candidate rope-rolling
// direction independent of the autocorrelation result.
float dominant_gradient_direction(const HeightMap& hmap);

} // namespace jomon

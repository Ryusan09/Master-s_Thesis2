#pragma once
#include "mesh_types.hpp"
#include "config.hpp"

namespace jomon {

// Fill hmap.H_detrended = H - box_blur(H, cfg.detrend_window).
// NaN cells in H are excluded from the blur and propagated to H_detrended.
void detrend(HeightMap& hmap, const Config& cfg);

} // namespace jomon

#pragma once
#include "mesh_types.hpp"
#include "config.hpp"

namespace jomon {

// Compute the masked 2-D autocorrelation of hmap.H_detrended and find the
// first significant periodic peak.  Returns the estimated period.
// Throws std::runtime_error if no convincing peak is found.
PeriodResult estimate_period(const HeightMap& hmap, const Config& cfg);

} // namespace jomon

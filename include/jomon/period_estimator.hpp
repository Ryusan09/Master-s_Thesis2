#pragma once
#include "mesh_types.hpp"
#include "config.hpp"

namespace jomon {

// Full pipeline: PLY path → PipelineResult (period + height map + selected vertices).
// Logs progress to stderr when cfg.verbose == true.
PipelineResult run_pipeline(const std::string& ply_path, const Config& cfg);

} // namespace jomon

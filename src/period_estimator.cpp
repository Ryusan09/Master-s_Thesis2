#include "jomon/period_estimator.hpp"
#include "jomon/ply_io.hpp"
#include "jomon/face_extract.hpp"
#include "jomon/height_map.hpp"
#include "jomon/detrend.hpp"
#include "jomon/autocorr2d.hpp"
#include <fmt/core.h>
#include <chrono>

namespace jomon {

namespace {
struct Timer {
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    double elapsed() const {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    }
};
} // anonymous namespace

PipelineResult run_pipeline(const std::string& ply_path, const Config& cfg) {
    auto log = [&](const std::string& msg) {
        if (cfg.verbose) fmt::print(stderr, "[{:.2f}s] {}\n",
            std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count()
            - 0, // placeholder; use per-step timer below
            msg);
    };
    (void)log; // suppress unused-variable warning when verbose=false

    Timer t;
    auto vlog = [&](const std::string& msg) {
        if (cfg.verbose) fmt::print(stderr, "[{:.2f}s] {}\n", t.elapsed(), msg);
    };

    vlog("Loading PLY: " + ply_path);
    MeshData mesh = load_ply(ply_path);
    vlog(fmt::format("Loaded {} vertices, {} faces",
                     mesh.V.rows(), mesh.F.rows()));

    vlog("Extracting target face (normal clustering)...");
    std::vector<int> selected = extract_target_face(mesh, cfg);
    vlog(fmt::format("Selected {} vertices ({:.1f}% of total)",
                     selected.size(),
                     100.0 * selected.size() / mesh.V.rows()));

    vlog("Building height map...");
    HeightMap hmap = build_height_map(mesh, selected, cfg);
    vlog(fmt::format("Height map: {}x{} cells at {:.3f} mm/cell",
                     hmap.H.cols(), hmap.H.rows(), hmap.grid_pitch));

    // Count valid cells
    int valid = 0;
    for (int r = 0; r < hmap.H.rows(); ++r)
        for (int c = 0; c < hmap.H.cols(); ++c)
            if (!std::isnan(hmap.H(r, c))) ++valid;
    vlog(fmt::format("Valid cells: {} / {}", valid, hmap.H.rows() * hmap.H.cols()));

    vlog("Detrending...");
    detrend(hmap, cfg);

    vlog("Estimating period via 2-D autocorrelation...");
    PeriodResult result = estimate_period(hmap, cfg);

    vlog("Estimating rolling direction via structure tensor...");
    result.rolling_direction_deg = dominant_gradient_direction(hmap);
    vlog(fmt::format("rolling_dir={:.1f}deg  groove_dir={:.1f}deg",
                     result.rolling_direction_deg, result.groove_direction_deg));

    vlog(fmt::format("Done in {:.2f}s", t.elapsed()));
    return PipelineResult{result, hmap, selected};
}

} // namespace jomon

// Compiled only when JOMON_BUILD_VIEWER=ON (Polyscope is fetched automatically).
// Build with: cmake --preset windows-msvc-viewer
//
// Modes:
//   jomon_viewer <input.ply>                      -- plain mesh
//   jomon_viewer <input.ply> --highlight-target   -- target face highlighted green
//   jomon_viewer <input.ply> --period [options]   -- period stripes on target face
#include "jomon/ply_io.hpp"
#include "jomon/face_extract.hpp"
#include "jomon/period_estimator.hpp"
#include "jomon/config.hpp"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include <fmt/core.h>
#include <array>
#include <cmath>
#include <string>
#include <vector>

static void print_usage(const char* prog) {
    fmt::print(stderr,
        "Usage: {} <input.ply> [--highlight-target | --period] [pipeline options]\n"
        "\n"
        "  --highlight-target          Color the dominant face green\n"
        "  --period                    Run full pipeline and show period stripes\n"
        "\n"
        "Pipeline options (only with --period):\n"
        "  --grid-pitch     <mm>       (default 0.20)\n"
        "  --detrend-window <n>        (default 12)\n"
        "  --period-min     <mm>       (default 1.5)\n"
        "  --period-max     <mm>       (default 6.0)\n"
        "  --normal-angle   <deg>      (default 30.0)\n"
        "  --verbose\n",
        prog);
}

// ---- period stripe coloring --------------------------------------------------
// Even stripe: warm orange.  Odd stripe: cool blue.
// Off-target face: dark gray.
static constexpr std::array<double,3> COLOR_EVEN    = {0.95, 0.55, 0.10};
static constexpr std::array<double,3> COLOR_ODD     = {0.15, 0.50, 0.90};
static constexpr std::array<double,3> COLOR_TARGET  = {0.70, 0.68, 0.65};
static constexpr std::array<double,3> COLOR_OFFFACE = {0.25, 0.25, 0.25};

static std::vector<std::array<double,3>> make_period_colors(
    const jomon::MeshData& mesh,
    const jomon::PipelineResult& pr)
{
    const auto& p    = pr.period;
    const auto& hmap = pr.hmap;

    // Period direction unit vector in 3-D (in the reference plane).
    Eigen::Vector3f period_vec = p.dx_mm * hmap.u_axis + p.dy_mm * hmap.v_axis;
    // pitch_mm == period_vec.norm() by construction; use it to normalise.
    Eigen::Vector3f period_unit = period_vec / p.pitch_mm;

    // Mark selected vertices for O(1) lookup.
    std::vector<bool> is_selected(static_cast<size_t>(mesh.V.rows()), false);
    for (int idx : pr.selected)
        is_selected[static_cast<size_t>(idx)] = true;

    std::vector<std::array<double,3>> colors(static_cast<size_t>(mesh.V.rows()));

    for (Eigen::Index i = 0; i < mesh.V.rows(); ++i) {
        if (!is_selected[static_cast<size_t>(i)]) {
            colors[static_cast<size_t>(i)] = COLOR_OFFFACE;
            continue;
        }
        Eigen::Vector3f v = mesh.V.row(i).transpose();
        float proj  = (v - hmap.origin).dot(period_unit);   // mm along period dir
        int   stripe = static_cast<int>(std::floor(proj / p.pitch_mm));
        // make stripe index non-negative so modulo is always 0 or 1
        colors[static_cast<size_t>(i)] = ((stripe & 1) == 0) ? COLOR_EVEN : COLOR_ODD;
    }
    return colors;
}
// ------------------------------------------------------------------------------

int main(int argc, char** argv) {
    if (argc < 2) { print_usage(argv[0]); return 1; }

    std::string ply_path;
    bool highlight = false;
    bool show_period = false;
    jomon::Config cfg;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if      (arg == "--highlight-target")              highlight    = true;
        else if (arg == "--period")                        show_period  = true;
        else if (arg == "--verbose")                       cfg.verbose  = true;
        else if (arg == "--grid-pitch"     && i+1 < argc) cfg.grid_pitch_mm              = std::stof(argv[++i]);
        else if (arg == "--detrend-window" && i+1 < argc) cfg.detrend_window             = std::stoi(argv[++i]);
        else if (arg == "--period-min"     && i+1 < argc) cfg.period_min_mm              = std::stof(argv[++i]);
        else if (arg == "--period-max"     && i+1 < argc) cfg.period_max_mm              = std::stof(argv[++i]);
        else if (arg == "--normal-angle"   && i+1 < argc) cfg.normal_angle_threshold_deg = std::stof(argv[++i]);
        else if (arg == "--help" || arg == "-h")           { print_usage(argv[0]); return 0; }
        else if (arg[0] != '-')                            ply_path = arg;
        else { fmt::print(stderr, "Unknown option: {}\n", arg); return 1; }
    }

    if (ply_path.empty()) {
        fmt::print(stderr, "Error: no PLY file specified.\n");
        print_usage(argv[0]);
        return 1;
    }

    try {
        fmt::print(stderr, "Loading {}...\n", ply_path);
        jomon::MeshData mesh = jomon::load_ply(ply_path);
        fmt::print(stderr, "Loaded {} vertices, {} faces\n",
                   mesh.V.rows(), mesh.F.rows());

        // Build Polyscope-compatible vertex/face arrays.
        std::vector<std::array<double,3>> vertices(static_cast<size_t>(mesh.V.rows()));
        for (Eigen::Index i = 0; i < mesh.V.rows(); ++i)
            vertices[static_cast<size_t>(i)] = {mesh.V(i,0), mesh.V(i,1), mesh.V(i,2)};

        std::vector<std::array<size_t,3>> faces(static_cast<size_t>(mesh.F.rows()));
        for (Eigen::Index i = 0; i < mesh.F.rows(); ++i)
            faces[static_cast<size_t>(i)] = {
                static_cast<size_t>(mesh.F(i,0)),
                static_cast<size_t>(mesh.F(i,1)),
                static_cast<size_t>(mesh.F(i,2))};

        polyscope::init();
        auto* ps_mesh = polyscope::registerSurfaceMesh("jomon mesh", vertices, faces);

        if (show_period) {
            fmt::print(stderr, "Running period estimation pipeline...\n");
            jomon::PipelineResult pr = jomon::run_pipeline(ply_path, cfg);
            const auto& r = pr.period;
            fmt::print(stderr,
                "pitch={:.3f}mm  direction={:.1f}deg  dxdy=({:.3f},{:.3f})mm  confidence={:.2f}\n",
                r.pitch_mm, r.direction_deg, r.dx_mm, r.dy_mm, r.confidence);

            auto colors = make_period_colors(mesh, pr);
            ps_mesh->addVertexColorQuantity("period stripes", colors)->setEnabled(true);

            // Add a title annotation in the Polyscope ImGui panel.
            polyscope::state::userCallback = [&r]() {
                ImGui::TextUnformatted("=== Period estimation ===");
                ImGui::Text("pitch      : %.3f mm",  r.pitch_mm);
                ImGui::Text("direction  : %.1f deg", r.direction_deg);
                ImGui::Text("dxdy       : (%.3f, %.3f) mm", r.dx_mm, r.dy_mm);
                ImGui::Text("confidence : %.2f",     r.confidence);
                ImGui::Separator();
                ImGui::TextUnformatted("Orange stripe = 1 period");
            };

        } else if (highlight) {
            fmt::print(stderr, "Extracting target face (normal_angle={}deg)...\n",
                       cfg.normal_angle_threshold_deg);
            std::vector<int> sel = jomon::extract_target_face(mesh, cfg);
            fmt::print(stderr, "Target face: {} / {} vertices\n",
                       sel.size(), mesh.V.rows());

            std::vector<std::array<double,3>> colors(
                static_cast<size_t>(mesh.V.rows()), {0.6, 0.6, 0.6});
            for (int idx : sel)
                colors[static_cast<size_t>(idx)] = {0.2, 0.8, 0.3};

            ps_mesh->addVertexColorQuantity("target", colors)->setEnabled(true);

        } else {
            ps_mesh->setSurfaceColor({0.75f, 0.72f, 0.65f});
        }

        fmt::print(stderr, "Opening viewer (close window to exit)...\n");
        polyscope::show();

    } catch (const std::exception& e) {
        fmt::print(stderr, "Error: {}\n", e.what());
        return 1;
    }
    return 0;
}

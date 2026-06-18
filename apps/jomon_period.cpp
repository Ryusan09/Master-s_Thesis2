#include "jomon/period_estimator.hpp"
#include "jomon/config.hpp"
#include <fmt/core.h>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

static void print_usage(const char* prog) {
    fmt::print(stderr,
        "Usage: {} <input.ply> [options]\n"
        "\n"
        "Options:\n"
        "  --grid-pitch     <mm>   Height-map cell size       (default: 0.20)\n"
        "  --detrend-window <n>    Box-blur half-width cells  (default: 12)\n"
        "  --period-min     <mm>   Min expected pitch         (default: 1.5)\n"
        "  --period-max     <mm>   Max expected pitch         (default: 6.0)\n"
        "  --normal-angle   <deg>  Normal clustering angle    (default: 30.0)\n"
        "  --verbose               Print progress to stderr\n"
        "  --help\n",
        prog);
}

int main(int argc, char** argv) {
    if (argc < 2) { print_usage(argv[0]); return 1; }

    std::string ply_path;
    jomon::Config cfg;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") { print_usage(argv[0]); return 0; }
        else if (arg == "--verbose")               cfg.verbose = true;
        else if (arg == "--grid-pitch"     && i+1 < argc) cfg.grid_pitch_mm               = std::stof(argv[++i]);
        else if (arg == "--detrend-window" && i+1 < argc) cfg.detrend_window               = std::stoi(argv[++i]);
        else if (arg == "--period-min"     && i+1 < argc) cfg.period_min_mm               = std::stof(argv[++i]);
        else if (arg == "--period-max"     && i+1 < argc) cfg.period_max_mm               = std::stof(argv[++i]);
        else if (arg == "--normal-angle"   && i+1 < argc) cfg.normal_angle_threshold_deg  = std::stof(argv[++i]);
        else if (arg[0] != '-') ply_path = arg;
        else { fmt::print(stderr, "Unknown option: {}\n", arg); return 1; }
    }

    if (ply_path.empty()) {
        fmt::print(stderr, "Error: no PLY file specified.\n");
        print_usage(argv[0]);
        return 1;
    }

    try {
        jomon::PipelineResult pr = jomon::run_pipeline(ply_path, cfg);
        const jomon::PeriodResult& r = pr.period;
        fmt::print("pitch={:.3f}mm  groove_dir={:.1f}deg  confidence={:.2f}\n",
                   r.pitch_mm, r.groove_direction_deg, r.confidence);
        fmt::print("period_vec=({:.3f}, {:.3f})mm  period_dir={:.1f}deg\n",
                   r.dx_mm, r.dy_mm, r.direction_deg);
        fmt::print("grid={:.2f}mm  detrend_window={}\n",
                   cfg.grid_pitch_mm, cfg.detrend_window);
    } catch (const std::exception& e) {
        fmt::print(stderr, "Error: {}\n", e.what());
        return 1;
    }
    return 0;
}

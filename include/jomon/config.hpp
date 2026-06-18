#pragma once

namespace jomon {

struct Config {
    // --- M2: face extraction ---
    float normal_angle_threshold_deg = 30.0f;  // max deviation from dominant normal

    // --- M2: height map sampling ---
    float grid_pitch_mm = 0.20f;    // grid cell size (mm); ~1/10 of expected pitch

    // --- M2: detrend ---
    int detrend_window = 12;        // box-blur half-width in grid cells
                                    // (~3x expected pitch at 0.2mm/cell for 3mm pitch)

    // --- M3: autocorrelation peak search ---
    float period_min_mm = 1.5f;     // minimum expected rope pitch
    float period_max_mm = 6.0f;     // maximum expected rope pitch

    // --- diagnostics ---
    bool verbose = false;
};

} // namespace jomon

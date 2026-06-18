// Validates the 2-D autocorrelation peak finder with synthetic sinusoidal data.
//
// Period vector (Pu, Pv): the pattern repeats when (u,v) -> (u+Pu, v+Pv).
// Sinusoid: h(u,v) = sin(2π*(Pu*u + Pv*v) / (Pu²+Pv²))
//
// Degenerate cases to avoid: when h is purely 1D (e.g. sin(u+v)/const),
// the autocorrelation has a degenerate valley at (Pu, -Pv) with the same
// correlation as (Pu, Pv). Choose period vectors that avoid this by verifying
// that the "mirror" lag (Pu, -Pv) has negative correlation.
//
// Pass criteria: pitch within 5%, direction within 2°.
#include "jomon/autocorr2d.hpp"
#include "jomon/mesh_types.hpp"
#include "jomon/config.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static bool run_test(const char* name,
                     float Pu, float Pv,    // true period vector (mm)
                     float grid_pitch,
                     float noise_amp) {
    const float pitch_sq   = Pu * Pu + Pv * Pv;
    const float true_pitch = std::sqrt(pitch_sq);
    const float dir_true   = std::atan2(Pv, Pu) * 180.0f / (float)M_PI;

    const int COLS = 256, ROWS = 256;

    jomon::HeightMap hmap;
    hmap.grid_pitch = grid_pitch;
    hmap.origin     = Eigen::Vector3f::Zero();
    hmap.u_axis     = Eigen::Vector3f::UnitX();
    hmap.v_axis     = Eigen::Vector3f::UnitY();

    hmap.H.resize(ROWS, COLS);
    for (int r = 0; r < ROWS; ++r) {
        for (int c = 0; c < COLS; ++c) {
            float u = static_cast<float>(c) * grid_pitch;
            float v = static_cast<float>(r) * grid_pitch;
            float phase = 2.0f * (float)M_PI * (Pu * u + Pv * v) / pitch_sq;
            float noise = noise_amp * ((float)(rand() % 1000) / 500.0f - 1.0f);
            hmap.H(r, c) = std::sin(phase) + noise;
        }
    }
    hmap.H_detrended = hmap.H;

    jomon::Config cfg;
    cfg.grid_pitch_mm = grid_pitch;
    cfg.period_min_mm = true_pitch * 0.5f;
    cfg.period_max_mm = true_pitch * 2.0f;

    try {
        jomon::PeriodResult res = jomon::estimate_period(hmap, cfg);

        float pitch_err = std::abs(res.pitch_mm - true_pitch) / true_pitch;
        // Direction: allow ±180° ambiguity (autocorrelation is symmetric).
        // The search restricts to dx >= 0 half-plane, so compare accordingly.
        float dir_cand_a = std::abs(res.direction_deg - dir_true);
        float dir_cand_b = std::abs(res.direction_deg - dir_true + 360.0f);
        float dir_cand_c = std::abs(res.direction_deg - dir_true - 360.0f);
        float dir_err = std::min({dir_cand_a, dir_cand_b, dir_cand_c});
        // Wrap to nearest half-circle (±90° max meaningful error)
        if (dir_err > 90.f) dir_err = 180.f - dir_err;

        bool ok = (pitch_err < 0.05f && dir_err < 2.0f);
        printf("  %-42s  pitch: %.3f->%.3f (err %.1f%%)  "
               "dir: %.1f->%.1f (err %.1f deg)  conf=%.2f  [%s]\n",
               name,
               true_pitch, res.pitch_mm, pitch_err * 100.f,
               dir_true, res.direction_deg, dir_err,
               res.confidence,
               ok ? "PASS" : "FAIL");
        return ok;
    } catch (const std::exception& e) {
        printf("  %-42s  EXCEPTION: %s\n", name, e.what());
        return false;
    }
}

int main() {
    srand(42);
    int pass = 0, total = 0;

    printf("=== synthetic_periodic autocorrelation tests ===\n\n");

    // Period vectors chosen so that cos(2pi*(Pu^2-Pv^2)/(Pu^2+Pv^2)) << 1
    // to ensure the mirror peak (Pu,-Pv) has clearly negative correlation.

    // Test 1: horizontal period – no mirror ambiguity
    ++total; if (run_test("horizontal 3.0mm  (0 deg)",
                          3.0f, 0.0f, 0.20f, 0.0f)) ++pass;

    // Test 2: oblique, ~31 deg.  Mirror: cos(2pi*(4-2.25)/6.25)=cos(1.76)=-0.19. OK.
    ++total; if (run_test("oblique (2.0, 1.5)mm  (~31 deg)",
                          2.0f, 1.5f, 0.20f, 0.0f)) ++pass;

    // Test 3: oblique with moderate noise
    ++total; if (run_test("oblique (2.5, 1.5)mm + noise=0.3",
                          2.5f, 1.5f, 0.20f, 0.3f)) ++pass;

    // Test 4: small pitch, fine grid
    ++total; if (run_test("fine pitch (1.8, 0.3)mm  (~9 deg)",
                          1.8f, 0.3f, 0.10f, 0.0f)) ++pass;

    // Test 5: steeper angle.  Mirror: cos(2pi*(2.25-9)/11.25)=cos(-3.77)=-0.81. OK.
    ++total; if (run_test("steeper (1.5, 3.0)mm  (~63 deg)",
                          1.5f, 3.0f, 0.20f, 0.0f)) ++pass;

    printf("\n%d / %d tests passed\n", pass, total);
    return (pass == total) ? 0 : 1;
}

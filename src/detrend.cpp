#include "jomon/detrend.hpp"
#include <cmath>
#include <limits>

namespace jomon {

// NaN-safe 2-D box blur.
// Returns blurred matrix; cells where no valid neighbours exist become NaN.
static Eigen::MatrixXf nan_box_blur(const Eigen::MatrixXf& H, int half) {
    const int rows = H.rows();
    const int cols = H.cols();
    constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

    // Horizontal pass
    Eigen::MatrixXf tmp(rows, cols);
    for (int r = 0; r < rows; ++r) {
        float sum = 0.f; int cnt = 0;
        for (int c = 0; c < cols; ++c) {
            // Add new element on the right
            if (c + half < cols && !std::isnan(H(r, c + half))) {
                sum += H(r, c + half); ++cnt;
            }
            // Remove element that left the window
            if (c - half - 1 >= 0 && !std::isnan(H(r, c - half - 1))) {
                sum -= H(r, c - half - 1); --cnt;
            }
            // Seed the window for the first column
            if (c == 0) {
                sum = 0.f; cnt = 0;
                for (int k = -half; k <= half; ++k) {
                    int kk = k;
                    if (kk < 0 || kk >= cols) continue;
                    if (!std::isnan(H(r, kk))) { sum += H(r, kk); ++cnt; }
                }
            }
            tmp(r, c) = (cnt > 0) ? sum / static_cast<float>(cnt) : kNaN;
        }
    }

    // Vertical pass
    Eigen::MatrixXf out(rows, cols);
    for (int c = 0; c < cols; ++c) {
        float sum = 0.f; int cnt = 0;
        for (int r = 0; r < rows; ++r) {
            if (r == 0) {
                sum = 0.f; cnt = 0;
                for (int k = -half; k <= half; ++k) {
                    if (k < 0 || k >= rows) continue;
                    if (!std::isnan(tmp(k, c))) { sum += tmp(k, c); ++cnt; }
                }
            } else {
                if (r + half < rows && !std::isnan(tmp(r + half, c))) {
                    sum += tmp(r + half, c); ++cnt;
                }
                if (r - half - 1 >= 0 && !std::isnan(tmp(r - half - 1, c))) {
                    sum -= tmp(r - half - 1, c); --cnt;
                }
            }
            out(r, c) = (cnt > 0) ? sum / static_cast<float>(cnt) : kNaN;
        }
    }
    return out;
}

void detrend(HeightMap& hmap, const Config& cfg) {
    Eigen::MatrixXf blur = nan_box_blur(hmap.H, cfg.detrend_window);

    constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
    const int rows = hmap.H.rows();
    const int cols = hmap.H.cols();

    hmap.H_detrended.resize(rows, cols);
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c) {
            if (std::isnan(hmap.H(r, c)) || std::isnan(blur(r, c)))
                hmap.H_detrended(r, c) = kNaN;
            else
                hmap.H_detrended(r, c) = hmap.H(r, c) - blur(r, c);
        }
}

} // namespace jomon

#include "jomon/autocorr2d.hpp"
#include <unsupported/Eigen/FFT>
#include <cmath>
#include <complex>
#include <limits>
#include <stdexcept>
#include <vector>

namespace jomon {

namespace {

// Next power of two >= n
static int next_pow2(int n) {
    int p = 1;
    while (p < n) p <<= 1;
    return p;
}

// 2-D forward FFT: apply 1-D FFT row-wise then column-wise
static Eigen::MatrixXcd fft2d(const Eigen::MatrixXd& in) {
    Eigen::FFT<double> fft;
    const int R = in.rows(), C = in.cols();
    Eigen::MatrixXcd mid(R, C);
    for (int r = 0; r < R; ++r) {
        Eigen::VectorXd   row_d = in.row(r);
        Eigen::VectorXcd  row_f;
        fft.fwd(row_f, row_d);
        mid.row(r) = row_f;
    }
    Eigen::MatrixXcd out(R, C);
    for (int c = 0; c < C; ++c) {
        Eigen::VectorXcd col   = mid.col(c);
        Eigen::VectorXcd col_f;
        fft.fwd(col_f, col);
        out.col(c) = col_f;
    }
    return out;
}

// 2-D inverse FFT: column-wise then row-wise; return real part
static Eigen::MatrixXd ifft2d_real(const Eigen::MatrixXcd& F) {
    Eigen::FFT<double> fft;
    const int R = F.rows(), C = F.cols();
    Eigen::MatrixXcd mid(R, C);
    for (int c = 0; c < C; ++c) {
        Eigen::VectorXcd col   = F.col(c);
        Eigen::VectorXcd col_r;
        fft.inv(col_r, col);
        mid.col(c) = col_r;
    }
    Eigen::MatrixXd out(R, C);
    for (int r = 0; r < R; ++r) {
        Eigen::VectorXcd row   = mid.row(r);
        Eigen::VectorXcd row_r;
        fft.inv(row_r, row);
        out.row(r) = row_r.real();
    }
    return out;
}

// Quadratic sub-pixel peak refinement in 1-D (3-point parabola)
static float subpixel_peak(float ym1, float y0, float yp1) {
    float denom = ym1 - 2.0f * y0 + yp1;
    if (std::abs(denom) < 1e-12f) return 0.0f;
    return 0.5f * (ym1 - yp1) / denom;
}

} // anonymous namespace

PeriodResult estimate_period(const HeightMap& hmap, const Config& cfg) {
    const auto& H = hmap.H_detrended;
    const int rows = H.rows(), cols = H.cols();

    // FFT dimensions: at least 2x to avoid circular aliasing, rounded to pow2
    const int fR = next_pow2(2 * rows);
    const int fC = next_pow2(2 * cols);

    // Build zero-padded signal H_pad and valid-mask M_pad
    Eigen::MatrixXd H_pad = Eigen::MatrixXd::Zero(fR, fC);
    Eigen::MatrixXd M_pad = Eigen::MatrixXd::Zero(fR, fC);

    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            if (!std::isnan(H(r, c))) {
                H_pad(r, c) = H(r, c);
                M_pad(r, c) = 1.0;
            }

    // Masked autocorrelation via Wiener–Khinchin:
    //   R = IFFT(|FFT(H)|^2) / IFFT(|FFT(M)|^2)
    auto FH = fft2d(H_pad);
    auto FM = fft2d(M_pad);

    Eigen::MatrixXcd PH = FH.cwiseAbs2().cast<std::complex<double>>();
    Eigen::MatrixXcd PM = FM.cwiseAbs2().cast<std::complex<double>>();

    Eigen::MatrixXd RH = ifft2d_real(PH);
    Eigen::MatrixXd RM = ifft2d_real(PM);

    // Normalise; suppress cells with too few overlapping valid pixels
    Eigen::MatrixXd R(fR, fC);
    const double min_overlap = 0.01 * static_cast<double>(rows * cols);
    for (int r = 0; r < fR; ++r)
        for (int c = 0; c < fC; ++c)
            R(r, c) = (RM(r, c) > min_overlap) ? RH(r, c) / RM(r, c) : 0.0;

    // Peak search range in grid cells
    const float p = hmap.grid_pitch;
    const int lag_min = std::max(1, static_cast<int>(std::floor(cfg.period_min_mm / p)));
    const int lag_max = static_cast<int>(std::ceil(cfg.period_max_mm / p));

    // Search over the half-plane (dc >= 0, or dc==0 and dr > 0) using
    // circular-shift indexing: positive lag l → index l; negative lag -l → index fR-l.
    double best_val = -std::numeric_limits<double>::infinity();
    int    best_dr = 0, best_dc = 1;

    for (int dr = -lag_max; dr <= lag_max; ++dr) {
        for (int dc = 0; dc <= lag_max; ++dc) {
            if (dc == 0 && dr <= 0) continue;  // skip origin and negative-only half
            float dist = std::sqrt((float)(dr * dr + dc * dc));
            if (dist < static_cast<float>(lag_min) || dist > static_cast<float>(lag_max))
                continue;

            int ri = (dr >= 0) ? dr : fR + dr;
            int ci = dc;  // dc >= 0 always in this search

            if (R(ri, ci) > best_val) {
                best_val = R(ri, ci);
                best_dr  = dr;
                best_dc  = dc;
            }
        }
    }

    // Subpixel refinement (parabolic in each dimension independently)
    auto ri = [&](int dr) { return (dr >= 0) ? dr : fR + dr; };

    float sub_dr = 0.f, sub_dc = 0.f;
    {
        int r0 = ri(best_dr), c0 = best_dc;
        int rm = ri(best_dr - 1), rp = ri(best_dr + 1);
        int cm = std::max(0, c0 - 1), cp = std::min(fC - 1, c0 + 1);
        sub_dr = subpixel_peak((float)R(rm, c0), (float)R(r0, c0), (float)R(rp, c0));
        sub_dc = subpixel_peak((float)R(r0, cm), (float)R(r0, c0), (float)R(r0, cp));
    }

    float lag_dr = static_cast<float>(best_dr) + sub_dr;
    float lag_dc = static_cast<float>(best_dc) + sub_dc;

    // Confidence: ratio of peak value to background RMS (excluding the DC peak)
    double sum_sq = 0.0; int n_bg = 0;
    for (int r = 0; r < fR; ++r)
        for (int c = 0; c < fC; ++c) {
            if (r == 0 && c == 0) continue;
            sum_sq += R(r, c) * R(r, c);
            ++n_bg;
        }
    double rms = (n_bg > 0) ? std::sqrt(sum_sq / n_bg) : 1.0;
    float confidence = (rms > 1e-12) ? static_cast<float>(best_val / (rms * 3.0)) : 0.f;
    confidence = std::min(1.0f, std::max(0.0f, confidence));

    if (best_val <= 0.0) {
        throw std::runtime_error(
            "No periodic peak found in the autocorrelation map. "
            "Try widening --period-min / --period-max or check the detrend window.");
    }

    PeriodResult result;
    result.dx_mm         = lag_dc * p;
    result.dy_mm         = lag_dr * p;
    result.pitch_mm      = std::sqrt(result.dx_mm * result.dx_mm + result.dy_mm * result.dy_mm);
    result.direction_deg = std::atan2(result.dy_mm, result.dx_mm) * 180.0f / (float)M_PI;
    result.confidence    = confidence;
    return result;
}

} // namespace jomon

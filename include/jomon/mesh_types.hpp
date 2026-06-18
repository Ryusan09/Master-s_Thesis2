#pragma once
#include <Eigen/Core>
#include <vector>

namespace jomon {

struct MeshData {
    Eigen::MatrixX3f V;  // Nx3 float vertex positions (mm)
    Eigen::MatrixX3i F;  // Mx3 int face vertex indices
};

struct HeightMap {
    Eigen::MatrixXf H;           // height above reference plane (NaN = no data)
    Eigen::MatrixXf H_detrended; // H minus low-frequency trend
    float           grid_pitch;  // mm per grid cell
    Eigen::Vector3f origin;      // world-space grid origin
    Eigen::Vector3f u_axis;      // unit vector for grid column direction
    Eigen::Vector3f v_axis;      // unit vector for grid row direction
};

struct PeriodResult {
    float pitch_mm;            // inter-groove distance (mm)
    float direction_deg;       // period vector angle from +u axis (deg); perpendicular to grooves
    float groove_direction_deg;// groove (strand) direction from +u axis, in [0, 180) deg
    float dx_mm;               // period vector x component (mm)
    float dy_mm;               // period vector y component (mm)
    float confidence;          // peak sharpness ratio [0, 1]
};

// Full pipeline output: period result + reference plane + selected vertices.
struct PipelineResult {
    PeriodResult     period;
    HeightMap        hmap;
    std::vector<int> selected;
};

} // namespace jomon

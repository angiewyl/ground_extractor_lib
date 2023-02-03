#pragma once

#include <vector>
#include <cstdint>
#include <Eigen/Dense>

namespace GroundExtraction
{

// Row Major. Usual robot coordinate frame (x positive towards the top [row 0], y positive to the right)
class Grid2D
{

enum class Label : std::uint8_t
{
    Unknown,
    Obstacle,
    Unoccupied
};

private:
    // std::vector<Label> m_grid;
    std::size_t m_cols{0};
    std::size_t m_rows{0};
    float m_resolution{0.1f};
    Eigen::Vector3f m_gridOrigin;

    
};

public: 

std::vector<Label> m_grid;

struct ExtractionSettings
{
    float map_boundaries[4];
    float m_reso;
    float zaxis_ground;
    float zaxis_ceil;
    double MSEmax;
    float plane_ground;
    float plane_offset;
    float plane_reso;
    float confidence_label;
    float confidence_zaxis;
    float confidence_plane;
    float confidence_threshold; 
};

}
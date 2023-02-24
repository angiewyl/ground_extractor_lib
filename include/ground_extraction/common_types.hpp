#pragma once

#include <array>
#include <string>
#include <vector>
#include <limits>
#include <cstdint>


namespace GroundExtraction
{

// Row Major. Usual robot coordinate frame (x positive towards the top [row 0], y positive to the right)
class Grid2D
{
public:
    enum class Labels : std::uint8_t
    {
        Unknown,
        Obstacle,
        Unoccupied
    };
    struct GridParameters
    {
        std::size_t cols;
        std::size_t rows;
        float reso;
        std::array<float,2> origin; // (x,y), this origin is placed at xmin, ymax
    };

    GridParameters m_parameters;
    std::vector<Labels> m_grid;
};


struct ExtractionSettings
{
    std::string filename;
    // map_boundaries in the form (xmin, xmax, ymin, ymax)
    std::array<float, 4> map_boundaries{std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()};
    float m_resolution{0.16};
    float zaxis_ground;
    float zaxis_ceil;
    double MSEmax;
    float plane_ground;
    float plane_offset;
    float plane_resolution{0.8}; 
    float confidence_label;
    float confidence_zaxis;
    float confidence_plane;
    float confidence_threshold; 
};


}
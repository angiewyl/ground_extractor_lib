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

enum class ExtractionSettingPreset
{
    DEFAULT,
    SLOPY_TERRAIN,
    FLAT_TERRAIN
};

struct ExtractionSettings
{
    std::string output_filename; // Should be an image output file
    // map_boundaries in the form (xmin, xmax, ymin, ymax)
    std::array<float, 4> map_boundaries{std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()};
    float m_resolution{0.16};
    float zaxis_ground{0.2};
    float zaxis_ceil{1.2f};
    float MSEmax{100};
    float plane_ground{0};
    float plane_offset{1.0f};
    float plane_resolution{1.0f}; 
    float confidence_label;
    float confidence_zaxis;
    float confidence_plane;
    float confidence_threshold;
    float m_inflation_dis;
};


}
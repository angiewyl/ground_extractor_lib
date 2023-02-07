#pragma once

#include <vector>
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

std::vector<Labels> m_grid;

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

ExtractionSettings input_param;

};



}
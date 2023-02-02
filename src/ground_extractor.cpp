#include "ground_extraction/ground_extractor.hpp"
#include "internal_utils.hpp"
#include <spdlog/spdlog.h>
#include "internal_utils.hpp"

namespace GroundExtraction
{



void grid_bounds(pcl::PointCloud<PointT> labelledCloud, size_t rows, size_t cols, float origin[2], float reso)
{
    float xmin = numeric_limits<float>::max();
    float xmax = numeric_limits<float>::lowest(); 
    float ymin = numeric_limits<float>::max();
    float ymax = numeric_limits<float>::lowest();
    for (const auto& point: *labelledCloud)
    {
        xmin = std::min(xmin, point.x);
        xmax = std::max(xmax, point.x);
        ymin = std::min(ymin, point.y);
        ymax = std::max(ymax, point.y);
    }
    
    rows = static_cast<size_t>(std::ceil((ymax-ymin)/reso));
    cols = static_cast<size_t>(std::ceil((xmax-xmin)/reso));
    origin = {xmin, ymax};
}


Grid2D Extract(const pcl::PointCloud<PointT>& labelledCloud, ExtractionSettings settings)
{
    
    
    
    
    Extractor extract(labelledCloud);
    extract.blah();

    return Grid2D();



}



}
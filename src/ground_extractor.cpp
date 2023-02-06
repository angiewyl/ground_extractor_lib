#include "ground_extraction/ground_extractor.hpp"
#include "internal_utils.hpp"
#include <spdlog/spdlog.h>
#include "internal_utils.hpp"
#include "ground_extraction/common_types.hpp"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

namespace GroundExtraction
{

typedef pcl::PointXYZL PointT;

Grid2D Extract(pcl::PointCloud<PointT>::Ptr labelledCloud, Grid2D::ExtractionSettings input, std::vector<Grid2D::Labels>& m_grid)
{
    GridParameters parameters;
      
    grid_bounds(labelledCloud, parameters, input);
    std::size_t grid_size = parameters.rows*parameters.cols;
    int *count = new int[grid_size];
    int *confidence_l = new int[grid_size];
    int *confidence_p = new int[grid_size];
    int *confidence_z = new int[grid_size];
    
    // std::array<int> count;
    // std::array<int> confidence_l;
    // std::array<int> confidence_p;
    // std::array<int, grid_size> confidence_z; 

    labels_method(labelledCloud, confidence_l, count, parameters);
    zaxis_method(labelledCloud, confidence_z, parameters, input);
    plane_method(labelledCloud, confidence_p, parameters, input);

    extract(labelledCloud, input, m_grid, count, confidence_l, confidence_p, confidence_z);
    
    dilate(m_grid, parameters);

    return Grid2D();
}



}
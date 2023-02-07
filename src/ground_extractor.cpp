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

Grid2D Extract(pcl::PointCloud<PointT>::Ptr labelled_cloud, const Grid2D::ExtractionSettings& input_param, std::vector<Grid2D::Labels>& m_grid)
{
    GridParameters grid_parameters;
      
    defineGridBounds(labelled_cloud, grid_parameters, input_param);
    std::size_t grid_size = grid_parameters.rows*grid_parameters.cols;
    int *num_points = new int[grid_size];
    int *num_obstacle_labels = new int[grid_size];
    int *num_obstacle_plane = new int[grid_size];
    int *num_obstacle_zaxis = new int[grid_size];

    LabelMethod(labelled_cloud, num_obstacle_labels, num_points, grid_parameters);
    ZaxisMethod(labelled_cloud, num_obstacle_zaxis, grid_parameters, input_param);
    PlaneMethod(labelled_cloud, num_obstacle_plane, grid_parameters, input_param);

    ConfidenceExtraction(labelled_cloud, input_param, m_grid, num_points, num_obstacle_labels, num_obstacle_plane, num_obstacle_zaxis);
    
    GridDilation(m_grid, grid_parameters);

    return Grid2D();
}



}
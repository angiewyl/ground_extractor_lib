#pragma once

#include <cstddef>
#include <vector>
// #include <limits>
// #include <Eigen/Dense>

#include "ground_extraction/ground_extractor.hpp"

#include <pcl/common/common_headers.h>
// #include "opencv2/core.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgproc.hpp"

namespace GroundExtraction
{

using PointT = pcl::PointXYZL;
struct GridParameters
{
    std::size_t cols;
    std::size_t rows;
    float reso;
    std::array<float,2> origin; // (x,y)
};

void defineGridBounds(pcl::PointCloud<PointT>::Ptr labelled_cloud, GridParameters& grid_parameters, const Grid2D::ExtractionSettings& input_param);

void LabelMethod(const pcl::PointCloud<PointT>::Ptr labelled_cloud, std::vector<std::uint8_t>& num_obstacle_labels, std::vector<std::uint8_t>& num_points, const GridParameters& grid_parameters);

void ZaxisMethod(const pcl::PointCloud<PointT>::Ptr labelled_cloud, std::vector<std::uint8_t>& num_obstacle_zaxis, const GridParameters& grid_parameters, const Grid2D::ExtractionSettings& input_param);

void PlaneMethod(const pcl::PointCloud<PointT>::Ptr labelled_cloud, std::vector<std::uint8_t>& num_obstacle_plane, const GridParameters& grid_parameters, const Grid2D::ExtractionSettings& input_param);

void ConfidenceExtraction(const pcl::PointCloud<PointT>::Ptr labelled_cloud, const Grid2D::ExtractionSettings& input_param, std::vector<Grid2D::Labels>& m_grid, const std::vector<std::uint8_t>& num_points, const std::vector<std::uint8_t>& num_obstacle_labels, const std::vector<std::uint8_t>& num_obstacle_plane, const std::vector<std::uint8_t>& num_obstacle_zaxis);

void GridDilation(std::vector<Grid2D::Labels>& m_grid, const GridParameters& grid_parameters);




// protected:
//     pcl::PointCloud<PointT>::Ptr labelled_cloud;    


}
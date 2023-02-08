#pragma once

#include <cstddef>
#include <vector>
// #include <limits>
// #include <Eigen/Dense>

#include "ground_extraction/common_types.hpp"
#include "ground_extraction/ground_extractor.hpp"

#include <pcl/common/common_headers.h>
// #include "opencv2/core.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgproc.hpp"

namespace GroundExtraction
{

using PointT = pcl::PointXYZL;

void defineGridBounds(pcl::PointCloud<PointT>::Ptr& labelled_cloud, Grid2D::GridParameters& m_parameters, const ExtractionSettings& input_param);

void LabelnZaxisMethod(pcl::PointCloud<PointT>::ConstPtr labelled_cloud, std::vector<std::uint8_t>& num_obstacle_labels, std::vector<std::uint8_t>& num_obstacle_zaxis, std::vector<std::uint8_t>& num_points, const Grid2D::GridParameters& m_parameters, const ExtractionSettings& input_param);

void PlaneMethod(pcl::PointCloud<PointT>::ConstPtr labelled_cloud, std::vector<std::uint8_t>& num_obstacle_plane, const Grid2D::GridParameters& m_parameters, const ExtractionSettings& input_param);

void ConfidenceExtraction(pcl::PointCloud<PointT>::ConstPtr labelled_cloud, const ExtractionSettings& input_param, std::vector<Grid2D::Labels>& m_grid, const std::vector<std::uint8_t>& num_points, const std::vector<std::uint8_t>& num_obstacle_labels, const std::vector<std::uint8_t>& num_obstacle_plane, const std::vector<std::uint8_t>& num_obstacle_zaxis);

void GridDilation(std::vector<Grid2D::Labels>& m_grid, const Grid2D::GridParameters& m_parameters);




// protected:
//     pcl::PointCloud<PointT>::Ptr labelled_cloud;    


}
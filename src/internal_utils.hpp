#pragma once

#include <cstddef>
#include <vector>
#include <Eigen/Dense>

namespace GroundExtraction
{

using PointT = pcl::PointXYZL;
struct GridParameters
{
    std::size_t cols;
    std::size_t rows;
    float reso;
    float origin[2]; // (x,y)
};

void defineGridBounds(pcl::PointCloud<PointT>::Ptr &labelled_cloud, GridParameters& grid_parameters, const Grid2D::ExtractionSettings& input_param);

template<class T>
void LabelMethod(const pcl::PointCloud<PointT>::Ptr labelled_cloud, T& num_obstacle_labels, T& num_points, const GridParameters& grid_parameters);

template<class T>
void ZaxisMethod(const pcl::PointCloud<PointT>::Ptr labelled_cloud, T& num_obstacle_zaxis, const GridParameters& grid_parameters, const Grid2D::ExtractionSettings& input_param);

template<class T>
void PlaneMethod(const pcl::PointCloud<PointT>::Ptr labelled_cloud, T& num_obstacle_plane, const GridParameters& grid_parameters, const Grid2D::ExtractionSettings& input_param);

template<class T>
void ConfidenceExtraction(const pcl::PointCloud<PointT>::Ptr labelled_cloud, const Grid2D::ExtractionSettings& input_param, std::vector<Grid2D::Labels>& m_grid, const T& num_points, const T& num_obstacle_labels, const T& num_obstacle_plane, const T& num_obstacle_zaxis);

void GridDilation(std::vector<Grid2D::Labels>& m_grid, const GridParameters& grid_parameters);




// protected:
//     pcl::PointCloud<PointT>::Ptr labelled_cloud;    


}
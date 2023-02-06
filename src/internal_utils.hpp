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

void grid_bounds(pcl::PointCloud<PointT>::Ptr &labelledCloud, GridParameters& parameters, const Grid2D::ExtractionSettings& input);

template<class T>
void labels_method(const pcl::PointCloud<PointT>::Ptr labelledCloud, T& confidence_label, T& count, const GridParameters& parameters);

template<class T>
void zaxis_method(const pcl::PointCloud<PointT>::Ptr labelledCloud, T& confidence_z, const GridParameters& parameters, const Grid2D::ExtractionSettings& input);

template<class T>
void plane_method(const pcl::PointCloud<PointT>::Ptr labelledCloud, T& confidence_p, const GridParameters& parameters, const Grid2D::ExtractionSettings& input);

template<class T>
void extract(const pcl::PointCloud<PointT>::Ptr labelledCloud, const Grid2D::ExtractionSettings& input, std::vector<Grid2D::Labels>& m_grid, const T& count, const T& confidence_l, const T& confidence_p, const T& confidence_z);

void dilate(std::vector<Grid2D::Labels>& m_grid, const GridParameters& parameters);




// protected:
//     pcl::PointCloud<PointT>::Ptr labelledCloud;    


}
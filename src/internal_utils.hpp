#pragma once

#include <cstddef>
#include <vector>
#include <Eigen/Dense>

namespace GroundExtraction
{

template<class DataType>
typedef pcl::PointXYZL PointT;

class Extractor
{
public:
    void grid_bounds(const pcl::PointCloud<PointT>& labelledCloud, struct Grid *grid, struct ExtractionSettings settings);

    void labels_method(const pcl::PointCloud<PointT> labelledCloud, std::array<int>& confidence_l, std::array<int>& count, struct Grid grid);

    void zaxis_method(const pcl::PointCloud<PointT> labelledCloud, std::array<int>& confidence_z, struct Grid grid, struct ExtractionSettings settings);
    
    void plane_method(const pcl::PointCloud<PointT> labelledCloud, std::array<int>& confidence_p, struct Grid grid, struct ExtractionSettings settings);

    void extract(const pcl::PointCloud<PointT> labelledCloud, ExtractionSettings settings, std::vector<Label> m_grid, std::array<int> count, std::array<int> confidence_l, std::array<int> confidence_p, std::array<int> confidence_z);

    void dilate(std::vector<Label> m_grid, Grid grid);

// protected:
//     pcl::PointCloud<PointT> labelledCloud;    

};



}
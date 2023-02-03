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
    struct Grid grid;
    void grid_bounds(const pcl::PointCloud<PointT> labelledCloud, &grid);

    void labels_method(const pcl::PointCloud<PointT> labelledCloud, std::array<int> confidence_l, std::array<int> count, Grid grid);

    void zaxis_method(const pcl::PointCloud<PointT> labelledCloud, std::array<int> confidence_z, Grid grid, ExtractionSettings settings);
    
    void plane_method(const pcl::PointCloud<PointT> labelledCloud, std::array<int> confidence_p, Grid grid, ExtractionSettings settings);

    void extract(const pcl::PointCloud<PointT> labelledCloud, ExtractionSettings settings, std::vector<Label> m_grid, std::array<int> count, std::array<int> confidence_l, std::array<int> confidence_p, std::array<int> confidence_z);

    void dilate(std::vector<Label> m_grid, Grid grid);

// protected:
//     pcl::PointCloud<PointT> labelledCloud;    

private:
    std::array<int, grid.rows*grid.cols> count;
    std::array<int, grid.rows*grid.cols> confidence_l;
    std::array<int, grid.rows*grid.cols> confidence_z;
    std::array<int, grid.rows*grid.cols> confidence_p;
    
    struct Grid
    {
        std::size_t cols{0};
        std::size_t rows{0};
        float reso{}
        float origin[2]; // (x,y)
    }
public:
};



}
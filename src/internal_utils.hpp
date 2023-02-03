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
    void grid_bounds(labelledCloud, &grid);

    void labels_method(labelledCloud, confidence_l, count, grid);

    void zaxis_method(labelledCloud, confidence_z, grid, settings);
    
    void plane_method(labelledCloud, confidence_p, grid, settings);

    void extract(labelledCloud, settings, m_grid, count, confidence_l, confidence_p, confidence_z);

protected:
    pcl::PointCloud<PointT> labelledCloud;
    

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
};



}
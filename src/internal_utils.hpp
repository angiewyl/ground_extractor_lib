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
    void grid_bounds(labelledCloud, &Grid);

    void labels_method(labelledCloud, confidence_l, count, Grid);

    void zaxis_method(labelledCloud, confidence_z, Grid, zaxis_ground, zaxis_ceil);
    
    void plane_method(labelledCloud, confidence_p, Grid, plane_reso, MSEmax, plane_ground, plane_offset);


protected:
    pcl::PointCloud<PointT> labelledCloud;
    std::array<int, grid.rows*grid.cols> count;
    std::array<int, grid.rows*grid.cols> confidence_l;
    std::array<int, grid.rows*grid.cols> confidence_z;
    std::array<int, grid.rows*grid.cols> confidence_p;
    struct Grid
    {
        std::size_t cols{0};
        std::size_t rows{0};
        float reso{0.1f};
        float origin[2]; // (x,y)
    };
};



}
#pragma once

#include <cstddef>
#include <vector>
#include <Eigen/Dense>

namespace GroundExtraction
{

typedef pcl::PointXYZL PointT;


void grid_bounds(const pcl::PointCloud<PointT>& labelledCloud, struct Grid *grid, struct ExtractionSettings settings);

void labels_method(const pcl::PointCloud<PointT> labelledCloud, int *confidence_l[], int *count[], struct Grid grid);

void zaxis_method(const pcl::PointCloud<PointT> labelledCloud, int *confidence_z[], struct Grid grid, struct ExtractionSettings settings);

void plane_method(const pcl::PointCloud<PointT> labelledCloud, int *confidence_p[], struct Grid grid, struct ExtractionSettings settings);

void extract(const pcl::PointCloud<PointT> labelledCloud, ExtractionSettings settings, std::vector<Label> m_grid, int count[], int confidence_l[], int confidence_p[], int confidence_z[]);

void dilate(std::vector<Label> m_grid, Grid grid);

// protected:
//     pcl::PointCloud<PointT> labelledCloud;    


}
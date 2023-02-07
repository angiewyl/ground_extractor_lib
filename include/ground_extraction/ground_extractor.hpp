#pragma once

#include <pcl/common/common_headers.h>
#include <opencv2/core.hpp>
#include "ground_extraction/common_types.hpp"

namespace GroundExtraction 
{

using PointT = pcl::PointXYZL;

Grid2D Extract(pcl::PointCloud<PointT>::Ptr labelled_cloud, const Grid2D::ExtractionSettings& input_param, std::vector<Grid2D::Labels>& m_grid);







}
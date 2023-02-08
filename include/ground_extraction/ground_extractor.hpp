#pragma once

#include <pcl/common/common_headers.h>
#include <opencv2/core.hpp>
#include "ground_extraction/common_types.hpp"

namespace GroundExtraction 
{

using PointT = pcl::PointXYZL;

// We modify the provided cloud for performance reasons.
Grid2D Extract(pcl::PointCloud<PointT>::Ptr labelled_cloud, const ExtractionSettings& input_param);







}
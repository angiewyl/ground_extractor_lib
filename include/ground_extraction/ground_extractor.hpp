#pragma once

#include <pcl/common/common.h>
#include <opencv/core.hpp>

#include "ground_extraction/ground_extractor.hpp"

#include "ground_extraction/common_types.hpp"

namespace GroundExtraction 
{

using PointT = pcl::PointXYZL;



Grid2D Extract(const pcl::PointCloud<PointT>& labelledCloud, ExtractionSettings settings);






}
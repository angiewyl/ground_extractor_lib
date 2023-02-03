#include "ground_extraction/ground_extractor.hpp"
#include "internal_utils.hpp"
#include <spdlog/spdlog.h>
#include "internal_utils.hpp"
#include "ground_extraction/common_types.hpp"

namespace GroundExtraction
{

typedef pcl::PointXYZL PointT;

Grid2D Extract(const pcl::PointCloud<PointT> labelledCloud, ExtractionSettings settings, std::vector<Label>& m_grid)
{
    template<class DataType>   
    void Extractor<DataType>::extract(const pcl::PointCloud<PointT> labelledCloud, ExtractionSettings settings, std::vector<Label>& m_grid, std::array<int> count, std::array<int> confidence_l, std::array<int> confidence_p, std::array<int> confidence_z)
    {

    }
    return Grid2D();
}



}
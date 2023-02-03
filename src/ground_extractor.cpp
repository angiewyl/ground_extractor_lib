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
        for (size_t i=0; i<sizeof(count); i++)
        {
            float confidence = (settings.confidence_label*confidence_l[i] + settings.confidence_zaxis*confidence_z[i] + settings.confidence_plane*confidence_p[i])/(settings.confidence_label + settings.confidence_plane + settings.confidence_zaxis);
            
            if (count[i] == 0)
            {
                m_grid.push_back(Unknown);
            }
            else if (confidence <= settings.confidence_threshold)
            {
                m_grid.push_back(Unoccupied);
            }
            else 
            {
                m_grid.push_back(Obstacle);
            }
        }
    }
    return Grid2D();
}



}
#include "ground_extraction/ground_extractor.hpp"
#include "internal_utils.hpp"
#include <spdlog/spdlog.h>
#include "internal_utils.hpp"
#include "ground_extraction/common_types.hpp"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

namespace GroundExtraction
{

typedef pcl::PointXYZL PointT;
struct Grid
{
    std::size_t cols;
    std::size_t rows;
    float reso;
    float origin[2]; // (x,y)
}

Grid2D Extract(const pcl::PointCloud<PointT> labelledCloud, struct ExtractionSettings settings, std::vector<Label>& m_grid)
{
    struct Grid grid;
      
    grid_bounds(labelledCloud, &grid, settings);
    const std::size_t grid_size = grid.rows*grid.cols;
    
    int count[grid_size];
    int confidence_l[grid_size];
    int confidence_z[grid_size];
    int confidence_p[grid_size];

    labels_method(labelledCloud, &confidence_l, &count, grid);
    zaxis_method(labelledCloud, &confidence_z, grid, settings);
    plane_method(labelledCloud, &confidence_p, grid, settings);
    
    void extract(const pcl::PointCloud<PointT> labelledCloud, struct ExtractionSettings settings, std::vector<Label>& m_grid, int count[], int confidence_l[], int confidence_p[], int confidence_z[])
    {
        for (std::size_t i=0; i<sizeof(count); i++)
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
        return;
    }

    extract(labelledCloud, settings, m_grid, count, confidence_l, confidence_p, confidence_z)
    
    void dilate(std::vector<Label>& m_grid, struct Grid grid)
    {
        cv::Mat src = cv::Mat(static_cast<int>(grid.rows), static_cast<int>(grid.cols), CV_8U, m_grid, AUTO_STEP);
        cv::Mat opening_dst;
        cv::morphologyEx(src, opening_dst, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(2,2)));
        src = opening_dst;
        return;
    } 

    dilate(m_grid, grid)

    return Grid2D();
}



}
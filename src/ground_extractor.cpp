#include "ground_extraction/ground_extractor.hpp"
#include "internal_utils.hpp"
#include <spdlog/spdlog.h>
#include <vector>
#include "internal_utils.hpp"
#include "ground_extraction/common_types.hpp"
#include <chrono>

namespace GroundExtraction
{

typedef pcl::PointXYZL PointT;

ExtractionSettings GenerateSettingsFromPreset(ExtractionSettingPreset preset)
{
    ExtractionSettings input_param;
    return input_param;
}

Grid2D Extract(pcl::PointCloud<PointT>::Ptr labelled_cloud, const ExtractionSettings& input_param)
{      
    OutlierRemoval(labelled_cloud);
    Grid2D gridOut;
#if defined(WITH_PROFILING_PRINTOUTS)    
    const auto start = std::chrono::high_resolution_clock::now();
#endif
    defineGridBounds(labelled_cloud, gridOut.m_parameters, input_param);
#if defined(WITH_PROFILING_PRINTOUTS)
    std::cout << "time taken (DEFINE BOUNDS): " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() << "micros." << std::endl;

    const auto labelnzaxis = std::chrono::high_resolution_clock::now();
#endif
    std::size_t grid_size = gridOut.m_parameters.rows*gridOut.m_parameters.cols;
    
    std::vector<std::uint8_t> num_obstacle_labels(grid_size);
    std::vector<std::uint8_t> num_obstacle_zaxis(grid_size);
    std::vector<std::uint8_t> num_obstacle_plane(grid_size);
    std::vector<std::uint8_t> num_points(grid_size);

    LabelnZaxisMethod(labelled_cloud, num_obstacle_labels, num_obstacle_zaxis, num_points, gridOut.m_parameters, input_param);
#if defined(WITH_PROFILING_PRINTOUTS)
    std::cout << "time taken (LABEL & ZAXIS): " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - labelnzaxis).count() << "micros." << std::endl;
#endif

    PlaneMethod(labelled_cloud, num_obstacle_plane, num_points, gridOut.m_parameters, input_param);
#if defined(WITH_PROFILING_PRINTOUTS)
    const auto confidence = std::chrono::high_resolution_clock::now();
#endif
    ConfidenceExtraction(labelled_cloud, input_param, gridOut.m_grid, num_points, num_obstacle_labels, num_obstacle_plane, num_obstacle_zaxis);
#if defined(WITH_PROFILING_PRINTOUTS)    
    std::cout << "time taken (CONFIDENCE): " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - confidence).count() << "micros." << std::endl;
#endif
    GridDilation(gridOut.m_grid, gridOut.m_parameters);
    ExportPNG(gridOut.m_grid, gridOut.m_parameters, input_param.output_filename);

    return gridOut;
}



}
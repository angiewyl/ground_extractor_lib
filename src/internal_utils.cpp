#include "internal_utils.hpp"
#include <cmath>
#include <cstddef>
#include <pcl/features/feature.h>

namespace GroundExtraction
{

using PointT = pcl::PointXYZL;


std::size_t findGridPosition(float x, float y, const std::array<float,2> origin, float reso, int cols)
{
    int cellx = static_cast<int>(floor((x-origin[0])/reso));
    int celly = static_cast<int>(floor((origin[1]-y)/reso));
    
    return (static_cast<std::size_t>(celly*cols + cellx));
}

void planeFitting(const std::vector<std::array<float, 3>>& ground_points, std::array<float,3>& plane_coefficients)
{
    float XXsum = 0; float XYsum = 0; float YYsum = 0; float XZsum = 0; float YZsum = 0;
    float Xavg = 0; float Yavg = 0; float Zavg = 0;
    int count = 0;
    for (int i=0; i<ground_points.size(); i++)
    {
        Xavg += ground_points[i][0];
        Yavg += ground_points[i][1];
        Zavg += ground_points[i][2];
        count ++;
    }
    Xavg /= count; Yavg /= count; Zavg /= count;
    for (int i=0; i<ground_points.size(); i++)
    {
        float x = ground_points[i][0];
        float y = ground_points[i][1];
        float z = ground_points[i][2];
        XXsum += ((x - Xavg)*(x - Xavg));
        XYsum += ((x - Xavg)*(y - Yavg));
        YYsum += ((y - Yavg)*(y - Yavg));

        XZsum += ((z - Zavg)*(x - Xavg));
        YZsum += ((z - Zavg)*(y - Yavg));
    }
    plane_coefficients[0] = (YYsum*XZsum - XYsum*YZsum)/(XXsum*YYsum - XYsum*XYsum);
    plane_coefficients[1] = (XXsum*YZsum - XYsum*XZsum)/(XXsum*YYsum - XYsum*XYsum);
    plane_coefficients[2] = Zavg - plane_coefficients[0]*Xavg - plane_coefficients[1]*Yavg;
    return;
}
void calculateMSE(const std::vector<std::array<float,3>>& ground_points, const std::array<float,3>& plane_coefficients, float& mean_squared_errors)
{
    mean_squared_errors = 0;
    for (int i=0; i<ground_points.size(); i++)
    {
        mean_squared_errors += (plane_coefficients[0]*ground_points[i][0] + plane_coefficients[1]*ground_points[i][1] + plane_coefficients[2] - ground_points[i][2])*(plane_coefficients[0]*ground_points[i][0] + plane_coefficients[1]*ground_points[i][1] + plane_coefficients[2] - ground_points[i][2]);
    }
    mean_squared_errors /= ground_points.size();
    return;
}

// If you're modifying, take a reference
void defineGridBounds(pcl::PointCloud<PointT>::Ptr& labelled_cloud, Grid2D::GridParameters& m_parameters, const ExtractionSettings& input_param)
{
    float x_min = std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::lowest(); 
    float y_min = std::numeric_limits<float>::max();
    float y_max = std::numeric_limits<float>::lowest();
    if (input_param.map_boundaries[0] != x_min || input_param.map_boundaries[1] != x_max || input_param.map_boundaries[2] != y_min || input_param.map_boundaries[3] != y_max)
    {
        x_min = input_param.map_boundaries[0];
        x_max = input_param.map_boundaries[1];
        y_min = input_param.map_boundaries[2];
        y_max = input_param.map_boundaries[3];

        pcl::PointCloud<PointT>::Ptr new_map (new pcl::PointCloud<PointT>);
        for (const auto& point: *labelled_cloud) 
        {
            if (point.x>x_min && point.x<x_max && point.y>y_min && point.y<y_max) 
            {
                new_map->points.push_back(point);
            }
        }
        labelled_cloud = new_map;
    }
    else
    { 
        for (const auto& point: *labelled_cloud)
        {
            x_min = std::min(x_min, point.x);
            x_max = std::max(x_max, point.x);
            y_min = std::min(y_min, point.y);
            y_max = std::max(y_max, point.y);
        }
    }
    m_parameters.reso = input_param.m_resolution;
    m_parameters.rows = static_cast<std::size_t>(std::ceil((y_max-y_min)/(input_param.m_resolution)));
    m_parameters.cols = static_cast<std::size_t>(std::ceil((x_max-x_min)/(input_param.m_resolution)));
    m_parameters.origin = {x_min, y_max};
    return;
}

void LabelnZaxisMethod(pcl::PointCloud<PointT>::ConstPtr labelled_cloud, std::vector<std::uint8_t>& num_obstacle_labels, std::vector<std::uint8_t>& num_obstacle_zaxis, std::vector<std::uint8_t>& num_points, const Grid2D::GridParameters& m_parameters, const ExtractionSettings& input_param)
{
    for (const auto& point: *labelled_cloud)
    {
        std::size_t position = findGridPosition(point.x, point.y, m_parameters.origin, m_parameters.reso, m_parameters.cols);
        num_points[position] += 1;
        if (point.label == 0)
        {
            num_obstacle_labels[position] += 1;
        }
        if (point.z>input_param.zaxis_ground && point.z < input_param.zaxis_ceil)
        {
            num_obstacle_zaxis[position] += 1;
        }
    }
    return;
}

struct PlaneParameters
{
    std::vector<std::array<float,3>> grid_points;
    std::vector<std::array<float,3>> ground_points;
    std::array<float,3> plane_coefficients;
    float mean_squared_errors;
};

struct GridPlane
{
    std::vector<std::array<float,7>> all_points;
};

void PlaneMethod(pcl::PointCloud<PointT>::ConstPtr labelled_cloud, std::vector<std::uint8_t>& num_obstacle_plane, const Grid2D::GridParameters& m_parameters, const ExtractionSettings& input_param)
{
#if defined(WITH_PROFILING_PRINTOUTS)    
    const auto start = std::chrono::high_resolution_clock::now();    
#endif    
    // x y z a b c MSE
    std::vector<std::array<float, 7>> all_points;
    // std::array<float, 4> plane_param; // TODO: change everything to float
    std::size_t plane_rows = static_cast<std::size_t>(std::ceil(m_parameters.rows*m_parameters.reso/input_param.plane_resolution));
    std::size_t plane_cols = static_cast<std::size_t>(std::ceil(m_parameters.cols*m_parameters.reso/input_param.plane_resolution));
    std::vector<PlaneParameters> plane_grid (plane_cols*plane_rows);
    std::vector<GridPlane> grid (m_parameters.rows*m_parameters.cols);
    for (const auto& point: *labelled_cloud)
    {
        std::size_t plane_position = findGridPosition(point.x, point.y, m_parameters.origin, input_param.plane_resolution, plane_cols);
        plane_grid[plane_position].grid_points.push_back({point.x, point.y, point.z});
        if (point.label != 0)
        {
            plane_grid[plane_position].ground_points.push_back({point.x, point.y, point.z});
        }
    }

    // #pragma omp parallel for num_threads(4)
    for (std::size_t i=0; i<plane_rows*plane_cols; i++)
    {
        planeFitting(plane_grid[i].ground_points, plane_grid[i].plane_coefficients);
        calculateMSE(plane_grid[i].ground_points, plane_grid[i].plane_coefficients, plane_grid[i].mean_squared_errors);
        
        for (int k=0; k<plane_grid[i].grid_points.size(); k++)
        {
            float x = plane_grid[i].grid_points[k][0];
            float y = plane_grid[i].grid_points[k][1];
            float z = plane_grid[i].grid_points[k][2];
            std::size_t position = findGridPosition(x, y, m_parameters.origin, m_parameters.reso, m_parameters.cols);
            grid[position].all_points.push_back({plane_grid[i].grid_points[k][0], plane_grid[i].grid_points[k][1], plane_grid[i].grid_points[k][2], plane_grid[i].plane_coefficients[0], plane_grid[i].plane_coefficients[1], plane_grid[i].plane_coefficients[2], plane_grid[i].mean_squared_errors}) ;      
        }
    }

#if defined(WITH_PROFILING_PRINTOUTS)
    std::cout << "time taken (PLANEFIT): " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() << "micros." << std::endl;
    const auto plane = std::chrono::high_resolution_clock::now();
#endif
    
    for (std::size_t i=0; i<num_obstacle_plane.size(); i++)
    {
        for (int k=0; k<all_points.size(); k++)
        {
            float x = grid[i].all_points[k][0];
            float y = grid[i].all_points[k][1];
            float z = grid[i].all_points[k][2];
            float a = grid[i].all_points[k][3];
            float b = grid[i].all_points[k][4];
            float c = grid[i].all_points[k][5];
            float MSE = grid[i].all_points[k][6];
            float point_dist = ((a*x + b*y - z + c)/sqrt(a*a + b*b + 1));
            if (point_dist > input_param.plane_ground && point_dist < (input_param.plane_ground + input_param.plane_offset) && MSE < input_param.MSEmax)
            {
                num_obstacle_plane[i] += 1;
            }
        
        }
    }
#if defined(WITH_PROFILING_PRINTOUTS)    
    std::cout << "time taken (PLANE): " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - plane).count() << "micros." << std::endl;
#endif
}
void ConfidenceExtraction(pcl::PointCloud<PointT>::ConstPtr labelled_cloud, const ExtractionSettings& input_param, std::vector<Grid2D::Labels>& m_grid, const std::vector<std::uint8_t>& num_points, const std::vector<std::uint8_t>& num_obstacle_labels, const std::vector<std::uint8_t>& num_obstacle_plane, const std::vector<std::uint8_t>& num_obstacle_zaxis)
{
    for (int i=0; i<static_cast<int>(num_points.size()); i++)
    {
        float confidence = (input_param.confidence_label*num_obstacle_labels[i] + input_param.confidence_zaxis*num_obstacle_zaxis[i] + input_param.confidence_plane*num_obstacle_plane[i])/(input_param.confidence_label + input_param.confidence_plane + input_param.confidence_zaxis);

        if (num_points[i] == 0)
        {
            m_grid.push_back(Grid2D::Labels::Unknown);
        }
        else if (confidence <= input_param.confidence_threshold)
        {
            m_grid.push_back(Grid2D::Labels::Unoccupied);
        }
        else 
        {
            m_grid.push_back(Grid2D::Labels::Obstacle);
        }
    }
    return;
}

void GridDilation(std::vector<Grid2D::Labels>& m_grid, const Grid2D::GridParameters& m_parameters)
{ 
    cv::Mat src(static_cast<int>(m_parameters.rows), static_cast<int>(m_parameters.cols), CV_8U, m_grid.data(), cv::Mat::AUTO_STEP);
    cv::Mat opening_dst;
    cv::morphologyEx(src, opening_dst, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2,2)));
    src = opening_dst;
    return;
}

void ExportPNG(const std::vector<Grid2D::Labels>& m_grid, const Grid2D::GridParameters& m_parameters, const std::string& filename)
{ 
    cv::Mat img(static_cast<int>(m_parameters.rows), static_cast<int>(m_parameters.cols), CV_8U, Scalar(200));
    
    for (std::size_t i=0; i<m_grid.size(); i++)
    {
        int row_num = static_cast<int>(floor(i/m_parameters.cols));
        int col_num = static_cast<int>(i- y*m_parameters.cols);
        if (m_grid[i] == Grid2D::Labels::Unoccupied)
        {
            img.at<uchar>(row_num, col_num) = 255;
        }
        
        if (m_grid[i] == Grid2D::Labels::Unknown)
        {
            img.at<uchar>(row_num, col_num) = 200;
        }
        
        if (m_grid[i] == Grid2D::Labels::Obstacle)
        {
            img.at<uchar>(row_num, col_num) = 0;
        }
    }
    imwrite(filename + ".png", img);
    return;
}

}


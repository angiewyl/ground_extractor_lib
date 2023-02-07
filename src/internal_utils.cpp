#include "internal_utils.hpp"
#include <cmath>
#include <cstddef>

namespace GroundExtraction
{

using PointT = pcl::PointXYZL;


std::size_t findGridPosition(float x, float y, std::array<float,2> origin, float reso, int cols)
{
    return (static_cast<std::size_t>(static_cast<std::size_t>((origin[1]-y)/reso))*cols + static_cast<std::size_t>((x-origin[0])/reso));
}

void planeFitting(const std::vector<std::array<float,3>& ground_points, double& plane_param[4])
{
    double XXsum = 0; double XYsum = 0; double YYsum = 0; double XZsum = 0; double YZsum = 0;
    double Xavg = 0; double Yavg = 0; double Zavg = 0;
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
    plane_param[0] = (YYsum*XZsum - XYsum*YZsum)/(XXsum*YYsum - XYsum*XYsum);
    plane_param[1] = (XXsum*YZsum - XYsum*XZsum)/(XXsum*YYsum - XYsum*XYsum);
    plane_param[2] = Zavg - plane_param[0]*Xavg - plane_param[1]*Yavg;
    return;
}
void calculateMSE(const std::vector<std::array<float,3>& ground_points, double& plane_param[4])
{
    for (int i=0; i<ground_points.size(); i++)
    {
        plane_param[3] += (plane_param[0]*ground_points[0] + plane_param[1]*ground_points[1] + plane_param[2] - ground_points[2])*(plane_param[0]*ground_points[0] + plane_param[1]*ground_points[1] + plane_param[2] - ground_points[2]);
    }
    plane_param[3] /= xyz.size();
    return;
}
std::vector<std::array<float, 3>> allGridPoints(const pcl::PointCloud<PointT>::Ptr labelled_cloud, std::size_t position, double& plane_param[4], std::array<float,2> origin, float plane_reso, std::size_t plane_cols)
{
    std::vector<std::array<float, 3>> grid_points;
    std::vector<std::array<float, 3>> ground_points;
    
    for (const auto& point: *labelled_cloud)
    {
        if (findGridPosition(point.x, point.y, origin, plane_reso, plane_cols) == position)
        {
            grid_points.push_back({point.x, point.y, point.z});
            if (point.label != 0)
            {
                ground_points.push_back({point.x, point.y, point.z});
            }
        }
    }
    planeFitting(ground_points, plane_param);
    calculateMSE(ground_points, plane_param);
    return grid_points;
}

void defineGridBounds(pcl::PointCloud<PointT>::Ptr &labelled_cloud, GridParameters& grid_parameters, const Grid2D::ExtractionSettings& input_param)
{
    if (input_param.map_boundaries != {0,0,0,0})
    {
        float x_min = input_param.map_boundaries[0];
        float x_max = input_param.map_boundaries[1];
        float y_min = input_param.map_boundaries[2];
        float y_max = input_param.map_boundaries[3];

        pcl::PointCloud<PointT> new_map (new pcl::PointCloud<PointT>);
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
        float x_min = numeric_limits<float>::max();
        float x_max = numeric_limits<float>::lowest(); 
        float y_min = numeric_limits<float>::max();
        float y_max = numeric_limits<float>::lowest();
        for (const auto& point: *labelled_cloud)
        {
            x_min = std::min(x_min, point.x);
            x_max = std::max(x_max, point.x);
            y_min = std::min(y_min, point.y);
            y_max = std::max(y_max, point.y);
        }
    }
    grid_parameters.reso = input_param.m_reso;
    grid_parameters.rows = static_cast<std::size_t>(std::ceil((y_max-y_min)/(input_param.m_reso)));
    grid_parameters.cols = static_cast<std::size_t>(std::ceil((x_max-x_min)/(input_param.m_reso)));
    grid_parameters.origin = {x_min, y_max};
    return;
}

template<class T>
void LabelMethod(const pcl::PointCloud<PointT>::Ptr labelled_cloud, T& num_obstacle_labels, T& num_points, const GridParameters& grid_parameters)
{
    for (const auto& point: *labelled_cloud)
    {
        std::size_t position = findGridPosition(point.x, point.y, grid_parameters.origin, grid_parameters.reso, grid_parameters.cols);
        
        num_points[position] += 1;
        if (point.label == 0)
        {
            num_obstacle_labels[position] += 1;
        }
    }
    return;
}
void ZaxisMethod(const pcl::PointCloud<PointT>::Ptr labelled_cloud, T& num_obstacle_zaxis, const GridParameters& grid_parameters, const Grid2D::ExtractionSettings& input_param)
{
    for (const auto& point: *labelled_cloud)
    {
        std::size_t position = findGridPosition(point.x, point.y, grid_parameters.origin, grid_parameters.reso, grid_parameters.cols);

        if (point.z>input_param.zaxis_ground && point.z < input_param.zaxis_ceil)
        {
            num_obstacle_zaxis[position] += 1;
        }
    }

    return;
}
void PlaneMethod(const pcl::PointCloud<PointT>::Ptr labelled_cloud, T& num_obstacle_plane, const GridParameters& grid_parameters, const Grid2D::ExtractionSettings& input_param)
{
    std::vector<std::array<double,7>> all_points;
    std::array<double, 4> plane_param = {0,0,0,0};
    std::size_t plane_rows = static_cast<std::size_t>(grid_parameters.rows*grid_parameters.reso/input_param.plane_reso);
    std::size_t plane_cols = static_cast<std::size_t>(grid_parameters.cols*grid_parameters.reso/input_param.plane_reso);

    for (std::size_t i=0; i<plane_rows*plane_cols)
    {
        std::vector<std::array<float, 3>> scaled_xyz = allGridPoints(labelled_cloud, i, plane_param, grid_parameters.origin, input_param.plane_reso, plane_cols);
        for (int k=0; k<scaled_xyz.size(); k++)
        {
            all_points.push_back({scaled_xyz[k][0], scaled_xyz[k][1], scaled_xyz[k][2], plane_param[0], plane_param[1], plane_param[2], plane_param[3]})       
        }
    }
    for (std::size_t i=0; i<grid_parameters.rows*grid_parameters.cols; i++)
    {
        for (int k=0; k<all_points.size(); k++)
        {
            double x = all_points[k][0];
            double y = all_points[k][1];
            if (findGridPosition(x, y, grid_parameters.origin, grid_parameters.reso, grid_parameters.cols) == i)
            {
                double z = all_points[k][2];
                double a = all_points[k][3];
                double b = all_points[k][4];
                double c = all_points[k][5];
                double MSE = all_points[k][6];
                double point_dist = ((a*x + b*y - z + c)/sqrt(a*a + b*b + 1));
                if (point_dist > input_param.plane_ground && point_dist < (input_param.plane_ground + input_param.plane_offset) && MSE < input_param.MSEmax)
                {
                    num_obstacle_plane[i] += 1;
                }
            }
        }
    }
}

void ConfidenceExtraction(const pcl::PointCloud<PointT>::Ptr labelled_cloud, const Grid2D::ExtractionSettings& input_param, std::vector<Grid2D::Labels>& m_grid, const T& num_points, const T& num_obstacle_labels, const T& num_obstacle_zaxis, const T& num_obstacle_plane)
{
    for (std::size_t i=0; i<sizeof(num_points); i++)
    {
        float confidence = (input_param.confidence_label*num_obstacle_labels[i] + input_param.confidence_zaxis*num_obstacle_zaxis[i] + input_param.confidence_plane*num_obstacle_plane[i])/(input_param.confidence_label + input_param.confidence_plane + input_param.confidence_zaxis);

        if (num_points[i] == 0)
        {
            m_grid.push_back(Unknown);
        }
        else if (confidence <= input_param.confidence_threshold)
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
void GridDilation(std::vector<Grid2D::Labels>& m_grid, const GridParameters& grid_parameters)
{
    cv::Mat src = cv::Mat(static_cast<int>(grid_parameters.rows), static_cast<int>(grid_parameters.cols), CV_8U, m_grid, AUTO_STEP);
    cv::Mat opening_dst;
    cv::morphologyEx(src, opening_dst, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(2,2)));
    src = opening_dst;
    return;
}

}


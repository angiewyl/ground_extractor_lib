#include "internal_utils.hpp"
#include <cmath>
#include <cstddef>

namespace GroundExtraction
{

using PointT = pcl::PointXYZL;


std::size_t grid_position(float x, float y, std::array<float,2> origin, float reso, int cols)
{
    return (static_cast<std::size_t>((origin[1]-y)/reso)*cols + static_cast<std::size_t>((x-origin[0])/reso));
}

void plane_fitting(const std::vector<std::array<float,3>& xyz, double& plane_param[4])
{
    double XXsum = 0; double XYsum = 0; double YYsum = 0; double XZsum = 0; double YZsum = 0;
    double Xavg = 0; double Yavg = 0; double Zavg = 0;
    int count = 0;
    for (int i=0; i<xyz.size(); i++)
    {
        Xavg += xyz[i][0];
        Yavg += xyz[i][1];
        Zavg += xyz[i][2];
        count ++;
    }
    Xavg /= count; Yavg /= count; Zavg /= count;
    for (int i=0; i<xyz.size(); i++)
    {
        float x = xyz[i][0];
        float y = xyz[i][1];
        float z = xyz[i][2];
        XXsum += ((x - Xavg)*(x - Xavg));
        XYsum += ((x - Xavg)*(y - Yavg));
        YYsum += ((y - Yavg)*(y - Yavg));

        XZsum += ((z - Zavg)*(x - Xavg));
        YZsum += ((z - Zavg)*(y - Yavg));
    }
    plane_param[0] = (YYsum*XZsum - XYsum*YZsum)/(XXsum*YYsum - XYsum*XYsum);
    plane_param[1] = (XXsum*YZsum - XYsum*XZsum)/(XXsum*YYsum - XYsum*XYsum);
    plane_param[1] = Zavg - a*Xavg - b*Yavg;
    return;
}

void mean_squared_errors(const std::vector<std::array<float,3>& xyz, double& plane_param[4])
{
    for (int i=0; i<xyz.size(); i++)
    {
        plane_param[3] += (plane_param[0]*xyz[0] + plane_param[1]*xyz[1] + plane_param[2] - xyz[2])*(plane_param[0]*xyz[0] + plane_param[1]*xyz[1] + plane_param[2] - xyz[2]);
    }
    plane_param[3] /= xyz.size();
    return;
}

std::vector<std::array<float, 3>> all_grid_points(const pcl::PointCloud<PointT>::Ptr labelledCloud, std::size_t position, double& plane_param[4], std::array<float,2> origin, float plane_reso, std::size_t plane_cols)
{
    std::vector<std::array<float, 3>> grid_points;
    std::vector<std::array<float, 3>> ground_points;
    
    for (const auto& point: *labelledCloud)
    {
        if (grid_position(point.x, point.y, origin, plane_reso, plane_cols) == position)
        {
            grid_points.push_back({point.x, point.y, point.z});
            if (point.label != 0)
            {
                ground_points.push_back({point.x, point.y, point.z});
            }
        }
    }
    plane_fitting(ground_points, plane_param);
    mean_squared_errors(ground_points, plane_param);
    return grid_points;
}



void grid_bounds(pcl::PointCloud<PointT>::Ptr &labelledCloud, GridParameters& parameters, const Grid2D::ExtractionSettings& input)
{
    if (input.map_boundaries != {0,0,0,0})
    {
        float xmin = input.map_boundaries[0];
        float xmax = input.map_boundaries[1];
        float ymin = input.map_boundaries[2];
        float ymax = input.map_boundaries[3];

        pcl::PointCloud<PointT> newmap (new pcl::PointCloud<PointT>);
        for (const auto& point: *labelledCloud) 
        {
            if (point.x>xmin && point.x<xmax && point.y>ymin && point.y<ymax) 
            {
                newmap->points.push_back(point);
            }
        }
        labelledCloud = newmap;
    }
    else
    { 
        float xmin = numeric_limits<float>::max();
        float xmax = numeric_limits<float>::lowest(); 
        float ymin = numeric_limits<float>::max();
        float ymax = numeric_limits<float>::lowest();
        for (const auto& point: *labelledCloud)
        {
            xmin = std::min(xmin, point.x);
            xmax = std::max(xmax, point.x);
            ymin = std::min(ymin, point.y);
            ymax = std::max(ymax, point.y);
        }
    }
    parameters.reso = input.m_reso;
    parameters.rows = static_cast<std::size_t>(std::ceil((ymax-ymin)/(input.m_reso)));
    parameters.cols = static_cast<std::size_t>(std::ceil((xmax-xmin)/(input.m_reso)));
    parameters.origin = {xmin, ymax};
}

template<class T>
void labels_method(const pcl::PointCloud<PointT>::Ptr labelledCloud, T& confidence_label, T& count, const GridParameters& parameters)
{
    for (const auto& point: *labelledCloud)
    {
        std::size_t position = grid_position(point.x, point.y, parameters.origin, parameters.reso, parameters.cols);
        
        *count[position] += 1;
        if (point.label == 0)
        {
            *confidence_l[position] += 1;
        }
    }
    return;
}

void zaxis_method(const pcl::PointCloud<PointT>::Ptr labelledCloud, T& confidence_z, const GridParameters& parameters, const Grid2D::ExtractionSettings& input)
{
    for (const auto& point: *labelledCloud)
    {
        std::size_t position = grid_position(point.x, point.y, parameters.origin, parameters.reso, parameters.cols);

        if (point.z>input.zaxis_ground && point.z < input.zaxis_ceil)
        {
            *confidence_z[position] += 1;
        }
    }

    return;
}


void plane_method(const pcl::PointCloud<PointT>::Ptr labelledCloud, T& confidence_p, const GridParameters& parameters, const Grid2D::ExtractionSettings& input)
{
    std::vector<std::array<double,7>> all_points;
    std::array<double, 4> plane_param = {0,0,0,0};
    std::size_t plane_rows = static_cast<std::size_t>(parameters.rows*parameters.reso/input.plane_reso);
    std::size_t plane_cols = static_cast<std::size_t>(parameters.cols*parameters.reso/input.plane_reso);

    for (std::size_t i=0; i<plane_rows*plane_cols)
    {
        std::vector<std::array<float, 3>> scaled_xyz = all_grid_points(labelledCloud, i, plane_param, parameters.origin, input.plane_reso, plane_cols);
        for (int k=0; k<scaledxyz.size(); k++)
        {
            all_points.push_back({scaledxyz[k][0], scaledxyz[k][1], scaledxyz[k][2], plane_param[0], plane_param[1], plane_param[2], plane_param[3]})       
        }
    }
    for (std::size_t i=0; i<parameters.rows*parameters.cols; i++)
    {
        for (int k=0; k<all_points.size(); k++)
        {
            double x = all_points[k][0];
            double y = all_points[k][1];
            if (grid_position(point.x, point.y, parameters.origin, parameters.reso, parameters.cols) == i)
            {
                double z = all_points[k][2];
                double a = all_points[k][3];
                double b = all_points[k][4];
                double c = all_points[k][5];
                double MSE = all_points[k][6];
                double point_dist = ((a*x + b*y - z + c)/sqrt(a*a + b*b + 1));
                if (point_dist > input.plane_ground && point_dist < (input.plane_ground + input.plane_offset) && MSE < input.MSEmax)
                {
                    *confidence_p[i] += 1;
                }
            }
        }
    }
}

void extract(const pcl::PointCloud<PointT>::Ptr labelledCloud, const Grid2D::ExtractionSettings& input, std::vector<Grid2D::Labels>& m_grid, const T& count, const T& confidence_l, const T& confidence_p, const T& confidence_z)
{
    for (std::size_t i=0; i<sizeof(count); i++)
    {
        float confidence = (input.confidence_label*confidence_l[i] + input.confidence_zaxis*confidence_z[i] + input.confidence_plane*confidence_p[i])/(input.confidence_label + input.confidence_plane + input.confidence_zaxis);

        if (count[i] == 0)
        {
            m_grid.push_back(Unknown);
        }
        else if (confidence <= input.confidence_threshold)
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

void dilate(std::vector<Grid2D::Labels>& m_grid, const GridParameters& parameters)
{
    cv::Mat src = cv::Mat(static_cast<int>(parameters.rows), static_cast<int>(parameters.cols), CV_8U, m_grid, AUTO_STEP);
    cv::Mat opening_dst;
    cv::morphologyEx(src, opening_dst, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(2,2)));
    src = opening_dst;
    return;
}

}


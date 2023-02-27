#include "ground_extraction/ground_extractor.hpp"

#include <chrono>
#include <cstddef>
#include <iostream>
#include <thread>
#include <limits>
#include <cmath>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/point_types.h>
#include <nlohmann/json.hpp>

unsigned int text_id = 0;
using PointT = pcl::PointXYZL;

namespace GroundExtraction
{

void GridConversion(const std::vector<Grid2D::Labels>& m_grid, pcl::PointCloud<PointT>::Ptr &ground_map, pcl::PointCloud<PointT>::Ptr &obstacle_map, const pcl::PointCloud<PointT>::Ptr labelled_cloud, std::string filename, const ExtractionSettings& input_param)
{               
    float x_min = std::numeric_limits<float>::max(); 
    float x_max = std::numeric_limits<float>::lowest();
    float y_min = std::numeric_limits<float>::max();
    float y_max = std::numeric_limits<float>::lowest();
    float reso = input_param.m_resolution;
    if (input_param.map_boundaries[0] != x_min || input_param.map_boundaries[1] != x_max || input_param.map_boundaries[2] != y_min || input_param.map_boundaries[3] != y_max)
    {
        x_min = input_param.map_boundaries[0];
        x_max = input_param.map_boundaries[1];
        y_min = input_param.map_boundaries[2];
        y_max = input_param.map_boundaries[3];
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
    std::size_t cols = static_cast<std::size_t>(std::ceil((x_max-x_min)/(reso)));
    for (std::size_t i=0; i<m_grid.size(); i++)
    {
        PointT ground, obstacle;
        if (m_grid[i] == Grid2D::Labels::Unoccupied)
        {
            std::size_t y = static_cast<std::size_t>(floor(i/cols));
            ground.y = y_max - (y+0.5)*reso;
            ground.x = ((i-y*cols)+0.5)*reso + x_min;
            ground.z = input_param.zaxis_ceil;
            ground.label = 1;
            ground_map->points.push_back(ground);
        }
        else if (m_grid[i] == Grid2D::Labels::Obstacle)
        {
            std::size_t y = static_cast<std::size_t>(floor(i/cols));
            obstacle.y = y_max - (y+0.5)*reso;
            obstacle.x = ((i-y*cols)+0.5)*reso + x_min;
            obstacle.z = input_param.zaxis_ceil;
            obstacle.label = 0;
            obstacle_map->points.push_back(obstacle);
        }
    }
}

static void pointpickingEventOccured(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
    std::cout << "[INOF] Point picking event occurred." << std::endl;
    float x, y, z;
    if (event.getPointIndex () == -1)
    {
        return;
    }
    event.getPoint(x, y, z);
    std::cout << "[INOF] Point coordinate ( " << x << ", " << y << ", " << z << ")" << std::endl;
}
pcl::visualization::PCLVisualizer::Ptr GridVis (std::string filename, pcl::PointCloud<PointT>::ConstPtr labelled_cloud, pcl::PointCloud<PointT>::ConstPtr ground_map, pcl::PointCloud<PointT>::ConstPtr obstacle_map)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointT> (labelled_cloud, filename+"cloud");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> blue_color(obstacle_map, 0, 0, 255);
    viewer->addPointCloud<PointT> (obstacle_map, blue_color, filename+"_obstacle");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red_color(ground_map, 255, 0, 0);
    viewer->addPointCloud<PointT> (ground_map, red_color, filename+"_ground");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, filename+"_ground");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, filename+"_obstacle");
    viewer->addCoordinateSystem (1.0,"cloud");
    viewer->initCameraParameters ();
    return (viewer);
}
static void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    if (event.getKeySym () == "r" && event.keyDown ())
    {
        std::cout << "r was pressed => removing all text" << std::endl;

        char str[512];
        for (unsigned int i = 0; i < text_id; ++i)
        {
        sprintf (str, "text#%03d", i);
        viewer->removeShape (str);
        }
        text_id = 0;
    }
}
static void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
        event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
        std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

        char str[512];
        sprintf (str, "text#%03d", text_id ++);
        viewer->addText ("clicked here", event.getX (), event.getY (), str);
    }
}
pcl::visualization::PCLVisualizer::Ptr interactionCustomizationVis ()
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);

    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
    viewer->registerMouseCallback (mouseEventOccurred, (void*)viewer.get ());

    return (viewer);
}

} // namespace GroundExtraction

using namespace GroundExtraction;

int main(int argc, char** argv)
{    
    ExtractionSettings input_param; 
    ifstream settingsF("/home/joel/ground_extration_lib/example/settings.json");
    if (!settingsF.is_open()) 
    {
        return 1;
    }
    std::cout << "done";
    nlohmann::json settingsJson = nlohmann::json::parse(settingsF);
    std::string filename = settingsJson["filename"].get<std::string>();
    pcl::PointCloud<PointT>::Ptr labelled_cloud (new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT> (filename + ".pcd", *labelled_cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file \n");
        return (-1);
    }
    
   
    if (filename == "Galen_lvl5")
    {
        input_param.map_boundaries[0] = settingsJson["x_min"].get<float>();
        input_param.map_boundaries[1] = settingsJson["x_max"].get<float>();
        input_param.map_boundaries[2] = settingsJson["y_min"].get<float>();
        input_param.map_boundaries[3] = settingsJson["y_max"].get<float>();
    }
    input_param.output_filename = filename + "_2D";
    // input_param.m_resolution = settingsJson["grid_resolution"].get<float>();
    // input_param.zaxis_ground = settingsJson["zaxis_max_height"].get<float>();
    // input_param.zaxis_ceil = settingsJson["zaxis_ground_height"].get<float>();
    // input_param.MSEmax = settingsJson["plane_MSE_threshold"].get<float>();
    // input_param.plane_ground = settingsJson["distance_from_plane"].get<float>();
    // input_param.plane_offset = settingsJson["plane_offset"].get<float>();
    // input_param.plane_resolution = settingsJson["plane_resolution"].get<float>();
    input_param.confidence_label = settingsJson["confidence_label"].get<float>();
    input_param.confidence_zaxis = settingsJson["confidence_zaxis"].get<float>();   
    input_param.confidence_plane = settingsJson["confidence_plane"].get<float>();
    input_param.confidence_threshold = settingsJson["probability_threshold"].get<float>();
    std::cout << "settingsjson done";
    

    pcl::PointCloud<PointT>::Ptr ground_map (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr obstacle_map (new pcl::PointCloud<PointT>);
    
    const auto start = std::chrono::high_resolution_clock::now();

    GridConversion(Extract(labelled_cloud, input_param).m_grid, ground_map, obstacle_map, labelled_cloud, filename, input_param);

    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = GridVis(filename, labelled_cloud, ground_map, obstacle_map);

    // int coordinate;
    // cout << "See point coordinates? (Yes: 1, No: 0): ";
    // cin >> coordinate;
    // if (coordinate == 1)
    // {
    //     viewer->registerPointPickingCallback(pointpickingEventOccured, (void*)&viewer);
    // }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}


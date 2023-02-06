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

// std::string filename = "Pixel_Lvl3_2609";
std::string filename = "Galen_lvl5";
// std::string filename = "pixel_park2_20220103";

namespace GroundExtraction
{

void gridconversion(const std::vector<Grid2D::Labels>& m_grid, pcl::PointCloud<PointT>::Ptr &groundmap, pcl::PointCloud<PointT>::Ptr &obstaclemap, const pcl::PointCloud<PointT>::Ptr labelledCloud, std::string filename, const Grid2D::ExtractionSettings& input)
{               
    float xmin = std::numeric_limits<float>::max(); 
    float xmax = std::numeric_limits<float>::lowest();
    float ymin = std::numeric_limits<float>::max();
    float ymax = std::numeric_limits<float>::lowest();
    float reso = input.m_reso;
    if (input.map_boundaries[0] == 0 || input.map_boundaries[1] == 0 || input.map_boundaries[2] == 0 || input.map_boundaries[3] == 0)
    {
        xmin = input.map_boundaries[0];
        xmax = input.map_boundaries[1];
        ymin = input.map_boundaries[2];
        ymax = input.map_boundaries[3];
    }
    else
    {
        for (const auto& point: *labelledCloud)   
        {
            xmin = std::min(xmin, point.x);
            xmax = std::max(xmax, point.x);
            ymin = std::min(ymin, point.y);
            ymax = std::max(ymax, point.y);                
        }
    }
    std::size_t cols = static_cast<std::size_t>(std::ceil((xmax-xmin)/(reso)));
    for (std::size_t i=0; i<m_grid.size(); i++)
    {
        PointT ground, obstacle;
        if (m_grid[i] == Grid2D::Labels::Unoccupied)
        {
            std::size_t y = static_cast<std::size_t>(floor(i/cols));
            ground.y = ymax - (y+0.5)*reso;
            ground.x = ((i-y*cols)+0.5)*reso + xmin;
            ground.z = input.zaxis_ceil;
            ground.label = 1;
            groundmap->points.push_back(ground);
        }
        else if (m_grid[i] == Grid2D::Labels::Obstacle)
        {
            std::size_t y = static_cast<std::size_t>(floor(i/cols));
            obstacle.y = ymax - (y+0.5)*reso;
            obstacle.x = ((i-y*cols)+0.5)*reso + xmin;
            obstacle.z = input.zaxis_ceil;
            obstacle.label = 0;
            obstaclemap->points.push_back(obstacle);
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
pcl::visualization::PCLVisualizer::Ptr gridVis (std::string filename, pcl::PointCloud<PointT>::ConstPtr labelledCloud, pcl::PointCloud<PointT>::ConstPtr groundmap, pcl::PointCloud<PointT>::ConstPtr obstaclemap)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointT> (labelledCloud, filename+"cloud");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> blue_color(obstaclemap, 0, 0, 255);
    viewer->addPointCloud<PointT> (obstaclemap, blue_color, filename+"_obstacle");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red_color(groundmap, 255, 0, 0);
    viewer->addPointCloud<PointT> (groundmap, red_color, filename+"_ground");
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


int main(int argc, char** argv)
{
    std::vector<Grid2D::Labels> m_grid;
    pcl::PointCloud<PointT>::Ptr labelledCloud (new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT> (filename + ".pcd", *labelledCloud) == -1)
    {
        PCL_ERROR ("Couldn't read file \n");
        return (-1);
    }
    
    GroundExtraction::Grid2D::ExtractionSettings input;
    ifstream settingsF("/home/joel/ground_extration_lib/example/settings.json");
    if (!settingsF.is_open()) 
    {
        return 1;
    }
    nlohmann::json settingsJson = nlohmann::json::parse(settingsF);
    
    input.map_boundaries[0] = settingsJson["grid_parameters"][0];
    input.map_boundaries[1] = settingsJson["grid_parameters"][1];
    input.map_boundaries[2] = settingsJson["grid_parameters"][2];
    input.map_boundaries[3] = settingsJson["grid_parameters"][3];
    input.m_reso = settingsJson["grid_resolution"].get<float>();
    input.zaxis_ground = settingsJson["zaxis_max_height"].get<float>();
    input.zaxis_ceil = settingsJson["zaxis_ground_height"].get<float>();
    input.MSEmax = settingsJson["plane_MSE_threshold"].get<double>();
    input.plane_ground = settingsJson["distance_from_plane"].get<float>();
    input.plane_offset = settingsJson["plane_offset"].get<float>();
    input.plane_reso = settingsJson["plane_resolution"].get<float>();
    input.confidence_label = settingsJson["confidence_label"].get<float>();
    input.confidence_zaxis = settingsJson["confidence_zaxis"].get<float>();   
    input.confidence_plane = settingsJson["confidence_plane"].get<float>();
    input.confidence_threshold = settingsJson["probability_threshold"].get<float>();
    
    GroundExtraction::Extract(labelledCloud, input, m_grid);


    pcl::PointCloud<PointT>::Ptr groundmap (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr obstaclemap (new pcl::PointCloud<PointT>);
    gridconversion(m_grid, groundmap, obstaclemap, labelledCloud, filename, input);
    
    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = gridVis(filename, labelledCloud, groundmap, obstaclemap);

    int coordinate;
    cout << "See point coordinates? (Yes: 1, No: 0): ";
    cin >> coordinate;
    if (coordinate == 1)
    {
        viewer->registerPointPickingCallback(pointpickingEventOccured, (void*)&viewer);
    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}

}
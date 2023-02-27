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

// OpenCV
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

unsigned int text_id = 0;
using PointT = pcl::PointXYZL;
// std::string filename = "Pixel_Lvl3_2609";
// std::string filename = "Galen_lvl5";
// std::string filename = "pixel_park2_20220103";

float x_min_ = std::numeric_limits<float>::max(); 
float x_max_ = std::numeric_limits<float>::lowest();
float y_min_ = std::numeric_limits<float>::max();
float y_max_ = std::numeric_limits<float>::lowest();

namespace GroundExtraction
{


void getMaxRange (const pcl::PointCloud<PointT>::Ptr labelled_cloud, const ExtractionSettings& input_param)
{
    for (const auto& point: *labelled_cloud)   
    {
        x_min_ = std::min(x_min_, point.x);
        x_max_ = std::max(x_max_, point.x);
        y_min_ = std::min(y_min_, point.y);
        y_max_ = std::max(y_max_, point.y);                
    }
}


void GridConversion(const std::vector<Grid2D::Labels>& m_grid, 
                    pcl::PointCloud<PointT>::Ptr &ground_map, 
                    pcl::PointCloud<PointT>::Ptr &obstacle_map,  
                    const ExtractionSettings& input_param)
{               
    float reso = input_param.m_resolution;

    std::size_t cols = static_cast<std::size_t>(std::ceil((x_max_-x_min_)/(reso)));
    for (std::size_t i = 0; i < m_grid.size(); i++)
    {
        PointT ground, obstacle;
        if (m_grid[i] == Grid2D::Labels::Unoccupied)
        {
            std::size_t y = static_cast<std::size_t>(floor(i/cols));
            ground.y = y_max_ - (y+0.5)*reso;
            ground.x = ((i-y*cols)+0.5)*reso + x_min_;
            ground.z = input_param.zaxis_ceil;
            ground.label = 1;
            ground_map->points.push_back(ground);
        }
        else if (m_grid[i] == Grid2D::Labels::Obstacle)
        {
            std::size_t y = static_cast<std::size_t>(floor(i/cols));
            obstacle.y = y_max_ - (y+0.5)*reso;
            obstacle.x = ((i-y*cols)+0.5)*reso + x_min_;
            obstacle.z = input_param.zaxis_ceil;
            obstacle.label = 0;
            obstacle_map->points.push_back(obstacle);
        }
    }
}

nlohmann::json CreateMapconfig (const ExtractionSettings& input_param, const cv::Size& size)
{
    nlohmann::json output;

    int origin_x = static_cast<int>(std::floor(y_max_/input_param.m_resolution)); // x
    int origin_y = static_cast<int>(std::floor(x_max_/input_param.m_resolution)); // y

    output["name"] = "test";
    output["img_height_px"] = size.height;
    output["img_width_px"] = size.width;
    output["origin_x_px"] = origin_x;
    output["origin_y_px"] = origin_y;
    output["resolution"] = input_param.m_resolution;

    return output;
}

bool checkSurroundingFree (const std::vector<Grid2D::Labels>& m_grid, int pixel_x, int pixel_y, int col_max, int row_max, int search)
{
    int min_x_check = std::max(pixel_x - search, 0);
    int max_x_check = std::min(pixel_x + search, col_max);
    int min_y_check = std::max(pixel_y - search, 0);
    int max_y_check = std::min(pixel_y + search, row_max);

    for (int i = min_x_check; i <= max_x_check; i ++)
        for (int j = min_y_check; j <= max_y_check; j ++)
        {
            const int grid_id = i + j * col_max;
            if (m_grid[grid_id] == Grid2D::Labels::Unoccupied){
                return true;
            }
        }   

    return false;
}

cv::Mat CreateImage (const pcl::PointCloud<PointT>::Ptr labelled_cloud, const std::vector<Grid2D::Labels>& m_grid, const ExtractionSettings& input_param)
{
    float reso = input_param.m_resolution;
    int cols = static_cast<int>(std::ceil((x_max_-x_min_)/reso)); 
    int rows = static_cast<int>(std::ceil((y_max_-y_min_)/reso));
    std::cout << "cols : " << cols << " rows : " << rows << std::endl;
    cv::Mat output(rows + 1, cols + 1 , CV_8U);
    std::cout << " Image size " << output.size[0] << " * " << output.size[1] << " pixels" << std::endl;
    int inflation_ratio = static_cast<int>(std::ceil(input_param.m_inflation_dis/input_param.m_resolution));
    std::cout << " Inflation ratio " << inflation_ratio << std::endl;
    for (std::size_t i = 0; i < m_grid.size(); i++)
    {
        int y = static_cast<int>(floor(i/cols));
        int x = i - y*cols;
        assert(y < output.size[1] && x < output.size[0]);
        if (m_grid[i] == Grid2D::Labels::Unoccupied){
            cv::circle(output, cv::Point(x, y), 1, uchar(255), -1);
        } else if (m_grid[i] == Grid2D::Labels::Obstacle){
            cv::circle(output, cv::Point(x, y), 1, uchar(0), inflation_ratio);
        }  else if (m_grid[i] == Grid2D::Labels::Unknown){
            if (checkSurroundingFree(m_grid, x, y, cols, rows, 4)) {
                cv::circle(output, cv::Point(x, y), 1, uchar(255), -1);
            } else {
                cv::circle(output, cv::Point(x, y), 1, uchar(100), -1);
            }
        }
    }

    cv::rotate(output, output, cv::ROTATE_90_COUNTERCLOCKWISE);
    return output;
}

pcl::visualization::PCLVisualizer::Ptr GridVis (pcl::PointCloud<PointT>::ConstPtr labelled_cloud, pcl::PointCloud<PointT>::ConstPtr ground_map, pcl::PointCloud<PointT>::ConstPtr obstacle_map)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    // viewer->addPointCloud<PointT> (labelled_cloud, filename+"cloud");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> blue_color(obstacle_map, 0, 0, 255);
    viewer->addPointCloud<PointT> (obstacle_map, blue_color, "obstacle");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red_color(ground_map, 255, 0, 0);
    viewer->addPointCloud<PointT> (ground_map, red_color, "ground");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "ground");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "obstacle");
    viewer->addCoordinateSystem (1.0,"cloud");
    viewer->initCameraParameters ();
    return (viewer);
}


} // namespace GroundExtraction

using namespace GroundExtraction;

int main(int argc, const char* argv[])
{    
	if(argc != 2){
        std::cout << "Arguments incomplets. ./ground_extractor <Pointcloud-Path>" << std::endl;
        return 0;
    }

    ExtractionSettings input_param;
    pcl::PointCloud<PointT>::Ptr labelled_cloud (new pcl::PointCloud<PointT>);

    std::string addr = argv[1];
    input_param.output_filename = addr + "_2D";
    if (pcl::io::loadPCDFile<PointT> (addr, *labelled_cloud) == -1)
    {
        return 0;
    }
    
    std::ifstream settingsF("/home/joel/ground_extration_lib/example/settings.json");
    if (!settingsF) 
    {
        std::cout << "Cannot load the setting file" << std::endl;
        return 0;
    }

    nlohmann::json settingsJson = nlohmann::json::parse(settingsF);
    input_param.m_inflation_dis = settingsJson["inflation_dis"].get<float>();
    input_param.confidence_label = settingsJson["confidence_label"].get<float>();
    input_param.confidence_zaxis = settingsJson["confidence_zaxis"].get<float>();   
    input_param.confidence_plane = settingsJson["confidence_plane"].get<float>();
    input_param.confidence_threshold = settingsJson["probability_threshold"].get<float>();
    std::cout << "Read settings json done" << std::endl;

    pcl::PointCloud<PointT>::Ptr ground_map (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr obstacle_map (new pcl::PointCloud<PointT>);

    getMaxRange(labelled_cloud, input_param);

    const auto start = std::chrono::high_resolution_clock::now();
    const auto grid_2d = Extract(labelled_cloud, input_param);
    GridConversion(grid_2d.m_grid, ground_map, obstacle_map, input_param);
    const auto end = std::chrono::high_resolution_clock::now();
    const auto time_used = end - start;

    std::cout<<"Time used for grid conversion (ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(time_used).count() << std::endl;
    
    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = GridVis(labelled_cloud, ground_map, obstacle_map);

    cv::Mat MapImage = CreateImage(labelled_cloud, grid_2d.m_grid, input_param);
    cv::imwrite("2D_Grid_Map.png", MapImage);

    nlohmann::json map2DConfig = CreateMapconfig(input_param, MapImage.size());

    std::ofstream map2DConfigFile("map_config.json");
	map2DConfigFile<<std::setw(4)<<map2DConfig<<std::endl;
	map2DConfigFile.close();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}

// #include "ground_extraction/ground_extractor.hpp"

// #include <chrono>
// #include <cstddef>
// #include <iostream>
// #include <thread>
// #include <limits>
// #include <cmath>
// #include <pcl/common/common_headers.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/console/parse.h>
// #include <pcl/visualization/point_cloud_color_handlers.h>
// #include <pcl/point_types.h>
// #include <nlohmann/json.hpp>

// unsigned int text_id = 0;
// using PointT = pcl::PointXYZL;

// namespace GroundExtraction
// {

// void GridConversion(const std::vector<Grid2D::Labels>& m_grid, pcl::PointCloud<PointT>::Ptr &ground_map, pcl::PointCloud<PointT>::Ptr &obstacle_map, const pcl::PointCloud<PointT>::Ptr labelled_cloud, std::string filename, const ExtractionSettings& input_param)
// {               
//     float x_min = std::numeric_limits<float>::max(); 
//     float x_max = std::numeric_limits<float>::lowest();
//     float y_min = std::numeric_limits<float>::max();
//     float y_max = std::numeric_limits<float>::lowest();
//     float reso = input_param.m_resolution;
//     if (input_param.map_boundaries[0] != x_min || input_param.map_boundaries[1] != x_max || input_param.map_boundaries[2] != y_min || input_param.map_boundaries[3] != y_max)
//     {
//         x_min = input_param.map_boundaries[0];
//         x_max = input_param.map_boundaries[1];
//         y_min = input_param.map_boundaries[2];
//         y_max = input_param.map_boundaries[3];
//     }
//     else
//     {
//         for (const auto& point: *labelled_cloud)   
//         {
//             x_min = std::min(x_min, point.x);
//             x_max = std::max(x_max, point.x);
//             y_min = std::min(y_min, point.y);
//             y_max = std::max(y_max, point.y);                
//         }
//     }
//     std::size_t cols = static_cast<std::size_t>(std::ceil((x_max-x_min)/(reso)));
//     for (std::size_t i=0; i<m_grid.size(); i++)
//     {
//         PointT ground, obstacle;
//         if (m_grid[i] == Grid2D::Labels::Unoccupied)
//         {
//             std::size_t y = static_cast<std::size_t>(floor(i/cols));
//             ground.y = y_max - (y+0.5)*reso;
//             ground.x = ((i-y*cols)+0.5)*reso + x_min;
//             ground.z = input_param.zaxis_ceil;
//             ground.label = 1;
//             ground_map->points.push_back(ground);
//         }
//         else if (m_grid[i] == Grid2D::Labels::Obstacle)
//         {
//             std::size_t y = static_cast<std::size_t>(floor(i/cols));
//             obstacle.y = y_max - (y+0.5)*reso;
//             obstacle.x = ((i-y*cols)+0.5)*reso + x_min;
//             obstacle.z = input_param.zaxis_ceil;
//             obstacle.label = 0;
//             obstacle_map->points.push_back(obstacle);
//         }
//     }
// }

// static void pointpickingEventOccured(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
// {
//     std::cout << "[INOF] Point picking event occurred." << std::endl;
//     float x, y, z;
//     if (event.getPointIndex () == -1)
//     {
//         return;
//     }
//     event.getPoint(x, y, z);
//     std::cout << "[INOF] Point coordinate ( " << x << ", " << y << ", " << z << ")" << std::endl;
// }
// pcl::visualization::PCLVisualizer::Ptr GridVis (std::string filename, pcl::PointCloud<PointT>::ConstPtr labelled_cloud, pcl::PointCloud<PointT>::ConstPtr ground_map, pcl::PointCloud<PointT>::ConstPtr obstacle_map)
// {
//     pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//     viewer->setBackgroundColor (0, 0, 0);
//     viewer->addPointCloud<PointT> (labelled_cloud, filename+"cloud");
//     pcl::visualization::PointCloudColorHandlerCustom<PointT> blue_color(obstacle_map, 0, 0, 255);
//     viewer->addPointCloud<PointT> (obstacle_map, blue_color, filename+"_obstacle");
//     pcl::visualization::PointCloudColorHandlerCustom<PointT> red_color(ground_map, 255, 0, 0);
//     viewer->addPointCloud<PointT> (ground_map, red_color, filename+"_ground");
//     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, filename+"_ground");
//     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, filename+"_obstacle");
//     viewer->addCoordinateSystem (1.0,"cloud");
//     viewer->initCameraParameters ();
//     return (viewer);
// }
// static void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
// {
//     pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
//     if (event.getKeySym () == "r" && event.keyDown ())
//     {
//         std::cout << "r was pressed => removing all text" << std::endl;

//         char str[512];
//         for (unsigned int i = 0; i < text_id; ++i)
//         {
//         sprintf (str, "text#%03d", i);
//         viewer->removeShape (str);
//         }
//         text_id = 0;
//     }
// }
// static void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void)
// {
//     pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
//     if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
//         event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
//     {
//         std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

//         char str[512];
//         sprintf (str, "text#%03d", text_id ++);
//         viewer->addText ("clicked here", event.getX (), event.getY (), str);
//     }
// }
// pcl::visualization::PCLVisualizer::Ptr interactionCustomizationVis ()
// {
//     pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//     viewer->setBackgroundColor (0, 0, 0);
//     viewer->addCoordinateSystem (1.0);

//     viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
//     viewer->registerMouseCallback (mouseEventOccurred, (void*)viewer.get ());

//     return (viewer);
// }

// } // namespace GroundExtraction

// using namespace GroundExtraction;

// int main(int argc, char** argv)
// {    
//     ExtractionSettings input_param; 
//     ifstream settingsF("/home/joel/ground_extration_lib/example/settings.json");
//     if (!settingsF.is_open()) 
//     {
//         return 1;
//     }
//     std::cout << "done";
//     nlohmann::json settingsJson = nlohmann::json::parse(settingsF);
//     std::string filename = settingsJson["filename"].get<std::string>();
//     pcl::PointCloud<PointT>::Ptr labelled_cloud (new pcl::PointCloud<PointT>);
//     if (pcl::io::loadPCDFile<PointT> (filename + ".pcd", *labelled_cloud) == -1)
//     {
//         PCL_ERROR ("Couldn't read file \n");
//         return (-1);
//     }
    
   
//     if (filename == "Galen_lvl5")
//     {
//         input_param.map_boundaries[0] = settingsJson["x_min"].get<float>();
//         input_param.map_boundaries[1] = settingsJson["x_max"].get<float>();
//         input_param.map_boundaries[2] = settingsJson["y_min"].get<float>();
//         input_param.map_boundaries[3] = settingsJson["y_max"].get<float>();
//     }
//     input_param.output_filename = filename + "_2D";
//     // input_param.m_resolution = settingsJson["grid_resolution"].get<float>();
//     // input_param.zaxis_ground = settingsJson["zaxis_max_height"].get<float>();
//     // input_param.zaxis_ceil = settingsJson["zaxis_ground_height"].get<float>();
//     // input_param.MSEmax = settingsJson["plane_MSE_threshold"].get<float>();
//     // input_param.plane_ground = settingsJson["distance_from_plane"].get<float>();
//     // input_param.plane_offset = settingsJson["plane_offset"].get<float>();
//     // input_param.plane_resolution = settingsJson["plane_resolution"].get<float>();
//     input_param.confidence_label = settingsJson["confidence_label"].get<float>();
//     input_param.confidence_zaxis = settingsJson["confidence_zaxis"].get<float>();   
//     input_param.confidence_plane = settingsJson["confidence_plane"].get<float>();
//     input_param.confidence_threshold = settingsJson["probability_threshold"].get<float>();
//     std::cout << "settingsjson done";
    

//     pcl::PointCloud<PointT>::Ptr ground_map (new pcl::PointCloud<PointT>);
//     pcl::PointCloud<PointT>::Ptr obstacle_map (new pcl::PointCloud<PointT>);
    
//     const auto start = std::chrono::high_resolution_clock::now();

//     GridConversion(Extract(labelled_cloud, input_param).m_grid, ground_map, obstacle_map, labelled_cloud, filename, input_param);

//     pcl::visualization::PCLVisualizer::Ptr viewer;
//     viewer = GridVis(filename, labelled_cloud, ground_map, obstacle_map);

//     // int coordinate;
//     // cout << "See point coordinates? (Yes: 1, No: 0): ";
//     // cin >> coordinate;
//     // if (coordinate == 1)
//     // {
//     //     viewer->registerPointPickingCallback(pointpickingEventOccured, (void*)&viewer);
//     // }

//     while (!viewer->wasStopped ())
//     {
//         viewer->spinOnce (100);
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }
//     return 0;
// }


#include "internal_utils.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <cstddef>

#include <Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pcl/features/feature.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>

namespace GroundExtraction {

using PointT = pcl::PointXYZL;

std::size_t findGridPosition(float x, float y, const std::array<float, 2> origin, float reso, int cols) {
    int cellx = static_cast<int>(floor((x - origin[0]) / reso));
    int celly = static_cast<int>(floor((origin[1] - y) / reso));

    return (static_cast<std::size_t>(celly * cols + cellx));
}

void planeFitting(const std::vector<std::array<float, 3>>& ground_points, Eigen::Vector4f& plane_coefficients) {
    float XXsum = 0;
    float XYsum = 0;
    float YYsum = 0;
    float XZsum = 0;
    float YZsum = 0;

    std::array<float, 3> centroid = {0, 0, 0};
    for (size_t i = 0; i < ground_points.size(); i++) {
        centroid[0] += ground_points[i][0];
        centroid[1] += ground_points[i][1];
        centroid[2] += ground_points[i][2];
    }
    float recpNumPts = 1.0f / static_cast<float>(ground_points.size());
    centroid[0] *= recpNumPts;
    centroid[1] *= recpNumPts;
    centroid[2] *= recpNumPts;
    for (size_t i = 0; i < ground_points.size(); i++) {
        float x = ground_points[i][0];
        float y = ground_points[i][1];
        float z = ground_points[i][2];
        XXsum += ((x - centroid[0]) * (x - centroid[0]));
        XYsum += ((x - centroid[0]) * (y - centroid[1]));
        YYsum += ((y - centroid[1]) * (y - centroid[1]));

        XZsum += ((z - centroid[2]) * (x - centroid[0]));
        YZsum += ((z - centroid[2]) * (y - centroid[1]));
    }
    plane_coefficients(0) = static_cast<float>((YYsum * XZsum - XYsum * YZsum) / (XXsum * YYsum - XYsum * XYsum));
    plane_coefficients(1) = (XXsum * YZsum - XYsum * XZsum) / (XXsum * YYsum - XYsum * XYsum);
    plane_coefficients(2) = -1;
    plane_coefficients(3) = centroid[2] - plane_coefficients(0) * centroid[0] - plane_coefficients(1) * centroid[1];
    return;
}

void planeFittingPCA(const std::vector<std::array<float, 3>>& ground_points, Eigen::Vector4f& plane_coefficients)
{
    float XX_sum = 0.0;
    float XY_sum = 0.0;
    float YY_sum = 0.0;
    float XZ_sum = 0.0;
    float YZ_sum = 0.0;
    float ZZ_sum = 0.0;
    Eigen::Vector3f abc;

    Eigen::Vector3f centroid = Eigen::Vector3f(0.0, 0.0, 0.0);
    for (size_t i = 0; i < ground_points.size(); i++) {
        centroid(0) += ground_points[i][0];
        centroid(1) += ground_points[i][1];
        centroid(2) += ground_points[i][2];
    }
    float recpNumPts = 1.0f / static_cast<float>(ground_points.size());
    centroid(0) *= recpNumPts;
    centroid(1) *= recpNumPts;
    centroid(2) *= recpNumPts;
    for (size_t i = 0; i < ground_points.size(); i++) {
        float x = ground_points[i][0];
        float y = ground_points[i][1];
        float z = ground_points[i][2];
        XX_sum += ((x - centroid[0]) * (x - centroid[0]));
        XY_sum += ((x - centroid[0]) * (y - centroid[1]));
        YY_sum += ((y - centroid[1]) * (y - centroid[1]));
        ZZ_sum += ((z - centroid[2]) * (z - centroid[2]));

        XZ_sum += ((z - centroid[2]) * (x - centroid[0]));
        YZ_sum += ((z - centroid[2]) * (y - centroid[1]));
    }
    float det_x =  YY_sum*ZZ_sum - YZ_sum*YZ_sum;
    float det_y = XX_sum*ZZ_sum - XZ_sum*XZ_sum;
    float det_z = XX_sum*YY_sum - XY_sum*XY_sum;

    if (det_x > det_y && det_x > det_z)
    {
        abc = Eigen::Vector3f(det_x, XZ_sum*YZ_sum-XY_sum*ZZ_sum, XY_sum*YZ_sum-XZ_sum*YY_sum);
    }
    else if (det_y > det_z)
    {
        abc = Eigen::Vector3f(XZ_sum*YZ_sum-XY_sum*ZZ_sum, det_y, XY_sum*XZ_sum-YZ_sum*XX_sum);   
    }
    else
    {
        abc = Eigen::Vector3f(XZ_sum*YZ_sum-XY_sum*YY_sum, XY_sum*XZ_sum-YZ_sum*XX_sum, det_z);
    }

    float norm = abc.norm();
    if (norm == 0.0)
    {
        plane_coefficients = Eigen::Vector4f(0,0,0,0);
        return;
    }

    abc /= norm;
    plane_coefficients = Eigen::Vector4f(abc(0), abc(1), abc(2), -abc.dot(centroid));
}




void calculateMSE(const std::vector<std::array<float, 3>>& ground_points,
                  const Eigen::Vector4f& plane_coefficients, float& mean_squared_errors) {
    mean_squared_errors = 0;
    for (int i = 0; i < ground_points.size(); i++) {
        mean_squared_errors +=
            (plane_coefficients(0) * ground_points[i][0] + plane_coefficients(1) * ground_points[i][1] +
             plane_coefficients(2) * ground_points[i][2] + plane_coefficients(3)) *
            (plane_coefficients(0) * ground_points[i][0] + plane_coefficients(1) * ground_points[i][1] +
             plane_coefficients(2) * ground_points[i][2] + plane_coefficients(3));
    }
    mean_squared_errors /= ground_points.size();
    return;
}

void OutlierRemoval(pcl::PointCloud<PointT>::Ptr& labelled_cloud) {
    pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(labelled_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*filtered_cloud);
    labelled_cloud = filtered_cloud;
    return;
}

void defineGridBounds(pcl::PointCloud<PointT>::Ptr& labelled_cloud, Grid2D::GridParameters& m_parameters,
                      const ExtractionSettings& input_param) {
    float x_min = std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::lowest();
    float y_min = std::numeric_limits<float>::max();
    float y_max = std::numeric_limits<float>::lowest();
    if (input_param.map_boundaries[0] != x_min || input_param.map_boundaries[1] != x_max ||
        input_param.map_boundaries[2] != y_min || input_param.map_boundaries[3] != y_max) {
        x_min = input_param.map_boundaries[0];
        x_max = input_param.map_boundaries[1];
        y_min = input_param.map_boundaries[2];
        y_max = input_param.map_boundaries[3];

        pcl::PointCloud<PointT>::Ptr new_map(new pcl::PointCloud<PointT>);
        for (const auto& point : *labelled_cloud) {
            if (point.x > x_min && point.x < x_max && point.y > y_min && point.y < y_max) {
                new_map->points.push_back(point);
            }
        }
        labelled_cloud = new_map;
    } else {
        for (const auto& point : *labelled_cloud) {
            x_min = std::min(x_min, point.x);
            x_max = std::max(x_max, point.x);
            y_min = std::min(y_min, point.y);
            y_max = std::max(y_max, point.y);
        }
    }
    m_parameters.reso = input_param.m_resolution;
    m_parameters.rows = static_cast<std::size_t>(std::ceil((y_max - y_min) / (input_param.m_resolution)));
    m_parameters.cols = static_cast<std::size_t>(std::ceil((x_max - x_min) / (input_param.m_resolution)));
    m_parameters.origin = {x_min, y_max};
    return;
}

void LabelnZaxisMethod(pcl::PointCloud<PointT>::ConstPtr labelled_cloud, std::vector<std::uint8_t>& num_obstacle_labels,
                       std::vector<std::uint8_t>& num_obstacle_zaxis, std::vector<std::uint8_t>& num_points,
                       const Grid2D::GridParameters& m_parameters, const ExtractionSettings& input_param) {
    for (const auto& point : *labelled_cloud) {
        std::size_t position =
            findGridPosition(point.x, point.y, m_parameters.origin, m_parameters.reso, m_parameters.cols);
        num_points[position] += 1;
        if (point.label == 0) {
            num_obstacle_labels[position] += 1;
        }
        if (point.z > input_param.zaxis_ground && point.z < input_param.zaxis_ceil) {
            num_obstacle_zaxis[position] += 1;
        }
    }
    return;
}

struct PlaneParameters {
    std::vector<std::array<float, 3>> grid_points;
    std::vector<std::array<float, 3>> ground_points;
    Eigen::Vector4f plane_coefficients;
    float mean_squared_errors;
};

struct GridPlane {
    std::vector<std::array<float, 8>> all_points;
};

void PlaneMethod(pcl::PointCloud<PointT>::ConstPtr labelled_cloud, std::vector<std::uint8_t>& num_obstacle_plane,
                 const std::vector<std::uint8_t>& num_points, const Grid2D::GridParameters& m_parameters,
                 const ExtractionSettings& input_param) {
#if defined(WITH_PROFILING_PRINTOUTS)
    const auto start = std::chrono::high_resolution_clock::now();
#endif
    // x y z a b c MSE
    std::size_t plane_rows =
        static_cast<std::size_t>(std::ceil(m_parameters.rows * m_parameters.reso / input_param.plane_resolution));
    std::size_t plane_cols =
        static_cast<std::size_t>(std::ceil(m_parameters.cols * m_parameters.reso / input_param.plane_resolution));
    std::vector<PlaneParameters> plane_grid(plane_cols * plane_rows);
    std::vector<GridPlane> grid(m_parameters.rows * m_parameters.cols);
    for (const auto& point : *labelled_cloud) {
        std::size_t plane_position =
            findGridPosition(point.x, point.y, m_parameters.origin, input_param.plane_resolution, plane_cols);
        std::array<float, 3> curPt = {point.x, point.y, point.z};
        plane_grid[plane_position].grid_points.push_back(curPt);
        if (point.label != 0) {
            plane_grid[plane_position].ground_points.push_back(curPt);
        }
    }

    // #pragma omp parallel for num_threads(4)
    for (std::size_t i = 0; i < plane_grid.size(); i++) {
        std::cout << plane_grid[i].ground_points.size() << ", ";
        if (plane_grid[i].ground_points.size() > 5) {
            planeFitting(plane_grid[i].ground_points, plane_grid[i].plane_coefficients);
            calculateMSE(plane_grid[i].ground_points, plane_grid[i].plane_coefficients,
                         plane_grid[i].mean_squared_errors);
            for (int k = 0; k < plane_grid[i].grid_points.size(); k++) {
                float x = plane_grid[i].grid_points[k][0];
                float y = plane_grid[i].grid_points[k][1];
                float z = plane_grid[i].grid_points[k][2];
                std::size_t position =
                    findGridPosition(x, y, m_parameters.origin, m_parameters.reso, m_parameters.cols);
                grid[position].all_points.push_back(
                    {x, y, z, static_cast<float>(plane_grid[i].plane_coefficients(0)), static_cast<float>(plane_grid[i].plane_coefficients(1)),
                     static_cast<float>(plane_grid[i].plane_coefficients(2)), static_cast<float>(plane_grid[i].plane_coefficients(3)), plane_grid[i].mean_squared_errors});
            }
        }
    }

#if defined(WITH_PROFILING_PRINTOUTS)
    std::cout << "time taken (PLANEFIT): "
              << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() -
                                                                       start)
                     .count()
              << "micros." << std::endl;
    const auto plane = std::chrono::high_resolution_clock::now();
#endif

    for (std::size_t i = 0; i < grid.size(); i++) {
        for (int k = 0; k < grid[i].all_points.size(); k++) {
            float x = grid[i].all_points[k][0];
            float y = grid[i].all_points[k][1];
            float z = grid[i].all_points[k][2];
            float a = grid[i].all_points[k][3];
            float b = grid[i].all_points[k][4];
            float c = grid[i].all_points[k][5];
            float d = grid[i].all_points[k][6];
            float MSE = grid[i].all_points[k][7];
            float point_dist = ((a * x + b * y + c*z + d) / sqrt(a * a + b * b + 1));
            if (point_dist > input_param.plane_ground &&
                point_dist < (input_param.plane_ground + input_param.plane_offset) && MSE < input_param.MSEmax) {
                num_obstacle_plane[i] += 1;
            }
            if (MSE > input_param.MSEmax) {
                num_obstacle_plane[i] += 1;
            }
        }
    }
#if defined(WITH_PROFILING_PRINTOUTS)
    std::cout << "time taken (PLANE): "
              << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() -
                                                                       plane)
                     .count()
              << "micros." << std::endl;
#endif
    return;
}

void PlaneMethod2(pcl::PointCloud<PointT>::ConstPtr labelled_cloud, std::vector<std::uint8_t>& num_obstacle_plane,
                  const Grid2D::GridParameters& m_parameters, const ExtractionSettings& input_param) {
    std::vector<std::array<float, 3>> ground_points;
    Eigen::Vector4f plane_coefficients; // a, b, c in plane equation z = ax + by + c
    float mean_squared_errors;
    for (const auto& point : *labelled_cloud) {
        if (point.label != 0) {
            ground_points.push_back({point.x, point.y, point.z});
        }
    }
    planeFittingPCA(ground_points, plane_coefficients);
    // calculateMSE(ground_points, plane_coefficients, mean_squared_errors);
    for (const auto& point : *labelled_cloud) {
        float a = static_cast<float>(plane_coefficients(0));
        float b = static_cast<float>(plane_coefficients(1));
        float c = static_cast<float>(plane_coefficients(2));
        float d = static_cast<float>(plane_coefficients(3));
        float point_dist = (a * point.x + b * point.y + c*point.z + d) / (sqrt(a * a + b * b + 1));

        size_t position = findGridPosition(point.x, point.y, m_parameters.origin, m_parameters.reso, m_parameters.cols);
        if (point_dist > input_param.plane_ground &&
            point_dist < (input_param.plane_ground + input_param.plane_offset)) 
        {
            num_obstacle_plane[position] += 1;
        }
        // if (mean_squared_errors > input_param.MSEmax) {
        //     num_obstacle_plane[position] += 1;
        // }
    }
    return;
}

void ConfidenceExtraction(pcl::PointCloud<PointT>::ConstPtr labelled_cloud, const ExtractionSettings& input_param,
                          std::vector<Grid2D::Labels>& m_grid, const std::vector<std::uint8_t>& num_points,
                          const std::vector<std::uint8_t>& num_obstacle_labels,
                          const std::vector<std::uint8_t>& num_obstacle_plane,
                          const std::vector<std::uint8_t>& num_obstacle_zaxis) {
    for (int i = 0; i < static_cast<int>(num_points.size()); i++) {
        if (num_points[i] == 0) {
            m_grid.push_back(Grid2D::Labels::Unknown);
        } else {
            float confidence =
                (input_param.confidence_label * num_obstacle_labels[i] +
                 input_param.confidence_zaxis * num_obstacle_zaxis[i] +
                 input_param.confidence_plane * num_obstacle_plane[i]) /
                (input_param.confidence_label + input_param.confidence_plane + input_param.confidence_zaxis);

            if (confidence <= input_param.confidence_threshold) {
                m_grid.push_back(Grid2D::Labels::Unoccupied);
            } else {
                m_grid.push_back(Grid2D::Labels::Obstacle);
            }
        }
    }
    return;
}

void GridDilation(std::vector<Grid2D::Labels>& m_grid, const Grid2D::GridParameters& m_parameters) {
    cv::Mat src(static_cast<int>(m_parameters.rows), static_cast<int>(m_parameters.cols), CV_8U, m_grid.data(),
                cv::Mat::AUTO_STEP);
    cv::Mat opening_dst;
    cv::morphologyEx(src, opening_dst, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)));
    src = opening_dst;
    return;
}

void ExportPNG(const std::vector<Grid2D::Labels>& m_grid, const Grid2D::GridParameters& m_parameters,
               const std::string& filename) {
    cv::Mat img(static_cast<int>(m_parameters.rows), static_cast<int>(m_parameters.cols), CV_8U, cv::Scalar(200));

    for (std::size_t i = 0; i < m_grid.size(); i++) {
        int row_num = static_cast<int>(floor(i / m_parameters.cols));
        int col_num = static_cast<int>(i - row_num * m_parameters.cols);
        if (m_grid[i] == Grid2D::Labels::Unoccupied) {
            img.at<uchar>(row_num, col_num) = 255;
        }

        if (m_grid[i] == Grid2D::Labels::Unknown) {
            img.at<uchar>(row_num, col_num) = 200;
        }

        if (m_grid[i] == Grid2D::Labels::Obstacle) {
            img.at<uchar>(row_num, col_num) = 0;
        }
    }
    cv::imwrite(filename + ".png", img);
    return;
}

} // namespace GroundExtraction

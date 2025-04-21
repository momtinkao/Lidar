#define PCL_NO_PRECOMPILE
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>

#include <iomanip>  // for setw, setfill
#include <iostream>
/*
Custom the PointCloud type
*/
struct CustomPoint {
    PCL_ADD_POINT4D;  // Add standard XYZ + padding
    uint32_t frame_id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;  // Ensure proper alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(
    CustomPoint,
    (float, x, x)(float, y, y)(float, z, z)(uint32_t, frame_id, frame_id))

int main(int argc, char** argv) {
    pcl::PointCloud<CustomPoint> cloud;

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read(argv[1], cloud);  // Remember to download the file first!
    pcl::PointCloud<pcl::PointXYZRGB> t_cloud;
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.5);
    ec.setClusterTolerance(0.3);
    ec.setMinClusterSize(600);
    ec.setMaxClusterSize(1500);
    uint32_t current_frame_id = 0;
    time_t a, b;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> result_clouds;
    for (auto& point : cloud) {
        pcl::PointXYZRGB current_point;
        current_point.x = point.x;
        current_point.y = point.y;
        current_point.z = point.z;
        current_point.r = 255;
        current_point.g = 255;
        current_point.b = 255;
        if (point.frame_id != current_frame_id) {
            std::cout << "current frame id: " << point.frame_id << " stored frame id:" << current_frame_id << std::endl;
            pcl::PointCloud<pcl::PointXYZRGB> filter_cloud;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr without_plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            if (current_frame_id == 0) {
                current_frame_id = point.frame_id;
                continue;
            } else {
                std::cout << "PointCloud before filtering: " << t_cloud.size() << " data points" << std::endl;
                // Voxel grid
                a = clock();
                sor.setInputCloud(t_cloud.makeShared());
                sor.setLeafSize(0.07f, 0.2f, 0.2f);
                sor.setDownsampleAllData(true);  // don't average unknown fields
                sor.filter(filter_cloud);
                b = clock();
                std::cout << "PointCloud after filtering: " << filter_cloud.width * filter_cloud.height << " data points" << " Cost " << double(b - a) / CLOCKS_PER_SEC << " Seconds" << std::endl;

                // Ground Removing
                a = clock();
                seg.setInputCloud(filter_cloud.makeShared());
                seg.segment(*inliers, *coefficients);
                extract.setInputCloud(filter_cloud.makeShared());
                extract.setIndices(inliers);
                extract.setNegative(false);
                extract.filter(*plane_cloud);
                extract.setNegative(true);
                extract.filter(*without_plane_cloud);
                b = clock();
                std::cout << "Removing ground cost " << double(b - a) / CLOCKS_PER_SEC << "seconds" << std::endl;

                // Cluster Cars
                a = clock();
                std::vector<pcl::PointIndices> cluster_indices;
                tree->setInputCloud(without_plane_cloud);
                ec.setSearchMethod(tree);
                ec.setInputCloud(without_plane_cloud);
                ec.extract(cluster_indices);
                for (const auto& cluster : cluster_indices) {
                    for (const auto& idx : cluster.indices) {
                        without_plane_cloud->points[idx].g = 0;
                        without_plane_cloud->points[idx].b = 0;
                    }
                }

                b = clock();
                std::cout << "Cluster cost " << double(b - a) / CLOCKS_PER_SEC << "seconds" << std::endl;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                *result_cloud = *without_plane_cloud;
                *result_cloud += *plane_cloud;
                result_clouds.emplace_back(result_cloud);
            }
            current_frame_id = point.frame_id;
            t_cloud.clear();
        }
        t_cloud.emplace_back(current_point);
    }
    pcl::visualization::PCLVisualizer viewer("RGB PCD Viewer");

    // Add the point cloud to the visualizer
    viewer.addPointCloud<pcl::PointXYZRGB>(result_clouds[1], "cloud");

    // Set background color of the viewer (optional)
    viewer.setBackgroundColor(0.0, 0.0, 0.0);  // Black background

    // Set the size of the points (optional)
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer.initCameraParameters();
    viewer.setCameraPosition(5.0,  // 摄像机 x 位置（站在 X 轴上）
                             0.0,  // 摄像机 y 位置
                             0.0,  // 摄像机 z 位置
                             0.0,  // 焦点 x 位置（指向原点，即 YZ 平面）
                             0.0,  // 焦点 y 位置
                             0.0,  // 焦点 z 位置
                             0.0,  // "上"方向 x 分量
                             0.0,  // "上"方向 y 分量
                             1.0   // "上"方向 z 分量（定义 Z 轴为“上”
    );

    // Start the visualization loop
    while (!viewer.wasStopped()) {
        for (auto& point_cloud : result_clouds) {
            viewer.updatePointCloud(point_cloud, "cloud");
            viewer.spinOnce(100);  // Update the viewer
        }
    }
    return 0;
}
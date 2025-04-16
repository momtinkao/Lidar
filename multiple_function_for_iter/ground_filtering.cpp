#define PCL_NO_PRECOMPILE
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iomanip>  // for setw, setfill
#include <iostream>

struct CustomPoint {
    PCL_ADD_POINT4D;  // Add standard XYZ + padding
    uint32_t frame_id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;  // Ensure proper alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(
    CustomPoint,
    (float, x, x)(float, y, y)(float, z, z)(uint32_t, frame_id, frame_id))

int main(int argc, char** argv) {
    pcl::PointCloud<CustomPoint>::Ptr original(new pcl::PointCloud<CustomPoint>);

    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read(argv[1], *original);  // Remember to download the file first!

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<CustomPoint> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.5);

    seg.setInputCloud(original);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
        return (-1);
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;

    pcl::PointCloud<CustomPoint>::Ptr cloud_f(new pcl::PointCloud<CustomPoint>());
    pcl::PointCloud<CustomPoint>::Ptr cloud_plane(new pcl::PointCloud<CustomPoint>());
    pcl::PCDWriter writer;
    pcl::ExtractIndices<CustomPoint> extract;
    extract.setInputCloud(original);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_plane);
    cloud_plane->width = cloud_plane->points.size();
    cloud_plane->height = 1;
    cloud_plane->is_dense = true;
    writer.write<CustomPoint>("plane.pcd", *cloud_plane, false);
    extract.setNegative(true);
    extract.filter(*cloud_f);
    writer.write<CustomPoint>("without_plane.pcd", *cloud_f, false);
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // tree->setInputCloud(cloud);
    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // ec.setClusterTolerance(0.5);
    // ec.setMinClusterSize(10);
    // ec.setMaxClusterSize(200);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(cloud_f);
    // ec.extract(cluster_indices);
    // std::cout << "cluster size: " << cluster_indices.size() << endl;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    // for (int i = 0; i < cloud_f->size(); i++) {
    //     pcl::PointXYZRGB point;
    //     point.x = cloud_f->points[i].x;
    //     point.y = cloud_f->points[i].y;
    //     point.z = cloud_f->points[i].z;
    //     point.r = 255;
    //     point.g = 255;
    //     point.b = 255;
    //     cloud_rgb->points.push_back(point);
    // }
    // for (const auto& cluster : cluster_indices) {
    //     for (const auto& idx : cluster.indices) {
    //         pcl::PointXYZRGB point;
    //         cloud_rgb->points[idx].r = 255;
    //         cloud_rgb->points[idx].g = 0;
    //         cloud_rgb->points[idx].b = 0;
    //     }
    // }
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_without_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
    // *cloud_rgb_without_plane = *cloud_rgb;
    // cloud_rgb_without_plane->width = cloud_rgb_without_plane->points.size();
    // cloud_rgb_without_plane->height = 1;
    // cloud_rgb_without_plane->is_dense = true;
    // writer.write<pcl::PointXYZRGB>("cluster_obj_without_plane.pcd", *cloud_rgb_without_plane, false);
    // for (int i = 0; i < cloud_plane->size(); i++) {
    //     pcl::PointXYZRGB point;
    //     point.x = cloud_plane->points[i].x;
    //     point.y = cloud_plane->points[i].y;
    //     point.z = cloud_plane->points[i].z;
    //     point.r = 0;
    //     point.g = 0;
    //     point.b = 255;
    //     cloud_rgb->points.push_back(point);
    // }
    // cloud_rgb->width = cloud_rgb->points.size();
    // cloud_rgb->height = 1;
    // cloud_rgb->is_dense = true;
    // writer.write<pcl::PointXYZRGB>("cluster_obj.pcd", *cloud_rgb, false);
    // std::cout << "cluster_size: " << cluster_indices.size() << endl;
    /*
    pcl::visualization::PCLVisualizer viewer("RGB PCD Viewer");

    // Add the point cloud to the visualizer
    viewer.addPointCloud<pcl::PointXYZ>(cloud->makeShared(), "cloud");

    // Set background color of the viewer (optional)
    viewer.setBackgroundColor(0.0, 0.0, 0.0);  // Black background

    // Set the size of the points (optional)
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer.initCameraParameters();
    viewer.setCameraPosition(10, 0, 0,   // 相機位置：Z 軸正方向，離原點 10 個單位
    0, 0, 0,    // 看向的目標點：原點 (0, 0, 0)
    0, 0, 1);


    // Start the visualization loop
    while (!viewer.wasStopped()) {
        viewer.spinOnce();  // Update the viewer
    }
    */

    return (0);
}
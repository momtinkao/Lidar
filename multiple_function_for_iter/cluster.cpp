#define PCL_NO_PRECOMPILE
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
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
#include <time.h>

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
    uint32_t current_frame_id = 0;
    pcl::PointCloud<CustomPoint>::Ptr original(new pcl::PointCloud<CustomPoint>);
    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    reader.read(argv[1], *original);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.3);
    ec.setMinClusterSize(600);
    ec.setMaxClusterSize(1500);
    std::vector<std::vector<pcl::PointIndices>> cluster_indices_a;
    pcl::PointCloud<pcl::PointXYZRGB> cluster_cloud;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> cluster_clouds;
    clock_t a, b;
    for (auto& p : original->points) {
        pcl::PointXYZRGB point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.r = 255;
        point.g = 255;
        point.b = 255;
        if (current_frame_id != p.frame_id) {
            if (current_frame_id == 0) {
                current_frame_id = p.frame_id;
                continue;
            } else {
                a = clock();
                std::vector<pcl::PointIndices> cluster_indices;
                cluster_clouds.emplace_back(cluster_cloud);
                tree->setInputCloud(cluster_cloud.makeShared());
                ec.setSearchMethod(tree);
                ec.setInputCloud(cluster_cloud.makeShared());
                ec.extract(cluster_indices);
                b = clock();
                std::cout << "Cluster cost " << double(b - a) / CLOCKS_PER_SEC << "seconds" << std::endl;
                std::cout << "cluster cloud size: " << cluster_indices.size() << std::endl;
                cluster_indices_a.push_back(cluster_indices);
            }
            current_frame_id = p.frame_id;
            cluster_cloud.clear();
        }
        cluster_cloud.emplace_back(point);
    }
    int i = 0;
    for (const auto& cluster_indices_t : cluster_indices_a) {
        for (const auto& cluster : cluster_indices_t) {
            pcl::PointCloud<pcl::PointXYZ> cluster_c;
            for (const auto& idx : cluster.indices) {
                cluster_clouds[i][idx].g = 0;
                cluster_clouds[i][idx].b = 0;
                // pcl::PointXYZ point;
                // point.x = cluster_clouds[i][idx].x;
                // point.y = cluster_clouds[i][idx].y;
                // point.z = cluster_clouds[i][idx].z;
                // cluster_c.push_back(point);
            }
            // pcl::PointXYZ minPoint, maxPoint;
            // Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
            // float theta = M_PI / 9;
            // transform_1(1, 1) = cos(theta);
            // transform_1(1, 2) = -sin(theta);
            // transform_1(2, 1) = sin(theta);
            // transform_1(2, 2) = cos(theta);
            // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            // pcl::transformPointCloud(*cluster_c.makeShared(), *transformed_cloud, transform_1);
            // pcl::getMinMax3D(*transformed_cloud, minPoint, maxPoint);
            // float length = maxPoint.y - minPoint.y;
            // float width = maxPoint.z - minPoint.z;
            // if ((length / width) >= 0.6 && (length / width) <= 1) {
            //     for (const auto& idx : cluster.indices) {
            //         cluster_clouds[i][idx].g = 0;
            //         cluster_clouds[i][idx].b = 0;
            //     }
            // }
        }
        i++;
    }
    pcl::visualization::PCLVisualizer viewer("RGB PCD Viewer");

    // Add the point cloud to the visualizer
    viewer.addPointCloud<pcl::PointXYZRGB>(cluster_clouds[1].makeShared(), "cloud");

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
    viewer.addCoordinateSystem(1.0);

    // Start the visualization loop
    while (!viewer.wasStopped()) {
        for (auto& point_cloud : cluster_clouds) {
            viewer.updatePointCloud(point_cloud.makeShared(), "cloud");
            viewer.spinOnce(1000);  // Update the viewer
        }
    }
    return 0;
}
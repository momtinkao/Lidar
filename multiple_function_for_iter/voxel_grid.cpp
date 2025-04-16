#define PCL_NO_PRECOMPILE
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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
    pcl::PointCloud<CustomPoint> clouds_filtered;

    // Fill in the cloud data
    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    // Replace the path below with the path where you saved your file
    reader.read(argv[1], cloud);  // Remember to download the file first!
    pcl::PointCloud<CustomPoint> t_cloud;
    uint32_t current_frame_id = 0;
    for (auto& point : cloud) {
        CustomPoint current_point;
        current_point.x = point.x;
        current_point.y = point.y;
        current_point.z = point.z;
        current_point.frame_id = point.frame_id;
        if (point.frame_id != current_frame_id) {
            std::cout << "current frame id: " << point.frame_id << " stored frame id:" << current_frame_id << std::endl;
            pcl::PointCloud<CustomPoint> filter_cloud;
            if (current_frame_id == 0) {
                current_frame_id = point.frame_id;
                continue;
            } else {
                std::cout << "PointCloud before filtering: " << t_cloud.size() << " data points" << std::endl;
                pcl::VoxelGrid<CustomPoint> sor;
                sor.setInputCloud(t_cloud.makeShared());
                sor.setLeafSize(0.07f, 0.2f, 0.2f);
                sor.setDownsampleAllData(true);  // don't average unknown fields
                sor.filter(filter_cloud);
                for (auto& p_ : filter_cloud) {
                    p_.frame_id = current_frame_id;
                }
                clouds_filtered += filter_cloud;
                std::cout << "PointCloud after filtering: " << filter_cloud.width * filter_cloud.height << " data points" << std::endl;
            }
            current_frame_id = point.frame_id;
            t_cloud.clear();
        }
        t_cloud.emplace_back(current_point);
    }
    writer.write("downsampled.pcd", clouds_filtered, false);

    return (0);
}
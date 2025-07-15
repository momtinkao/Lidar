/* obstacle_detector.hpp

 * Copyright (C) 2021 SS47816

 * Implementation of 3D LiDAR Obstacle Detection & Tracking Algorithms

**/

#pragma once

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <ctime>
#include <iostream>
#include <limits>  // Required for std::numeric_limits
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

// Assuming box.hpp is in the same directory as obstacle_detector.hpp
// (e.g., both are in your project's "include" folder)
#include "box.hpp"

namespace lidar_obstacle_detector {
template <typename PointT>
class ObstacleDetector {
   public:
    ObstacleDetector();
    virtual ~ObstacleDetector();

    // ****************** Detection ***********************

    typename pcl::PointCloud<PointT>::Ptr filterCloud(
        const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
        const float filter_res, const Eigen::Vector4f &min_pt,
        const Eigen::Vector4f &max_pt);

    std::pair<typename pcl::PointCloud<PointT>::Ptr,
              typename pcl::PointCloud<PointT>::Ptr>
    segmentPlane(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                 const int max_iterations, const float distance_thresh);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clustering(
        const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
        const float cluster_tolerance, const int min_size, const int max_size);

    Box axisAlignedBoundingBox(
        const typename pcl::PointCloud<PointT>::ConstPtr &cluster, const int id);

    Box pcaBoundingBox(const typename pcl::PointCloud<PointT>::Ptr &cluster,
                       const int id);

    // ****************** Tracking ***********************
    void obstacleTracking(const std::vector<Box> &prev_boxes,
                          std::vector<Box> *curr_boxes,
                          const float displacement_thresh,
                          const float iou_thresh);

   private:
    // ****************** Detection ***********************
    std::pair<typename pcl::PointCloud<PointT>::Ptr,
              typename pcl::PointCloud<PointT>::Ptr>
    separateClouds(const pcl::PointIndices::ConstPtr &inliers,
                   const typename pcl::PointCloud<PointT>::ConstPtr &cloud);

    // ****************** Tracking ***********************
    bool compareBoxes(const Box &a, const Box &b, const float displacement_thresh,
                      const float iou_thresh);

    std::vector<std::vector<int>> associateBoxes(
        const std::vector<Box> &prev_boxes, const std::vector<Box> &curr_boxes,
        const float displacement_thresh, const float iou_thresh);

    std::vector<std::vector<int>> connectionMatrix(
        const std::vector<std::vector<int>> &connection_pairs,
        std::vector<int> *left, std::vector<int> *right);

    // Declaration (uses pointers)
    bool hungarianFind(const int i,
                       const std::vector<std::vector<int>> &connection_matrix,
                       std::vector<bool> *right_connected_marker,
                       std::vector<int> *right_pair);

    std::vector<int> hungarian(
        const std::vector<std::vector<int>> &connection_matrix);

    int searchBoxIndex(const std::vector<Box> &Boxes, const int id);
};

// constructor:
template <typename PointT>
ObstacleDetector<PointT>::ObstacleDetector() {}

// de-constructor:
template <typename PointT>
ObstacleDetector<PointT>::~ObstacleDetector() {}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ObstacleDetector<PointT>::filterCloud(
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
    const float filter_res, const Eigen::Vector4f &min_pt,
    const Eigen::Vector4f &max_pt) {
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    if (!cloud || cloud->empty()) return cloud_filtered;  // Return empty if input is null or empty

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.07f, filter_res, filter_res);
    vg.filter(*cloud_filtered);

    if (cloud_filtered->empty()) return cloud_filtered;

    typename pcl::PointCloud<PointT>::Ptr cloud_roi(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(min_pt);
    region.setMax(max_pt);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_roi);

    if (cloud_roi->empty()) return cloud_roi;

    std::vector<int> indices_roof;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5f, -1.7f, -1.0f, 1.0f));
    roof.setMax(Eigen::Vector4f(2.6f, 1.7f, -0.4f, 1.0f));
    roof.setInputCloud(cloud_roi);
    roof.filter(indices_roof);

    if (!indices_roof.empty()) {
        pcl::PointIndices::Ptr inliers_roof(new pcl::PointIndices);
        inliers_roof->indices = indices_roof;

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_roi);
        extract.setIndices(inliers_roof);
        extract.setNegative(true);
        extract.filter(*cloud_roi);
    }

    return cloud_roi;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ObstacleDetector<PointT>::separateClouds(
    const pcl::PointIndices::ConstPtr &inliers,
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud) {
    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr ground_cloud(new pcl::PointCloud<PointT>());

    if (!cloud || cloud->empty() || !inliers) return {obstacle_cloud, ground_cloud};

    for (int index : inliers->indices) {
        if (index >= 0 && static_cast<size_t>(index) < cloud->points.size()) {
            ground_cloud->points.push_back(cloud->points[index]);
        }
    }
    if (!ground_cloud->points.empty()) {
        ground_cloud->width = ground_cloud->points.size();
        ground_cloud->height = 1;
        ground_cloud->is_dense = true;
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);

    return {obstacle_cloud, ground_cloud};
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ObstacleDetector<PointT>::segmentPlane(
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
    const int max_iterations, const float distance_thresh) {
    if (!cloud || cloud->empty() || cloud->points.size() < 3) {
        std::cerr << "Input cloud for segmentPlane is null, empty or too small." << std::endl;
        typename pcl::PointCloud<PointT>::Ptr empty_obstacles(new pcl::PointCloud<PointT>());
        typename pcl::PointCloud<PointT>::Ptr empty_ground(new pcl::PointCloud<PointT>());
        if (cloud && !cloud->empty()) *empty_obstacles = *cloud;
        return {empty_obstacles, empty_ground};
    }

    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers_seg{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iterations);
    seg.setDistanceThreshold(distance_thresh);

    seg.setInputCloud(cloud);
    seg.segment(*inliers_seg, *coefficients);
    if (inliers_seg->indices.empty()) {
        std::cout << "Could not estimate a planar model for the given dataset. Treating all points as obstacles." << std::endl;
        typename pcl::PointCloud<PointT>::Ptr all_obstacles(new pcl::PointCloud<PointT>(*cloud));
        typename pcl::PointCloud<PointT>::Ptr empty_ground(new pcl::PointCloud<PointT>());
        return {all_obstacles, empty_ground};
    }

    return separateClouds(inliers_seg, cloud);
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ObstacleDetector<PointT>::clustering(
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
    const float cluster_tolerance, const int min_size, const int max_size) {
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    if (!cloud || cloud->empty() || cloud->points.size() < static_cast<size_t>(min_size)) {
        return clusters;
    }

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto &getIndices : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        for (int index : getIndices.indices) {
            if (index >= 0 && static_cast<size_t>(index) < cloud->points.size()) {
                cluster->points.push_back(cloud->points[index]);
            }
        }
        if (!cluster->points.empty()) {
            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(cluster);
        }
    }
    return clusters;
}

template <typename PointT>
Box ObstacleDetector<PointT>::axisAlignedBoundingBox(
    const typename pcl::PointCloud<PointT>::ConstPtr &cluster, const int id) {
    if (!cluster || cluster->empty()) {
        return Box(id, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
    }
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);

    const Eigen::Vector3f position((max_pt.x + min_pt.x) / 2.0f,
                                   (max_pt.y + min_pt.y) / 2.0f,
                                   (max_pt.z + min_pt.z) / 2.0f);
    const Eigen::Vector3f dimension(std::max(0.0f, max_pt.x - min_pt.x),
                                    std::max(0.0f, max_pt.y - min_pt.y),
                                    std::max(0.0f, max_pt.z - min_pt.z));

    return Box(id, position, dimension);
}

template <typename PointT>
Box ObstacleDetector<PointT>::pcaBoundingBox(
    const typename pcl::PointCloud<PointT>::Ptr &cluster, const int id) {
    if (!cluster || cluster->empty()) {
        return Box(id, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity());
    }

    PointT global_min_pt, global_max_pt;
    pcl::getMinMax3D(*cluster, global_min_pt, global_max_pt);
    const float box_height = std::max(0.0f, global_max_pt.z - global_min_pt.z);

    Eigen::Vector4f pca_centroid_4d;
    pcl::compute3DCentroid(*cluster, pca_centroid_4d);
    Eigen::Vector3f pca_centroid = pca_centroid_4d.template head<3>();

    typename pcl::PointCloud<PointT>::Ptr cluster_for_pca(new pcl::PointCloud<PointT>(*cluster));
    for (size_t i = 0; i < cluster_for_pca->size(); ++i) {
        cluster_for_pca->points[i].z = pca_centroid(2);
    }

    if (cluster_for_pca->points.size() < 3) {
        std::cerr << "PCA Bounding Box: Cluster has less than 3 points after Z-squashing. Falling back to AABB." << std::endl;
        return axisAlignedBoundingBox(cluster, id);
    }

    typename pcl::PointCloud<PointT>::Ptr pca_projected_cloud(new pcl::PointCloud<PointT>);
    pcl::PCA<PointT> pca(true);
    pca.setInputCloud(cluster_for_pca);

    // Check if PCA computation is valid (eigenvalues are reasonable)
    // This check might need adjustment based on typical point cloud characteristics
    if (pca.getEigenValues().rows() < 2 || pca.getEigenValues()(0) < 1e-7 || pca.getEigenValues()(1) < 1e-7) {
        std::cerr << "PCA Bounding Box: Degenerate cluster for PCA (eigenvalues too small or insufficient). Falling back to AABB." << std::endl;
        return axisAlignedBoundingBox(cluster, id);
    }

    pca.project(*cluster_for_pca, *pca_projected_cloud);

    const auto eigen_vectors = pca.getEigenVectors();

    PointT min_pt_pca, max_pt_pca;
    pcl::getMinMax3D(*pca_projected_cloud, min_pt_pca, max_pt_pca);

    const Eigen::Vector3f meanDiagonal = 0.5f * (max_pt_pca.getVector3fMap() + min_pt_pca.getVector3fMap());

    const Eigen::Quaternionf quaternion(eigen_vectors);
    const Eigen::Vector3f position = eigen_vectors * meanDiagonal + pca_centroid;

    const Eigen::Vector3f dimension(std::max(0.0f, max_pt_pca.x - min_pt_pca.x),
                                    std::max(0.0f, max_pt_pca.y - min_pt_pca.y),
                                    box_height);

    return Box(id, position, dimension, quaternion);
}

// ************************* Tracking ***************************
template <typename PointT>
void ObstacleDetector<PointT>::obstacleTracking(
    const std::vector<Box> &prev_boxes, std::vector<Box> *curr_boxes,
    const float displacement_thresh, const float iou_thresh) {
    if (!curr_boxes || curr_boxes->empty() || prev_boxes.empty()) {
        return;
    }

    std::vector<int> pre_ids;
    std::vector<int> cur_ids;
    std::vector<int> matches;

    auto connection_pairs = associateBoxes(prev_boxes, *curr_boxes, displacement_thresh, iou_thresh);

    if (connection_pairs.empty()) return;

    auto connection_matrix = connectionMatrix(connection_pairs, &pre_ids, &cur_ids);

    if (connection_matrix.empty() || (!connection_matrix.empty() && connection_matrix[0].empty())) return;

    matches = hungarian(connection_matrix);

    for (size_t j = 0; j < matches.size(); ++j) {
        if (matches[j] != -1 && static_cast<size_t>(matches[j]) < pre_ids.size() && j < cur_ids.size()) {
            const int prev_box_original_id = pre_ids[matches[j]];
            const int prev_box_current_idx = searchBoxIndex(prev_boxes, prev_box_original_id);

            const int curr_box_original_id = cur_ids[j];
            const int curr_box_current_idx = searchBoxIndex(*curr_boxes, curr_box_original_id);

            if (prev_box_current_idx > -1 && curr_box_current_idx > -1) {
                (*curr_boxes)[curr_box_current_idx].id = prev_boxes[prev_box_current_idx].id;
            }
        }
    }
}

template <typename PointT>
bool ObstacleDetector<PointT>::compareBoxes(const Box &a, const Box &b,
                                            const float displacement_thresh,
                                            const float iou_thresh) {
    const float dist_sq = (a.position - b.position).squaredNorm();
    const float dist = std::sqrt(dist_sq);

    const float a_max_dim = std::max({a.dimension[0], a.dimension[1], a.dimension[2]});
    const float b_max_dim = std::max({b.dimension[0], b.dimension[1], b.dimension[2]});

    float ctr_dis_metric = std::numeric_limits<float>::max();
    if (std::min(a_max_dim, b_max_dim) > 1e-6) {
        ctr_dis_metric = dist / std::min(a_max_dim, b_max_dim);
    } else if (dist < 1e-6) {
        ctr_dis_metric = 0.0f;
    }

    auto dim_similarity = [&](float dim_a, float dim_b) {
        if (std::abs(dim_a + dim_b) < 1e-6) {
            return (std::abs(dim_a - dim_b) < 1e-6) ? 0.0f : 2.0f;
        }
        // Ensure dimensions are non-negative before division
        float non_neg_dim_a = std::max(0.0f, dim_a);
        float non_neg_dim_b = std::max(0.0f, dim_b);
        if (std::abs(non_neg_dim_a + non_neg_dim_b) < 1e-6) {  // if sum is still zero
            return (std::abs(non_neg_dim_a - non_neg_dim_b) < 1e-6) ? 0.0f : 2.0f;
        }
        return std::abs(non_neg_dim_a - non_neg_dim_b) / (non_neg_dim_a + non_neg_dim_b);
    };

    const float x_dim_sim = dim_similarity(a.dimension[0], b.dimension[0]);
    const float y_dim_sim = dim_similarity(a.dimension[1], b.dimension[1]);
    const float z_dim_sim = dim_similarity(a.dimension[2], b.dimension[2]);

    if (ctr_dis_metric <= displacement_thresh && x_dim_sim <= iou_thresh &&
        y_dim_sim <= iou_thresh && z_dim_sim <= iou_thresh) {
        return true;
    }
    return false;
}

template <typename PointT>
std::vector<std::vector<int>> ObstacleDetector<PointT>::associateBoxes(
    const std::vector<Box> &prev_boxes, const std::vector<Box> &curr_boxes,
    const float displacement_thresh, const float iou_thresh) {
    std::vector<std::vector<int>> connection_pairs;
    for (const auto &prev_box : prev_boxes) {
        for (const auto &cur_box : curr_boxes) {
            if (this->compareBoxes(cur_box, prev_box, displacement_thresh, iou_thresh)) {
                connection_pairs.push_back({prev_box.id, cur_box.id});
            }
        }
    }
    return connection_pairs;
}

template <typename PointT>
std::vector<std::vector<int>> ObstacleDetector<PointT>::connectionMatrix(
    const std::vector<std::vector<int>> &connection_pairs,
    std::vector<int> *left_ids, std::vector<int> *right_ids) {
    if (!left_ids || !right_ids) return {};  // Should not happen if called correctly

    left_ids->clear();
    right_ids->clear();

    std::unordered_set<int> left_set, right_set;
    for (const auto &pair : connection_pairs) {
        left_set.insert(pair[0]);
        right_set.insert(pair[1]);
    }
    left_ids->assign(left_set.begin(), left_set.end());
    right_ids->assign(right_set.begin(), right_set.end());

    std::vector<std::vector<int>> connection_matrix(
        left_ids->size(), std::vector<int>(right_ids->size(), 0));

    for (const auto &pair : connection_pairs) {
        auto it_left = std::find(left_ids->begin(), left_ids->end(), pair[0]);
        auto it_right = std::find(right_ids->begin(), right_ids->end(), pair[1]);

        if (it_left != left_ids->end() && it_right != right_ids->end()) {
            connection_matrix[std::distance(left_ids->begin(), it_left)]
                             [std::distance(right_ids->begin(), it_right)] = 1;
        }
    }
    return connection_matrix;
}

// Definition (MODIFIED to use pointers to match declaration)
template <typename PointT>
bool ObstacleDetector<PointT>::hungarianFind(
    const int i,  // Index for the "left" set (prev_boxes)
    const std::vector<std::vector<int>> &connection_matrix,
    std::vector<bool> *right_connected_marker,  // Pointer to visited status for "right" set (curr_boxes)
    std::vector<int> *right_pair) {             // Pointer to matching for "right" set: right_pair[v_idx] = u_idx

    if (!right_connected_marker || !right_pair || connection_matrix.empty()) return false;
    if (static_cast<size_t>(i) >= connection_matrix.size()) return false;  // Bounds check for i

    for (size_t v_idx = 0; v_idx < connection_matrix[0].size(); ++v_idx) {
        if (static_cast<size_t>(v_idx) >= right_connected_marker->size() || static_cast<size_t>(v_idx) >= right_pair->size()) continue;  // Bounds check

        if (connection_matrix[i][v_idx] == 1 && !(*right_connected_marker)[v_idx]) {
            (*right_connected_marker)[v_idx] = true;

            // If v_idx is not matched OR an augmenting path is found from its current match
            if ((*right_pair)[v_idx] == -1 ||
                hungarianFind((*right_pair)[v_idx], connection_matrix, right_connected_marker, right_pair)) {
                (*right_pair)[v_idx] = i;  // Match i with v_idx
                return true;
            }
        }
    }
    return false;
}

template <typename PointT>
std::vector<int> ObstacleDetector<PointT>::hungarian(
    const std::vector<std::vector<int>> &connection_matrix) {
    if (connection_matrix.empty() || connection_matrix[0].empty()) return {};

    size_t num_left = connection_matrix.size();      // Typically prev_boxes
    size_t num_right = connection_matrix[0].size();  // Typically curr_boxes

    // match_for_right[j] stores the index of the left_box that right_box j is matched with, or -1 if unmatched.
    std::vector<int> match_for_right(num_right, -1);
    int matches_count = 0;

    for (size_t u_idx = 0; u_idx < num_left; ++u_idx) {
        // For each DFS starting from u_idx, we need a fresh "visited" marker for the right set
        std::vector<bool> visited_right_dfs(num_right, false);
        if (hungarianFind(u_idx, connection_matrix, &visited_right_dfs, &match_for_right)) {
            matches_count++;
        }
    }

    std::cout << "Hungarian Algorithm: Matched " << matches_count
              << " pairs. (Left set size: " << num_left << ", Right set size: " << num_right << ")." << std::endl;

    return match_for_right;
}

template <typename PointT>
int ObstacleDetector<PointT>::searchBoxIndex(const std::vector<Box> &boxes, const int id) {
    for (size_t i = 0; i < boxes.size(); ++i) {
        if (boxes[i].id == id) return static_cast<int>(i);
    }
    return -1;
}

}  // namespace lidar_obstacle_detector

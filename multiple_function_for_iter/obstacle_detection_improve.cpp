#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
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

#include <algorithm>
#include <chrono>  // 用於計時
#include <cmath>
#include <deque>  // 用於快取管理
#include <iomanip>
#include <iostream>
#include <map>     // 用於快取
#include <memory>  // 用於 std::shared_ptr
#include <string>
#include <vector>

// 假設 obstacle_detector.hpp 和 box.hpp 在 include 資料夾下
#include "include/obstacle_detector.hpp"

// 用於儲存每一幀的數據
struct FrameData {
    uint32_t id;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_obstacle_points_viz;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr non_clustered_points_viz;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_points_viz;
    std::vector<Box> boxes_viz;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_viz;

    FrameData() : clustered_obstacle_points_viz(new pcl::PointCloud<pcl::PointXYZRGB>),
                  non_clustered_points_viz(new pcl::PointCloud<pcl::PointXYZRGB>),
                  ground_points_viz(new pcl::PointCloud<pcl::PointXYZRGB>),
                  raw_cloud_viz(new pcl::PointCloud<pcl::PointXYZ>) {}
};

// --- 全域變數 ---
pcl::visualization::PCLVisualizer::Ptr viewer;
pcl::visualization::PCLVisualizer::Ptr viewer_voxel_grid;
pcl::visualization::PCLVisualizer::Ptr viewer_ground_filtering;
pcl::visualization::PCLVisualizer::Ptr viewer_original;
pcl::visualization::PCLVisualizer::Ptr viewer_cluster;

std::string pcd_directory;
std::string filename_prefix = "0-";  // 您的檔案前綴
uint32_t total_frames = 0;
size_t current_frame_idx = 0;

// 快取機制
const size_t CACHE_SIZE = 20;  // 快取中最多保留的幀數
std::map<uint32_t, std::shared_ptr<FrameData>> frame_cache;
std::deque<uint32_t> cache_order;

// 視覺化與播放控制
bool frame_update_required = true;
bool paused = false;
bool show_ground = true;
bool show_non_clustered = true;
bool auto_play = false;  // 自動播放狀態

// 汽車分類函式
bool classifyBoxAsCar(Box& box, int num_points_in_cluster) {
    const float MIN_CAR_HEIGHT = 1.5f, MAX_CAR_HEIGHT = 2.2f;
    const float MIN_CAR_WIDTH = 1.0f, MAX_CAR_WIDTH = 2.0f;
    const float MIN_CAR_LENGTH = 2.0f, MAX_CAR_LENGTH = 10.0f;
    const int MIN_POINTS_FOR_CAR = 200;

    float height = box.dimension.z();
    float length = std::max(box.dimension.x(), box.dimension.y());
    float width = std::min(box.dimension.x(), box.dimension.y());
    std::cout << "ID:" << box.id << "Length:  " << length << ", Width:" << width << ", Height: " << height << std::endl;
    if (length / width > 0.8 && length / width <= 5 && height < 2.5 && height > 1 && length > 2.5) {
        box.is_classified_as_car = true;
        std::cout << "Car Length:  " << length << ", Car Width:" << width << ", Car Height: " << height << std::endl;
        return true;
    }
    if (length / width > 0.8 && length / width <= 5 && height < 2 && height > 0.8 && length > 1 && length <= 2.5) {
        std::cout << "Motor Length:  " << length << ", Motor Width:" << width << ", Motor Height: " << height << std::endl;
        box.is_classified_as_motor = true;
        return true;
    }
    box.is_classified_as_motor = false;
    box.is_classified_as_car = false;

    return false;
}

void addDistanceMarkers(pcl::visualization::PCLVisualizer::Ptr& vis) {
    std::cout << "正在加入旋轉後的距離標記..." << std::endl;

    // --- 1. 定義旋轉 ---
    // 順時鐘 45 度，圍繞 Z 軸 (垂直向上) 旋轉。
    // 在數學中，繞 Z 軸逆時針為正角度，因此順時鐘是負角度。
    const float angle_rad = 50.0f * M_PI / 180.0f;
    const float angle_rad2 = -35 * M_PI / 180.0f;
    Eigen::AngleAxisf rotation_vector(angle_rad, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotation_vector2(angle_rad2, Eigen::Vector3f::UnitX());
    Eigen::Quaternionf rotation_quaternion(rotation_vector);
    Eigen::Quaternionf rotation_quaternion2(rotation_vector2);
    Eigen::Vector3f forward_vector(0.0f, 0.0f, 1.0f);
    // --- 2. 設定標記外觀 ---
    // 尺寸定義：長、寬、高。
    const float marker_length = 0.2f;  // 方塊沿著行進方向的厚度
    const float marker_width = 8.0f;   // 方塊橫跨道路的寬度
    const float marker_height = 0.2f;  // 方塊本身的高度
    const float road_level_z = -6.0f;  // 標記所在的 Z 高度 (垂直位置)

    // --- 3. 沿著旋轉後的軸線，每隔 10 公尺建立一個標記 ---
    Eigen::Vector3f base_position(road_level_z, 10.0f, 0);
    for (float dist = 20.0f; dist <= 100.0f; dist += 10.0f) {
        // a. 計算標記的中心點位置
        // 首先，定義一個在旋轉前、沿著 -X 軸 (我們定義的"前方") 的點
        // 然後，使用旋轉物件計算這個點旋轉後的新位置
        Eigen::Vector3f point_on_path = base_position + dist * forward_vector;
        Eigen::Vector3f rotated_position = rotation_quaternion * point_on_path;

        // b. 加入旋轉後的方塊 (Bounding Box)
        std::string box_id = "marker_box_" + std::to_string(static_cast<int>(dist));
        // addCube 函式讓我們直接傳入旋轉後的位置和旋轉本身
        vis->addCube(rotated_position, rotation_quaternion, marker_height, marker_width, marker_length, box_id);
        vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, box_id);
        vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, box_id);  // 青色 (Cyan)
        vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, box_id);

        // c. 在標記旁加入文字
        std::string text_content = std::to_string(static_cast<int>(dist)) + " m";
        std::string text_id = "marker_text_" + std::to_string(static_cast<int>(dist));

        // 將文字放在方塊的一側 "旁邊"
        // 我們在方塊的局部座標系中定義一個偏移量 (沿著它的 Y 軸，即寬度方向)
        Eigen::Vector3f text_offset(0.0f, marker_width / 2.0f + 0.5f, 1.0f);  // 沿寬度方向偏移，並提高一點高度
        // 用同樣的旋轉來變換這個偏移量，並加到方塊的中心點上，得到文字在世界座標中的最終位置
        Eigen::Vector3f text_position_eigen = rotated_position + rotation_quaternion * text_offset;

        pcl::PointXYZ text_pcl_position(text_position_eigen.x(), text_position_eigen.y(), text_position_eigen.z());
        vis->addText3D(text_content, text_pcl_position, 0.8, 1.0, 1.0, 1.0, text_id);  // 白色文字
    }
    Eigen::Vector3f base_position2(road_level_z, -5.0f, 0);
    for (float dist = 20.0f; dist <= 100.0f; dist += 10.0f) {
        // a. 計算標記的中心點位置
        // 首先，定義一個在旋轉前、沿著 -X 軸 (我們定義的"前方") 的點
        // 然後，使用旋轉物件計算這個點旋轉後的新位置
        Eigen::Vector3f point_on_path = base_position2 + dist * forward_vector;
        Eigen::Vector3f rotated_position = rotation_quaternion2 * point_on_path;

        // b. 加入旋轉後的方塊 (Bounding Box)
        std::string box_id = "marker_box_2" + std::to_string(static_cast<int>(dist));
        // addCube 函式讓我們直接傳入旋轉後的位置和旋轉本身
        vis->addCube(rotated_position, rotation_quaternion2, marker_height, marker_width, marker_length, box_id);
        vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, box_id);
        vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, box_id);  // 青色 (Cyan)
        vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, box_id);

        // c. 在標記旁加入文字
        std::string text_content = std::to_string(static_cast<int>(dist)) + " m";
        std::string text_id = "marker_text_2" + std::to_string(static_cast<int>(dist));

        // 將文字放在方塊的一側 "旁邊"
        // 我們在方塊的局部座標系中定義一個偏移量 (沿著它的 Y 軸，即寬度方向)
        Eigen::Vector3f text_offset(0.0f, marker_width / 2.0f + 0.5f, 1.0f);  // 沿寬度方向偏移，並提高一點高度
        // 用同樣的旋轉來變換這個偏移量，並加到方塊的中心點上，得到文字在世界座標中的最終位置
        Eigen::Vector3f text_position_eigen = rotated_position + rotation_quaternion2 * text_offset;

        pcl::PointXYZ text_pcl_position(text_position_eigen.x(), text_position_eigen.y(), text_position_eigen.z());
        vis->addText3D(text_content, text_pcl_position, 0.8, 1.0, 1.0, 1.0, text_id);  // 白色文字
    }
}

// 從磁碟載入並處理單一幀的完整流程
std::shared_ptr<FrameData> loadAndProcessFrame_update(uint32_t frame_id) {
    // --- 1. 讀取檔案 ---
    std::string pcd_file_path = pcd_directory + "/" + filename_prefix + std::to_string(frame_id) + ".pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr frame_cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    if (reader.read(pcd_file_path, *frame_cloud_raw) == -1) {
        PCL_ERROR("無法讀取幀 %u 的 PCD 檔案: %s\n", frame_id, pcd_file_path.c_str());
        return nullptr;
    }
    if (frame_cloud_raw->empty()) {
        std::cout << "警告: 幀 " << frame_id << " 的點雲為空。" << std::endl;
        return std::make_shared<FrameData>();
    }

    std::cout << "\n[處理程序] 正在載入並處理幀 ID: " << frame_id << ", 原始點數: " << frame_cloud_raw->size() << std::endl;

    auto frame_data = std::make_shared<FrameData>();
    frame_data->id = frame_id;
    pcl::copyPointCloud(*frame_cloud_raw, *frame_data->raw_cloud_viz);

    // --- 2. 設定處理參數 ---
    const float VOXEL_GRID_SIZE = 0.2f;
    const int MAX_PLANE_ITERATIONS = 100;
    const float GROUND_THRESH = 0.25f;
    const float CLUSTER_THRESH = 0.7f;
    const int CLUSTER_MIN_SIZE = 10;
    const int CLUSTER_MAX_SIZE = 10000;

    // --- 3. 座標系轉換 ---
    Eigen::Affine3f transform_old_to_new = Eigen::Affine3f::Identity();
    // New X' = Old Z (Forward)
    transform_old_to_new(0, 0) = 0;
    transform_old_to_new(0, 1) = 0;
    transform_old_to_new(0, 2) = 1;
    // New Y' = -Old Y (Left)
    transform_old_to_new(1, 0) = 0;
    transform_old_to_new(1, 1) = -1;
    transform_old_to_new(1, 2) = 0;
    // New Z' = Old X (Up)
    transform_old_to_new(2, 0) = 1;
    transform_old_to_new(2, 1) = 0;
    transform_old_to_new(2, 2) = 0;
    Eigen::Affine3f transform_new_to_old = transform_old_to_new.inverse();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*frame_cloud_raw, *original_rgb_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_proc_cs(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*original_rgb_cloud, *cloud_proc_cs, transform_old_to_new);

    // --- 4. 預處理：體素下採樣 ---
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    vg.setInputCloud(cloud_proc_cs);
    vg.setLeafSize(VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE);
    vg.filter(*cloud_downsampled);
    pcl::transformPointCloud(*cloud_downsampled, *frame_data->non_clustered_points_viz, transform_new_to_old);

    // --- 5. ROI 濾波：定義多個可旋轉區域並合併 ---
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_roi_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // 為了清晰和安全，我們為每個斜向車道建立獨立的濾波器和臨時點雲
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_pos45(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_neg45(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_front(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::CropBox<pcl::PointXYZRGB> front;
    front.setInputCloud(cloud_downsampled);
    front.setMin(Eigen::Vector4f(3, -5, -12.0f, 1.0f));
    front.setMax(Eigen::Vector4f(22, 5, -2.0f, 1.0f));
    front.filter(*temp_cloud_front);
    *combined_roi_cloud += *temp_cloud_front;

    // --- ROI 3a: +45度斜向車道 ---
    pcl::CropBox<pcl::PointXYZRGB> box_filter_pos45;
    box_filter_pos45.setInputCloud(cloud_downsampled);  // *** 關鍵：設定輸入為原始的下採樣點雲

    // a. 定義旋轉
    Eigen::Affine3f transform_pos45 = Eigen::Affine3f::Identity();
    float angle_pos45_rad = (30.0f * M_PI / 180.0f);
    transform_pos45.rotate(Eigen::AngleAxisf(angle_pos45_rad, Eigen::Vector3f::UnitZ()));  // *** 修正：UnitZ()
    box_filter_pos45.setTransform(transform_pos45);

    // b. 定義車道在「旋轉前」的尺寸
    float lane_length = 250.0f;
    float lane_width = 10.0f;
    // 注意：您的 Z 軸範圍是 (-6.0, -2.0)，這代表只偵測光達「下方」2米到6米的物體
    box_filter_pos45.setMin(Eigen::Vector4f(-20, -4.0f, -12.0f, 1.0f));
    box_filter_pos45.setMax(Eigen::Vector4f(lane_length, 8.0f, -2.0f, 1.0f));

    // c. 執行濾波並將結果合併
    box_filter_pos45.filter(*temp_cloud_pos45);
    *combined_roi_cloud += *temp_cloud_pos45;

    // --- ROI 3b: -45度斜向車道 ---
    pcl::CropBox<pcl::PointXYZRGB> box_filter_neg45;
    box_filter_neg45.setInputCloud(cloud_downsampled);  // *** 關鍵：同樣設定輸入為原始的下採樣點雲

    // a. 定義旋轉
    Eigen::Affine3f transform_neg45 = Eigen::Affine3f::Identity();
    float angle_neg45_rad = (-45.0f * M_PI / 180.0f);
    transform_neg45.rotate(Eigen::AngleAxisf(angle_neg45_rad, Eigen::Vector3f::UnitZ()));  // *** 修正：UnitZ()
    box_filter_neg45.setTransform(transform_neg45);

    // b. 定義車道在「旋轉前」的尺寸 (使用與上面相同的尺寸)
    box_filter_neg45.setMin(Eigen::Vector4f(-20, -11, -12.0f, 1.0f));
    box_filter_neg45.setMax(Eigen::Vector4f(lane_length, -5, -2.0f, 1.0f));

    // c. 執行濾波並將結果合併
    box_filter_neg45.filter(*temp_cloud_neg45);
    *combined_roi_cloud += *temp_cloud_neg45;

    // --- 所有 ROI 區域濾波與合併結束 ---

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud = combined_roi_cloud;

    if (filtered_cloud->empty()) {
        std::cout << "  ROI 區域內無有效點。" << std::endl;
        return frame_data;
    }

    // --- 6. 後續處理：地面分割、分群、物件匡選 ---
    lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZRGB> obstacle_detector;

    auto segmented_clouds = obstacle_detector.segmentPlane(filtered_cloud, MAX_PLANE_ITERATIONS, GROUND_THRESH);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacles_cloud = segmented_clouds.first;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud = segmented_clouds.second;

    if (!ground_cloud->empty()) {
        pcl::transformPointCloud(*ground_cloud, *frame_data->ground_points_viz, transform_new_to_old);
    }
    if (obstacles_cloud->empty()) {
        std::cout << "  地面分割後無障礙物點。" << std::endl;
        return frame_data;
    }

    auto clusters = obstacle_detector.clustering(obstacles_cloud, CLUSTER_THRESH, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_clusters_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    static int global_box_id_counter = 0;

    for (const auto& cluster : clusters) {
        if (cluster->empty()) continue;

        unsigned char r = rand() % 200 + 50;
        unsigned char g = rand() % 200 + 50;
        unsigned char b = rand() % 200 + 50;
        for (const auto& pt : cluster->points) {
            pcl::PointXYZRGB colored_pt = pt;
            colored_pt.r = r;
            colored_pt.g = g;
            colored_pt.b = b;
            all_clusters_colored->push_back(colored_pt);
        }
        time_t a, a_;
        a = clock();
        Box box_proc = obstacle_detector.pcaBoundingBox(cluster, global_box_id_counter++);
        a_ = clock();
        std::cout << "Bounding Box for car cost " << double(a_ - a) / CLOCKS_PER_SEC << std::endl;
        classifyBoxAsCar(box_proc, cluster->size());

        Box box_viz;
        box_viz.id = box_proc.id;
        box_viz.is_classified_as_car = box_proc.is_classified_as_car;
        box_viz.is_classified_as_motor = box_proc.is_classified_as_motor;
        box_viz.dimension = box_proc.dimension;
        Eigen::Affine3f pose_proc = Eigen::Translation3f(box_proc.position) * box_proc.quaternion;
        Eigen::Affine3f pose_viz = transform_new_to_old * pose_proc;
        box_viz.position = pose_viz.translation();
        box_viz.quaternion = Eigen::Quaternionf(pose_viz.rotation());
        frame_data->boxes_viz.push_back(box_viz);
    }

    if (!all_clusters_colored->empty()) {
        pcl::transformPointCloud(*all_clusters_colored, *frame_data->clustered_obstacle_points_viz, transform_new_to_old);
    }

    return frame_data;
}

// 鍵盤事件回呼函式
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* viewer_void) {
    if (event.keyDown()) {
        if (event.getKeySym() == "n") {
            if (total_frames > 0) {
                current_frame_idx = (current_frame_idx + 1) % total_frames;
                frame_update_required = true;
            }
        } else if (event.getKeySym() == "m") {
            if (total_frames > 0) {
                current_frame_idx = (current_frame_idx + 20) % total_frames;
                frame_update_required = true;
            }
        } else if (event.getKeySym() == "v") {
            if (total_frames > 0) {
                current_frame_idx = (current_frame_idx == 0) ? total_frames - 20 : current_frame_idx - 20;
                frame_update_required = true;
            }
        } else if (event.getKeySym() == "b") {
            if (total_frames > 0) {
                current_frame_idx = (current_frame_idx == 0) ? total_frames - 1 : current_frame_idx - 1;
                frame_update_required = true;
            }
        } else if (event.getKeySym() == "p") {
            paused = !paused;
            std::cout << (paused ? "\n視覺化已暫停" : "\n視覺化已恢復") << std::endl;
        } else if (event.getKeySym() == "g" || event.getKeySym() == "u") {
            if (event.getKeySym() == "g") show_ground = !show_ground;
            if (event.getKeySym() == "u") show_non_clustered = !show_non_clustered;
            frame_update_required = true;
        } else if (event.getKeySym() == "a") {
            auto_play = !auto_play;
            std::cout << "\n自動播放 " << (auto_play ? "已啟用" : "已停用") << std::endl;
        }
    }
}

// 主函式
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <input_pcd_directory>" << std::endl;
        return -1;
    }
    pcd_directory = argv[1];

    // 預先掃描資料夾以計算總幀數
    std::cout << "正在掃描資料夾中的幀數..." << std::endl;
    pcl::PCDReader reader;
    while (true) {
        pcl::PCLPointCloud2 dummy;
        std::string path_to_check = pcd_directory + "/" + filename_prefix + std::to_string(total_frames) + ".pcd";
        if (reader.readHeader(path_to_check, dummy) != 0) {
            break;
        }
        total_frames++;
    }
    if (total_frames == 0) {
        std::cerr << "在指定資料夾中找不到任何有效的 .pcd 檔案 (e.g., " << filename_prefix << "0.pcd...)" << std::endl;
        return -1;
    }
    std::cout << "掃描完成，共找到 " << total_frames << " 幀。" << std::endl;

    // 自動播放參數
    const std::chrono::milliseconds auto_play_delay(15);  // 每幀延遲100ms -> 10 FPS
    auto last_frame_time = std::chrono::steady_clock::now();

    // 初始化視覺化視窗
    viewer.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0.05, 0.05, 0.05);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(20, 0, 10, 0, 0, 10, 0, 0, 1);
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());

    viewer_voxel_grid.reset(new pcl::visualization::PCLVisualizer("Voxel Viewer"));
    viewer_voxel_grid->setBackgroundColor(0.05, 0.05, 0.05);
    viewer_voxel_grid->addCoordinateSystem(1.0);
    viewer_voxel_grid->initCameraParameters();
    viewer_voxel_grid->setCameraPosition(20, 0, 10, 0, 0, 10, 0, 0, 1);
    viewer_voxel_grid->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer_voxel_grid.get());

    viewer_ground_filtering.reset(new pcl::visualization::PCLVisualizer("Ground Viewer"));
    viewer_ground_filtering->setBackgroundColor(0.05, 0.05, 0.05);
    viewer_ground_filtering->addCoordinateSystem(1.0);
    viewer_ground_filtering->initCameraParameters();
    viewer_ground_filtering->setCameraPosition(20, 0, 10, 0, 0, 10, 0, 0, 1);
    viewer_ground_filtering->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer_ground_filtering.get());

    viewer_original.reset(new pcl::visualization::PCLVisualizer("Original Viewer"));
    viewer_original->setBackgroundColor(0.05, 0.05, 0.05);
    viewer_original->addCoordinateSystem(1.0);
    viewer_original->initCameraParameters();
    viewer_original->setCameraPosition(20, 0, 10, 0, 0, 10, 0, 0, 1);
    viewer_original->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer_original.get());

    viewer_cluster.reset(new pcl::visualization::PCLVisualizer("CLuster Viewer"));
    viewer_cluster->setBackgroundColor(0.05, 0.05, 0.05);
    viewer_cluster->addCoordinateSystem(1.0);
    viewer_cluster->initCameraParameters();
    viewer_cluster->setCameraPosition(20, 0, 10, 0, 0, 10, 0, 0, 1);
    viewer_cluster->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer_cluster.get());
    current_frame_idx = (size_t)atoi(argv[2]);
    // 主迴圈
    while (!viewer->wasStopped()) {
        // 自動播放邏輯
        if (auto_play && !paused) {
            auto current_time = std::chrono::steady_clock::now();
            if (current_time - last_frame_time > auto_play_delay) {
                current_frame_idx = (current_frame_idx + 1) % total_frames;
                frame_update_required = true;
            }
        }

        // 幀更新邏輯
        if (!paused && frame_update_required) {
            std::shared_ptr<FrameData> frame_to_display;

            // 1. 檢查快取
            if (frame_cache.count(current_frame_idx)) {
                frame_to_display = frame_cache[current_frame_idx];
            } else {
                frame_to_display = loadAndProcessFrame_update(current_frame_idx);
                if (frame_to_display) {
                    if (frame_cache.size() >= CACHE_SIZE) {
                        uint32_t oldest_frame_id = cache_order.front();
                        cache_order.pop_front();
                        frame_cache.erase(oldest_frame_id);
                    }
                    frame_cache[current_frame_idx] = frame_to_display;
                    cache_order.push_back(current_frame_idx);
                }
            }

            // 2. 更新視覺化
            if (frame_to_display) {
                viewer->removeAllPointClouds();
                viewer->removeAllShapes();

                viewer_voxel_grid->removeAllPointClouds();
                viewer_ground_filtering->removeAllPointClouds();

                if (!auto_play) {
                    std::cout << "[顯示] 正在顯示幀 ID: " << frame_to_display->id << std::endl;
                } else {
                    std::cout << "[播放中] 幀: " << frame_to_display->id << "/" << total_frames - 1 << "  \r";
                    fflush(stdout);
                }

                auto& frame = *frame_to_display;
                std::string cloud_id = std::to_string(frame.id);

                // New
                if (frame.raw_cloud_viz && !frame.raw_cloud_viz->empty()) {
                    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> raw_color(frame.raw_cloud_viz, 128, 128, 128);  // 灰色
                    viewer->addPointCloud(frame.raw_cloud_viz, raw_color, "raw_cloud_" + cloud_id);
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "raw_cloud_" + cloud_id);

                    viewer_original->addPointCloud(frame.raw_cloud_viz, raw_color, "original_cloud_" + cloud_id);
                    viewer_original->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "raw_cloud_" + cloud_id);

                    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> voxel_color(frame.non_clustered_points_viz, 255, 255, 255);  // 灰色
                    viewer_voxel_grid->addPointCloud(frame.non_clustered_points_viz, voxel_color, "voxel_cloud_" + cloud_id);
                    viewer_voxel_grid->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "voxel_cloud_" + cloud_id);

                    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> ground_color(frame.ground_points_viz, 255, 255, 255);  // 灰色
                    viewer_ground_filtering->addPointCloud(frame.ground_points_viz, ground_color, "grounded_cloud_" + cloud_id);
                    viewer_ground_filtering->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "grounded_cloud_" + cloud_id);

                    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(frame.clustered_obstacle_points_viz);
                    viewer_cluster->addPointCloud(frame.raw_cloud_viz, raw_color, "raw_cloud_" + cloud_id);
                    viewer_cluster->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "raw_cloud_" + cloud_id);
                    viewer_cluster->addPointCloud(frame.clustered_obstacle_points_viz, rgb, "clusters_" + cloud_id);
                    viewer_cluster->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "clusters_" + cloud_id);
                }

                // 2. 在背景之上，用「彩色」繪製被分群的障礙物點雲
                //    這會覆蓋掉原始點雲中對應部分的顏色
                if (frame.clustered_obstacle_points_viz && !frame.clustered_obstacle_points_viz->empty()) {
                    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(frame.clustered_obstacle_points_viz);
                    viewer->addPointCloud(frame.clustered_obstacle_points_viz, rgb, "clusters_" + cloud_id);
                    // 讓障礙物的點稍微大一點，看得更清楚
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "clusters_" + cloud_id);
                }

                // if (frame.clustered_obstacle_points_viz && !frame.clustered_obstacle_points_viz->empty()) {
                //     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(frame.clustered_obstacle_points_viz);
                //     viewer->addPointCloud(frame.clustered_obstacle_points_viz, rgb, "clusters_" + cloud_id);
                //     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "clusters_" + cloud_id);
                // }

                // if (show_ground && frame.ground_points_viz && !frame.ground_points_viz->empty()) {
                //     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color(frame.ground_points_viz, 0, 128, 0);
                //     viewer->addPointCloud(frame.ground_points_viz, color, "ground_" + cloud_id);
                //     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "ground_" + cloud_id);
                // }

                for (const auto& box : frame.boxes_viz) {
                    std::string box_id = "box_" + cloud_id + "_" + std::to_string(box.id);
                    double r, g, b;
                    if (box.is_classified_as_car) {
                        r = 0;
                        g = 1;
                        b = 0;
                    } else if (box.is_classified_as_motor) {
                        r = 0;
                        g = 0;
                        b = 1;
                    } else {
                        r = 1;
                        g = 0;
                        b = 0;
                    }
                    std::string text_content = "ID: " + std::to_string(box.id);

                    // 2. 建立一個專屬於這個文字的唯一 ID
                    std::string text_id = "text_" + box_id;

                    // 3. (關鍵) 將 Eigen::Vector3f 轉換為 pcl::PointXYZ
                    // 首先計算好在 Eigen::Vector3f 中的最終座標
                    Eigen::Vector3f text_eigen_position = box.position;
                    text_eigen_position.z() += box.dimension.z() / 2.0f + 0.2f;  // 移到框頂再往上一點

                    // 然後用最終的 Eigen 向量來建立 pcl::PointXYZ 物件
                    pcl::PointXYZ text_pcl_position(
                        text_eigen_position.x(),
                        text_eigen_position.y(),
                        text_eigen_position.z());

                    // 4. 呼叫 addText3D 函式，傳入轉換後的 pcl::PointXYZ
                    viewer->addCube(box.position, box.quaternion.normalized(), box.dimension.x(), box.dimension.y(), box.dimension.z(), box_id);
                    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, box_id);
                    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, box_id);
                    viewer->addText3D(text_content, text_pcl_position, 0.4, 1.0, 1.0, 1.0, text_id);
                }
                addDistanceMarkers(viewer);

                last_frame_time = std::chrono::steady_clock::now();
            }
            frame_update_required = false;
        }

        viewer->spinOnce(100);
        viewer_voxel_grid->spinOnce(100);
        viewer_ground_filtering->spinOnce(100);
    }
    return 0;
}

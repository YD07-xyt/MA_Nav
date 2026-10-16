#pragma once
#include "3d_occ_map/rog_map.h"
#include "grid_map.hpp"
#include "map/3d_occ_map/config.hpp"
#include "map/terrain_analysis.hpp"
#include "utils/logger.hpp"
#include <memory>
#include <mutex>
#include <optional>
namespace ma_map {
class MaMap {
public:
    explicit MaMap(const std::string& cfg_path) {
        grid_map_ptr_ = std::make_shared<grid_map::GridMap>();
        rog_map_ptr_ = std::make_shared<rog_map::ROGMap>();

        cfg_ = rog_map::Config(cfg_path);
        rog_map_ptr_->setConfig(cfg_);
        rog_map_ptr_->init();
        
        Terrain::TerrainAnalyzer::Config terrain_cfg;
        terrain_cfg.kernel_size = cfg_.terrain_kernel_size;
        terrain_cfg.max_step_height = cfg_.terrain_max_step_height;
        terrain_cfg.robot_height = cfg_.terrain_robot_height;
        terrain_cfg.steep_threshold = cfg_.terrain_steep_threshold;
        terrain_cfg.resolution = cfg_.resolution;
        terrain_cfg.map_size_x = cfg_.local_update_box_d.x();
        terrain_cfg.map_size_y = cfg_.local_update_box_d.y();
        terrain_analyzer_.setConfig(terrain_cfg);
        grid_map_ptr_->init(
            cfg_.grid_map_x_left,
            cfg_.grid_map_x_right,
            cfg_.grid_map_y_left,
            cfg_.grid_map_y_right,
            cfg_.grid_map_resolution
        );

        auto voxel_num = grid_map_ptr_->getVoxelNum();
        
        occupancy_2d = grid_map::RowMatrixXi::Zero(voxel_num.x(), voxel_num.y());
        global_2d_occ_=grid_map::RowMatrixXi::Zero(voxel_num.x(), voxel_num.y());
        dynamic_2d_occ_=grid_map::RowMatrixXi::Zero(voxel_num.x(), voxel_num.y());
        publish_grid_map();
    }
    auto set_global_map(grid_map::RowMatrixXi global_map) -> void {
        global_2d_occ_ = global_map;
        std::lock_guard<std::mutex> lock(back_mtx_);
        grid_map_ptr_->setMap(global_2d_occ_);
        publish_grid_map();
    }
    // 语义标记写到后台缓冲并发布，避免被下一帧地图更新覆盖
    auto set_semantics_polygon(const std::vector<Eigen::Vector2d>& polygon, grid_map::GridMap::Semantics semantics)
        -> void {
        std::lock_guard<std::mutex> lock(back_mtx_);
        grid_map_ptr_->semantics_polygon_region(polygon, semantics);
        publish_grid_map();
    }
    auto set_mapping_model(bool mapping_model) -> void {
        mapping_model_ = mapping_model;
    }
    auto update_map() -> void {
        if (map_empty_) {
            static double last_print_t =
                std::chrono::duration<double>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            double cur_t =
                std::chrono::duration<double>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            if (cfg_.ros_callback_en && (cur_t - last_print_t > 1.0)) {
                logger::MaMap->warn("No point cloud input, check the topic name.");
                last_print_t = cur_t;
            }
            return;
        }

        if (unfinished_frame_cnt == 0) {
            return;
        }

        if (unfinished_frame_cnt > 1) {
            logger::MaMap->warn("Unfinished frame cnt > 1, the map may not work in real-time");
        }
        static rog_map::PointCloud temp_pc;
        static rog_map::Pose temp_pose;

        updete_lock.lock();
        temp_pc = cloud_;
        temp_pose = pc_pose_;
        unfinished_frame_cnt = 0;
        updete_lock.unlock();

        rog_map_ptr_->updateProbMap(temp_pc, temp_pose);
        update_terrain();
        if(!mapping_model_){
            dynamic_2d_occ_.setZero();
        }

        // 按真实占地写入：terrain 点是 terrain 格（边长 cfg_.resolution）的中心，
        // 该中心正好落在 2D 细格边界上，直接 posToIndex 截断会随浮点误差左右跳一格。
        // 这里把整个 terrain 格覆盖到 2D 栅格：两端各向格内收 1/4 细格再取索引，避开边界截断。
        const double terrain_half = 0.5 * cfg_.resolution;
        const double eps = 0.25 * grid_map_ptr_->getResolution();
        const Eigen::Vector2i max_idx(dynamic_2d_occ_.rows() - 1, dynamic_2d_occ_.cols() - 1);
        for(auto & i : terrain_map_){
            Eigen::Vector2i lo, hi;
            grid_map_ptr_->posToIndex(Eigen::Vector2d(i.x() - terrain_half + eps, i.y() - terrain_half + eps), lo);
            grid_map_ptr_->posToIndex(Eigen::Vector2d(i.x() + terrain_half - eps, i.y() + terrain_half - eps), hi);
            lo = lo.cwiseMax(Eigen::Vector2i::Zero());
            hi = hi.cwiseMin(max_idx);
            for (int x = lo.x(); x <= hi.x(); ++x) {
                for (int y = lo.y(); y <= hi.y(); ++y) {
                    dynamic_2d_occ_(x, y) = 1; // 标记为障碍物
                }
            }
        }
        occupancy_2d = dynamic_2d_occ_.cwiseMax(global_2d_occ_) ;
        {
            std::lock_guard<std::mutex> lock(back_mtx_);
            grid_map_ptr_->setMap(occupancy_2d);
            //grid_map_ptr_->inflate(0.02);
            publish_grid_map();
        }
    };
    auto update_odom(const rog_map::Pose& pose) -> void {
        rog_map_ptr_->updateRobotState(pose);
    }
    auto update_cloud(const rog_map::PointCloud& cloud) -> void {
        if (!rog_map_ptr_->robot_state_.rcv) {
            logger::MaMap->warn("No odom received, skip cloud callback.");
            return;
        }
        double cbk_t =
            std::chrono::duration<double>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        if (cbk_t - rog_map_ptr_->robot_state_.rcv_time > cfg_.odom_timeout) {
            logger::MaMap->warn("Odom timeout, skip cloud callback.");
            return;
        }
        updete_lock.lock();
        cloud_ = cloud;
        pc_pose_ = std::make_pair(rog_map_ptr_->robot_state_.p, rog_map_ptr_->robot_state_.q);
        unfinished_frame_cnt++;
        map_empty_ = false;
        updete_lock.unlock();
    }
public:
    auto get_terrain_map() -> utils::vec_E<utils::Vec3f> {
        return terrain_map_;
    }
    // 返回最近一次发布的只读快照。调用方持有该 shared_ptr 期间内容不变，
    // 地图线程只写后台缓冲，写完整体换出新快照；不要通过返回值修改地图。
    auto get_grid_map() const -> std::shared_ptr<grid_map::GridMap> {
        std::lock_guard<std::mutex> lock(front_mtx_);
        return front_grid_map_;
    }
    auto get_occupancy_2d()-> grid_map::RowMatrixXi {
        return occupancy_2d;
    }
    auto get_rog_map() -> std::shared_ptr<rog_map::ROGMap> {
        return rog_map_ptr_;
    }
    auto get_config()->rog_map::Config{
        return cfg_;
    }
    auto get_global_occ() const -> grid_map::RowMatrixXi {
        return global_2d_occ_;
    }
private:
    bool mapping_model_ = true; // true: 建图模式，false: 规划模式
    utils::vec_E<utils::Vec3f> terrain_map_;
    grid_map::RowMatrixXi global_2d_occ_;     // 永久层（可 set_global_map 加载）
    grid_map::RowMatrixXi dynamic_2d_occ_;    // 动态层（每帧重建）
    grid_map::RowMatrixXi occupancy_2d;
    auto update_terrain() -> void {
        utils::Vec3f box_max = rog_map_ptr_->robot_state_.p + cfg_.visualization_range / 2;
        utils::Vec3f box_min = rog_map_ptr_->robot_state_.p - cfg_.visualization_range / 2;
        utils::vec_E<utils::Vec3f> inf_occ_map, real_occ_map;
        //rog_map_ptr_->boxSearchInflate(box_min, box_max, utils::OCCUPIED, inf_occ_map);
        rog_map_ptr_->boxSearch(box_min, box_max, utils::OCCUPIED, real_occ_map);
        terrain_map_ = terrain_analyzer_.analyze(rog_map_ptr_->robot_state_, real_occ_map);
        // utils::vec_E<utils::Vec3f> output;
        // for (auto pt: terrain_map) {
        //     if (pt.z() - rog_map_ptr_->robot_state_.p.z() < 0.07) {
        //         //continue;
        //     }
        //     output.emplace_back(pt);
        // }
    }
    
private:
    rog_map::Pose pc_pose_;
    rog_map::PointCloud cloud_;
    // 双缓冲：grid_map_ptr_ 为后台缓冲（仅在 back_mtx_ 下写），
    // front_grid_map_ 为对外发布的快照（每次发布新建，发布后不再修改）。
    // 调用方须先持有 back_mtx_。
    auto publish_grid_map() -> void {
        auto snapshot = std::make_shared<grid_map::GridMap>(*grid_map_ptr_);
        std::lock_guard<std::mutex> lock(front_mtx_);
        front_grid_map_.swap(snapshot);
    }
    std::shared_ptr<grid_map::GridMap> grid_map_ptr_;
    std::shared_ptr<grid_map::GridMap> front_grid_map_;
    std::mutex back_mtx_;
    mutable std::mutex front_mtx_;
    std::shared_ptr<rog_map::ROGMap> rog_map_ptr_;
    Terrain::TerrainAnalyzer terrain_analyzer_;
    rog_map::Config cfg_;
    bool map_empty_ = true;
    int unfinished_frame_cnt {0};
    rog_map::mutex updete_lock;
};
}
#pragma once

#include "3d_occ_map/rog_map.h"
#include "utils/type_utils.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

namespace Terrain {

using Map = utils::vec_E<utils::Vec3f>;

class TerrainAnalyzer {
public:
    struct Config {
        int   kernel_size = 5;         // 中值滤波核大小
        float resolution = 0.1f;       // 网格分辨率 (米)
        float map_size_x = 5.0f;       // 分析窗口 x 方向大小 (米)
        float map_size_y = 5.0f;       // 分析窗口 y 方向大小 (米)
        float max_step_height = 0.15f; // 台阶/墙体最小高度差（用于触发间隙分析）
        float robot_height = 0.3f;     // 机器人安全通行所需的最小垂直间隙
        float steep_threshold = 0.17f;  // 建议设为与 max_step_height 相近或稍大，如0.2m
    };

    TerrainAnalyzer() = default;
    explicit TerrainAnalyzer(const Config& cfg) : config_(cfg) {}

    void setConfig(const Config& cfg) { config_ = cfg; }

    // 输入 occ_map：世界坐标系下的 3D 占据点云
    // robot_pose：机器人当前位姿（用于确定窗口中心与地面参考高度）
    // 输出：每个网格中心的点云，z=0 表示可通行，z=1 表示不可通行
    auto analyze(const utils::RobotState& robot_pose,
                 const Map& occ_map) -> Map;

    // 同上，额外输出空闲证据 free_out（网格中心，z=0）：
    // 本帧该格自身有占据点（地面可见）、未判为障碍，且判定明确
    // （平坦，或间隙分析为可通行）。仅因绝对高度阈值未判障碍的格不算空闲，
    // 这类格保持原值，避免把低矮障碍回写成空闲。
    auto analyze(const utils::RobotState& robot_pose,
                 const Map& occ_map, Map& free_out) -> Map;

private:
    Config config_;

    // 中值滤波（仅考虑非 NaN 邻居）
    auto medianFilter(const Eigen::ArrayXXf& input, int kernel_size) -> Eigen::ArrayXXf;

    // 网格化工具（以窗口 origin 为基准）
    struct GridInfo {
        int width, height;
        utils::Vec3f origin;   // 窗口左下角世界坐标
    };
    auto setupGrid(const utils::Vec3f& robot_pos) const -> GridInfo;
    auto worldToGrid(const GridInfo& grid, const utils::Vec3f& world) const
        -> std::optional<std::pair<int, int>>;
    auto gridToWorld(const GridInfo& grid, int gx, int gy) const -> utils::Vec3f;
};

} // namespace Terrain
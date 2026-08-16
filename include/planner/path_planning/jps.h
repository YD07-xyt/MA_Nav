#pragma once

#include "map/grid_map.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cstdint>
#include <vector>

namespace path_planning {

/**
 * 跳点搜索 (Jump Point Search, JPS) —— 8 连通栅格规划器,接口与 AStar 兼容。
 *
 * 与 A* 逐个扩展相邻栅格不同,JPS 会对栅格图进行剪枝:它沿直线"跳跃",
 * 只扩展跳点(拐点 / 强制邻居 / 终点)。在同样的栅格图上,它通常只需要
 * 展开比普通 A* 少一到两个数量级的节点,因此规划速度快得多。
 *
 * 性能设计(为什么快):
 *  - 所有搜索状态都存放在以线性栅格地址 (x * cols + y) 索引的扁平数组中,
 *    没有 std::string 哈希键,也没有动态节点。
 *  - 每次搜索的状态用"代际计数"保护:同一张地图上重复搜索时,不需要
 *    对整张栅格做 O(N) 的初始化清零。
 *  - 占用判定(ESDF < safe_threshold,语义与 AStar 相同)按栅格惰性求值并
 *    缓存,射线只对经过的栅格付费。
 *  - 展开节点时,只扫描相对来向的自然方向 + 强制邻居方向(基于父节点的
 *    剪枝),而不是全部 8 个方向。
 */
class JPS {
public:
    JPS(grid_map::GridMap& map, double safe_threshold);
    JPS() = default;

    auto set_map(const grid_map::GridMap& map) -> void;
    auto set_safe_threshold(const double& safe_threshold) -> void {
        safe_threshold_ = safe_threshold;
        std::fill(blocked_cache_.begin(), blocked_cache_.end(), 0); // 安全阈值变化后占用可能改变
    }

    // JPS 搜索。返回航点路径(栅格中心,首点为精确起点,与
    // AStar::original_astar_search 的约定一致)。
    // 返回空 vector = 起点/终点无效、无路径或超时。
    auto jps_search(const Eigen::Vector2d& start, const Eigen::Vector2d& goal, int timeout_ms = 1000)
        -> std::vector<Eigen::Vector2d>;

private:
    grid_map::GridMap map_;
    double safe_threshold_ = 0.0;

    // 缓存的栅格几何信息(与 map_ 保持一致)
    int nx_ = 0;
    int ny_ = 0;
    Eigen::Vector2d origin_ = Eigen::Vector2d::Zero();
    double resolution_ = 0.1;

    // 终点所在栅格(jump_ray 热路径使用)
    int goal_x_ = -1;
    int goal_y_ = -1;

    // 逐栅格占用缓存:0 = 未知, 1 = 自由, -1 = 被占用
    mutable std::vector<int8_t> blocked_cache_;

    // 单次搜索的扁平数组,带代际保护(避免每次搜索整体 memset)
    std::vector<int32_t> open_gen_;
    std::vector<int32_t> closed_gen_;
    std::vector<double> g_score_;
    std::vector<double> f_score_;
    std::vector<int32_t> parent_; // 父节点线性地址,-1 = 无
    int32_t search_gen_ = 0;

    inline int addr(int x, int y) const { return x * ny_ + y; }
    inline bool in_map(int x, int y) const { return x >= 0 && x < nx_ && y >= 0 && y < ny_; }

    // 带缓存的逐栅格占用判定;越界栅格一律视为被占用。
    bool blocked(int x, int y) const;

    // 八叉距离(以栅格为单位),是 8 连通栅格的一致启发函数。
    static double heuristic_cells(int dx, int dy);

    // 判断 (x, y) 相对来向 (dx, dy) 是否存在强制邻居。
    bool has_forced_neighbor(int x, int y, int dx, int dy) const;

    // 从 (x, y) 沿 (dx, dy) 扫描直线射线。成功时把第一个跳点(或终点所在
    // 栅格)写入 jp 并返回 true。
    bool jump_ray(int x, int y, int dx, int dy, Eigen::Vector2i& jp) const;

    // 展开 (x, y) 时需要扫描的方向,由来向决定(父节点剪枝)。
    // 起点节点(in == 0,0)扫描全部 8 个方向。
    int neighbor_directions(int x, int y, int in_dx, int in_dy, int dirs[8][2]) const;

    // 8 个邻居中是否至少有一个自由栅格。
    bool has_free_neighbor(int x, int y) const;

    // 8 个邻居中被占用栅格的个数(用于诊断日志)。
    int blocked_neighbor_count(int x, int y) const;

    // 环形搜索在 max_radius 个栅格内最近的"可通行"栅格(自由且至少有一个
    // 自由邻居)。用于从被占用 / 孤立 / 地图外的起点恢复搜索,与 A* 对地图外
    // 点的容忍行为保持一致。
    bool find_navigable_cell(int x, int y, int max_radius, Eigen::Vector2i& out) const;

    bool check_collision(const Eigen::Vector2d& pos) const;

    std::vector<Eigen::Vector2d> reconstruct_path(int goal_lin) const;
};

} // namespace path_planning

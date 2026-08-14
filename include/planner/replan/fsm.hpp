#pragma once

#include "planner/opt/traj_optimizer.hpp"
#include "planner/path_planning/astar.h"
#include "utils/type_utils.hpp"
#include "utils/logger.hpp"
#include <Eigen/Core>
#include <optional>
#include <spdlog/spdlog.h>
#include <utils/expected.hpp>
namespace planner {
class FSM {
public:
    struct PlannerConfig {
        // TrajOpt::TrajectoryParams params_;
        Opt::OptimizerParams opt_params;
        float safe_threshold;
        //重规划触发参数
        struct FsmParam {
            Eigen::Vector3d deviation;
            double replan_interval; // 路径最大年龄（s），超过则强制重规划
            double replan_lateral_dev; // 横向偏差阈值（m），机器人偏离参考路径超过则重规划
            double min_replan_interval; // 最小重规划间隔（s），防抖，避免多触发源共振
            double goal_reached_radius; // 起点≈终点判定半径（m）：机器人距目标小于该值时直接退出不规划，
                // 避免退化路径（A* 单点）与无意义的周期重规划
            double hard_clearance; // 优化结果硬净空（m）：轨迹采样点净空低于该值直接拒收。
                // 应 ≤ 通道允许的最小净空（≈通道半宽）；太小失去安全网，太大窄缝永远过不去
        } fsm_param;
        struct AstarParam {
            double max_vel;
            double max_acc;
            double time_resolution;
            int min_traj_num;
            double traj_cut_length;
            double distance_weight;
            double yaw_weight;
        } astar_param;
    } planner_config_;

public:
    explicit FSM(const PlannerConfig& planner_config): planner_config_(planner_config) {}
    enum path_error {
        none,
        success,
        astar_path_empty,
        optimizer_failed,
        astar_path_collision,
    };
    using path = std::vector<Eigen::Vector2d>;
    using astar_opt_path = tl::expected<std::pair<path, Opt::EsdfTrajectoryOptimizer>, path_error>;
    auto set_astar_param(path_planning::AStar& astar);
    auto plan(
        const utils::RobotState& goal_pose,
        const utils::RobotState& current_pose,
        std::shared_ptr<grid_map::GridMap> grid_map
    ) -> astar_opt_path;

public:
private:
    // 将过于靠近障碍物的点沿 ESDF 梯度外推到安全距离，确保 A*/碰撞检测可通过
    auto getSafeStart(const Eigen::Vector2d& pos) -> Eigen::Vector2d;
    /**
  @brief: 检查是否碰撞
  */
    auto checkCollision(path_planning::AStar::Trajectory astar_traj) -> bool;
    /**
  @brief: 判断2个点是否相等
  @param: 点1,2 ，允许误差
  */
    auto checkPointEqual(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, const Eigen::Vector3d& deviation)
        -> bool;

    enum PathState {
        running,
        successed,
        failed,
        idle,
    } path_state_ = PathState::idle;
    // PathState opt_state_=PathState::idle;
    path_planning::AStar::Trajectory astar_traj_;
    std::shared_ptr<grid_map::GridMap> grid_map_;
    utils::RobotState old_goal_pose_;
    // 重规划触发辅助
    double last_plan_time_ = -1.0; // 上次成功规划时间（steady_clock 秒）
    double last_replan_time_ = -1.0; // 上次触发重规划的时间（防抖用）
    std::vector<Eigen::Vector2d> last_opt_path_; // 最近一次优化轨迹采样点（横向偏差检测用）
    /** @brief: 计算点到折线路径的横向距离（逐段最近点） */
    auto lateralDeviation(const Eigen::Vector2d& pos, const std::vector<Eigen::Vector2d>& path) -> double;
};
} // namespace planner
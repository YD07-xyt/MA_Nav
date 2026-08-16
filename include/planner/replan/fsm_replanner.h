#pragma once
#include "map/grid_map.hpp"
#include "planner/path_planning/path_planning.hpp"
#include "planner/traj_optimize/minco_opt/grid_map_esdf.hpp"
#include "planner/traj_optimize/minco_opt/minco_optimizer.hpp"
#include "planner/path_planning/post_processing.h"
#include "utils/expected.hpp"
#include "utils/logger.hpp"
#include "utils/type_utils.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>
namespace replan {
class FsmReplan {
public:
    //重规划触发参数
    struct ReplanParam {
        Eigen::Vector3d goal_deviation;
        double replan_interval; // 路径最大年龄（s），超过则强制重规划
        double replan_lateral_dev; // 横向偏差阈值（m），机器人偏离参考路径超过则重规划
        double min_replan_interval; // 最小重规划间隔（s），防抖，避免多触发源共振
        double goal_reached_radius; // 起点≈终点判定半径（m）：机器人距目标小于该值时直接退出不规划，
            // 避免退化路径（A* 单点）与无意义的周期重规划
        double hard_clearance; // 优化结果硬净空（m）：轨迹采样点净空低于该值直接拒收。
            // 应 ≤ 通道允许的最小净空（≈通道半宽）；太小失去安全网，太大窄缝永远过不去
    } replan_param;
    struct PlannerConfig {
        ReplanParam replan_params;
        minco_opt::MincoOptimizerConfig minco_opt_params;
        path_planning::PathPostProcessing::PathPostProcessingParams path_planning_params;
    } planner_config_;
    auto set_param(const PlannerConfig& planner_config) -> void {
        planner_config_ = planner_config;
        path_planning.set_param(planner_config_.path_planning_params);
    };
    FsmReplan() = default;
    explicit FsmReplan(const PlannerConfig& planner_config) {
        planner_config_ = planner_config;
        path_planning.set_param(planner_config_.path_planning_params);
    }

private:
    enum PathState {
        SUCCESSED,
        FAILED,
        IDLE,
    } path_state_ = PathState::IDLE;
    bool need_replan_ = true;
    utils::RobotState old_goal_pose_;
    std::vector<Eigen::Vector2d> last_opt_path_;
    struct ResultPath {
        path_planning::PathPostProcessing::Trajectory planning_traj;
        Trajectory<5, 2> opt_traj;
        PathState path_state;
    };
    ResultPath result_;

public:
    enum PathError {
        PLANNING_FAILED,
        MINCO_OPT_FIALED,
        MAX_RETRIES,
    };

    using path = tl::expected<ResultPath, PathError>;
    auto plan(
        const utils::RobotState& goal_pose,
        const utils::RobotState& current_pose,
        std::shared_ptr<grid_map::GridMap> grid_map
    ) -> path {
        
        // 起点≈终点（机器人已到达目标附近）：直接退出，不规划。
        if ((current_pose.p.head<2>() - goal_pose.p.head<2>()).norm()
            < planner_config_.replan_params.goal_reached_radius) {
            path_state_ = PathState::SUCCESSED;
            result_.path_state = path_state_;
            return result_;
        }
        const auto safe_threshold = planner_config_.path_planning_params.safe_threshold;
        // 触发重规划的三个条件（满足任一即重规划）：
        //   1. 路径碰障（A* 原始网格路径 + 优化后航点路径，见 checkCollision）
        //   2. 路径年龄超过 replan_interval_
        //   3. 机器人偏离参考路径超过 replan_lateral_dev_
        Eigen::Vector2d diff = (old_goal_pose_.p - goal_pose.p).head<2>();
        Eigen::Vector2d threshold = planner_config_.replan_params.goal_deviation.head<2>();
        if (diff.cwiseAbs().x() > threshold.x() && diff.cwiseAbs().y() > threshold.y()) {
            // 任一轴超出阈值即触发重规划
            need_replan_ = true;
        }
        //路径碰障（jps优化后航点路径)
        const bool path_collision = result_.planning_traj.optimized_path.empty()
            || check_collision(result_.planning_traj, *grid_map, safe_threshold);
        if (path_collision) {
            need_replan_ = true;
        }

        //机器人偏离参考路径超过 replan_lateral_dev_
        const double lateral_dev = lateral_deviation(
            Eigen::Vector2d(current_pose.p.x(), current_pose.p.y()),
            last_opt_path_.empty() ? result_.planning_traj.optimized_path : last_opt_path_
        );

        if (lateral_dev > planner_config_.replan_params.replan_lateral_dev) {
            need_replan_ = true;
        }
        // 暂时不考虑加入 路径年龄超过 replan_interval_
        if (need_replan_) {
            utils::TimeConsuming timer("planner_fsm", false); // true 表示允许打印
            path_planning.set_map(*grid_map);

            Eigen::Vector2d start(current_pose.p.x(), current_pose.p.y());
            Eigen::Vector2d goal(goal_pose.p.x(), goal_pose.p.y());

            // 若起点/终点过于靠近障碍物，沿 ESDF 梯度外推到安全点，保证可规划
            start = get_safe_pos(start, *grid_map, safe_threshold);
            goal = get_safe_pos(goal, *grid_map, safe_threshold);

            path_planning.set_use_jps(true);

            auto trajectory = path_planning.path_planning(start, goal, 5000);
            if (!trajectory.has_value()) {
                logger::fsm_replan->info("planning failed");
                need_replan_=true;
                return tl::make_unexpected(PathError::PLANNING_FAILED);
            }

            // MINCO 五阶轨迹优化(时间 + 平滑 + ESDF 避障 + 速度/加速度软约束)。
            // 优化失败或安全检查不过时保持 path_planning 原始轨迹,安全兜底。
            result_.planning_traj = trajectory.value();
            // [临时诊断] 打印 raw_path / optimized_path 点数,确认直线/近距离时的短路情况
            logger::fsm_replan->info(
                "path points: raw={} optimized={} total_time={:.2f}s",
                result_.planning_traj.raw_path.size(),
                result_.planning_traj.optimized_path.size(),
                result_.planning_traj.total_time
            );

            auto minco_path = minco_optimize(result_.planning_traj, grid_map, current_pose);
            if (!minco_path) {
                need_replan_=true;
                return tl::make_unexpected(PathError::MINCO_OPT_FIALED);
            }
            result_.path_state = PathState::SUCCESSED;
            old_goal_pose_ = goal_pose;
            last_opt_path_ = sample_minco_trajectory(minco_path.value(), 0.02);
            result_.opt_traj = minco_path.value();
            need_replan_=false;
            return result_;
        }
        return result_;
    }

private:
    // 将过于靠近障碍物的点沿 ESDF 梯度外推到安全距离，确保 A*/碰撞检测可通过
    auto get_safe_pos(const Eigen::Vector2d& pos, const grid_map::GridMap& grid_map, const double& safe_threshold)
        -> Eigen::Vector2d;
    auto check_point_equal(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, const Eigen::Vector3d& deviation)
        -> bool;
    auto check_collision(
        path_planning::PathPostProcessing::Trajectory traj,
        const grid_map::GridMap& grid_map,
        const double& safe_threshold
    ) -> bool;
    auto lateral_deviation(const Eigen::Vector2d& pos, const std::vector<Eigen::Vector2d>& path) -> double;
    auto sample_minco_trajectory(Trajectory<5, 2> spline, const double dt_check) -> std::vector<Eigen::Vector2d>;

private:
    path_planning::PathPlanning path_planning;
    minco_opt::MincoOptimizer minco_; // MINCO 五阶优化器(非线程安全,单线程调用)
    enum MincoError { OPTFAIL };
    // MINCO 轨迹优化:成功返回 true 并原地更新 traj;失败返回 false,traj 保持原样。
    auto minco_optimize(
        path_planning::PathPostProcessing::Trajectory& traj,
        std::shared_ptr<grid_map::GridMap> grid_map,
        const utils::RobotState& current_pose
    ) -> tl::expected<Trajectory<5, 2>, MincoError>;

    // 把优化后的五阶样条采样回填到 Trajectory(下游 MPC/可视化格式不变)。
    auto rebuild_optimized_trajectory(
        path_planning::PathPostProcessing::Trajectory& traj,
        const Trajectory<5, 2>& spline,
        const path_planning::PathPostProcessing::PathPostProcessingParams& pp_params
    ) -> void;
};
} // namespace replan

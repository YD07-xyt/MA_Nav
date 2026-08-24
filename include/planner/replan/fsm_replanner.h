#pragma once
#include "map/grid_map.hpp"
#include "planner/path_planning/path_planning.hpp"
#include "planner/traj_optimize/ma_spline_opt/optimizer_config.h"
#include "planner/traj_optimize/ma_spline_opt/traj_optimizer.h"
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
        //double replan_interval; // 路径最大年龄（s），超过则强制重规划
        double replan_lateral_dev; // 横向偏差阈值（m），机器人偏离参考路径超过则重规划
        double minco_traj_validity_duration; // minco轨迹有效期 (s)，超时则视为无效
        double goal_reached_radius; // 起点≈终点判定半径（m）：机器人距目标小于该值时直接退出不规划，
            // 避免退化路径（A* 单点）与无意义的周期重规划
        //double hard_clearance; // 优化结果硬净空（m）：轨迹采样点净空低于该值直接拒收。
        // 应 ≤ 通道允许的最小净空（≈通道半宽）；太小失去安全网，太大窄缝永远过不去
        double minco_traj_continuity_threshold; ///< 轨迹连续性距离阈值
            ///< (m)，机器人距轨迹投影点小于此值时使用投影点边界
        double projection_search_resolution; ///< 投影点搜索时间分辨率 (s)，越小越精确但计算量越大
    } replan_param;
    struct PlannerConfig {
        ReplanParam replan_params;
        minco_opt::MincoOptimizerConfig minco_opt_params;
        ma_spline_opt::MaSplineOptimizerConfig ma_spline_opt_params;
        path_planning::PathPostProcessing::PathPostProcessingParams path_planning_params;
    } planner_config_;
    auto set_param(const PlannerConfig& planner_config) -> void {
        planner_config_ = planner_config;
        ma_opt_.set_config(planner_config_.ma_spline_opt_params);
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
        Trajectory<5, 2> minco_opt_traj;
        ma_spline_opt::MAsplineOutput ma_spline_traj;
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
    auto minco_plan(
        const utils::RobotState& goal_pose,
        const utils::RobotState& current_pose,
        std::shared_ptr<grid_map::GridMap> grid_map
    ) -> path;
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
        const double hard_clearance = std::max(0.05, safe_threshold * 0.2);

        Eigen::Vector2d diff = (old_goal_pose_.p - goal_pose.p).head<2>();
        Eigen::Vector2d threshold = planner_config_.replan_params.goal_deviation.head<2>();
        if (diff.cwiseAbs().x() > threshold.x() || diff.cwiseAbs().y() > threshold.y()) {
            need_replan_ = true;
        }

        const bool path_collision = result_.planning_traj.optimized_path.empty()
            || check_collision(result_.planning_traj, *grid_map, hard_clearance);

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

            const Eigen::Vector3d& current_vel =
                Eigen::Vector3d(current_pose.v.x(), current_pose.v.y(), current_pose.wz);
            path_planning.set_velocity(current_vel, Eigen::Vector3d::Zero());

            auto trajectory = path_planning.path_planning(start, goal, 5000);

            if (!trajectory.has_value()) {
                logger::fsm_replan->warn("planning failed");
                need_replan_ = true;
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
            minco_opt::GridMapESDF grid_map_esdf(grid_map);
            ma_opt_.set_esdf_interface(&grid_map_esdf);
            auto ma_intput = ma_spline_opt::from_path_planning_trajectory(trajectory.value());
            auto ma_output = ma_opt_.optimize(ma_intput);
            result_.ma_spline_traj = ma_output;

            result_.path_state = PathState::SUCCESSED;
            old_goal_pose_ = goal_pose;
            need_replan_ = false;
            return result_;
        }
        return result_;
    }
    auto one_plan(
        const utils::RobotState& goal_pose,
        const utils::RobotState& current_pose,
        std::shared_ptr<grid_map::GridMap> grid_map
    ) -> path {
        utils::TimeConsuming timer("planner_fsm", true); // true 表示允许打印
        path_planning.set_map(*grid_map);
        Eigen::Vector2d start(current_pose.p.x(), current_pose.p.y());
        Eigen::Vector2d goal(goal_pose.p.x(), goal_pose.p.y());

        path_planning.set_use_jps(true);

        const Eigen::Vector3d& current_vel = Eigen::Vector3d(current_pose.v.x(), current_pose.v.y(), current_pose.wz);
        path_planning.set_velocity(current_vel, Eigen::Vector3d::Zero());

        auto trajectory = path_planning.path_planning(start, goal, 5000);
        if (!trajectory.has_value()) {
            logger::fsm_replan->warn("planning failed");
            return tl::make_unexpected(PathError::PLANNING_FAILED);
        }
        logger::fsm_replan->info(
            "path points: raw={} optimized={} total_time={:.2f}s",
            result_.planning_traj.raw_path.size(),
            result_.planning_traj.optimized_path.size(),
            result_.planning_traj.total_time
        );
        const auto& timed = result_.planning_traj.timed_trajectory;
        logger::fsm_replan->info(
            "timed_trajectory: size={}, first_t={:.3f}, last_t={:.3f}, total_time={:.3f}",
            timed.size(),
            timed.empty() ? -1.0 : timed.front().time,
            timed.empty() ? -1.0 : timed.back().time,
            result_.planning_traj.total_time
        );
        result_.planning_traj = trajectory.value();
        utils::TimeConsuming opt_timer("ma_opt", true);
        minco_opt::GridMapESDF grid_map_esdf(grid_map);
        ma_opt_.set_esdf_interface(&grid_map_esdf);
        auto ma_intput = ma_spline_opt::from_path_planning_trajectory(trajectory.value());
        auto ma_output = ma_opt_.optimize(ma_intput);
        result_.ma_spline_traj = ma_output;
        return result_;
    }

private:
    path_planning::PathPlanning path_planning;
    ma_spline_opt::MaSplineTrajectoryOptimizer ma_opt_;
    minco_opt::MincoOptimizer minco_; // MINCO 五阶优化器(非线程安全,单线程调用)
    enum MincoError { OPTFAIL, TIMEDOUT };

private:
    /**replan */
    Trajectory<5, 2> last_trajectory_; ///< 上一条优化后的轨迹 (S=3, Quintic)
    bool has_valid_trajectory_ = false; ///< 是否有有效的历史轨迹
    std::chrono::system_clock::time_point last_trajectory_time_; ///< 上一条轨迹的发布时间戳
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
    /**
     * @brief 轨迹投影结果结构体
     *
     * 存储在历史轨迹上查找投影点的结果，包括投影时间、距离、
     * 以及投影点处的位置、速度和加速度信息。
     */
    struct TrajectoryProjectionResult {
        bool valid = false; ///< 投影是否有效
        double projection_time = 0.0; ///< 投影点在轨迹上的时间参数
        double distance = 0.0; ///< 机器人当前位置到投影点的距离
        Eigen::Vector2d position; ///< 投影点位置
        Eigen::Vector2d velocity; ///< 投影点速度
        Eigen::Vector2d acceleration; ///< 投影点加速度 (S=3 热启动)

        TrajectoryProjectionResult():
            position(Eigen::Vector2d::Zero()),
            velocity(Eigen::Vector2d::Zero()),
            acceleration(Eigen::Vector2d::Zero()) {}
    };
    /**
     * @brief 在历史轨迹上查找距离当前机器人位置最近的投影点
     *
     * 通过在轨迹时间参数上进行搜索，找到距离机器人当前位置最近的点。
     * 该方法用于实现平滑重规划：当机器人偏离轨迹不大时，使用投影点
     * 的位置和速度作为新轨迹的初始边界条件，而非零速假设。
     *
     * @param traj 要搜索的轨迹
     * @param robot_pos 机器人当前2D位置
     * @param traj_start_time 轨迹的起始绝对时间
     * @param current_time 当前时间
     * @return 投影结果，包含投影点的位置、速度和距离信息
     *
     * @note 搜索范围限制在 [当前相对时间, 轨迹总时长] 内，
     *       因为已经经过的轨迹部分没有参考意义。
     */
    auto find_projection_on_trajectory(
        const Trajectory<5, 2>& traj,
        const Eigen::Vector2d& robot_pos,
        const std::chrono::system_clock::time_point& traj_start_time,
        const std::chrono::system_clock::time_point& current_time
    ) -> TrajectoryProjectionResult;

private:
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

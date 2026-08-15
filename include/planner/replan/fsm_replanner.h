#pragma once
#include "planner/path_planning/path_planning.hpp"
#include "planner/traj_optimize/minco_opt/grid_map_esdf.hpp"
#include "planner/traj_optimize/minco_opt/minco_optimizer.hpp"
#include "planner/path_planning/post_processing.h"
#include "utils/expected.hpp"
#include "utils/logger.hpp"
#include "utils/type_utils.hpp"
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
namespace replan {
class FsmReplan {
public:
    //重规划触发参数
    struct ReplanParam {
        Eigen::Vector3d deviation;
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
        path_planning::PathPostProcessing::PathPostProcessingParams path_planning_params;
    } planner_config_;
    auto set_param(const PlannerConfig& planner_config) -> void {
        planner_config_ = planner_config;
        path_planning.set_param(planner_config_.path_planning_params);
    };
    FsmReplan()=default;
    explicit FsmReplan(const PlannerConfig& planner_config) {
        planner_config_ = planner_config;
        path_planning.set_param(planner_config_.path_planning_params);
    }
    enum PathError {
        NONE,
        SUCCESS,
        FAILED,
    };
    using path = tl::expected<std::pair<path_planning::PathPostProcessing::Trajectory,Trajectory<5, 2>>, PathError>;
    auto plan(
        const utils::RobotState& goal_pose,
        const utils::RobotState& current_pose,
        std::shared_ptr<grid_map::GridMap> grid_map
    ) -> path {
        utils::TimeConsuming timer("planner_fsm", true); // true 表示允许打印
        path_planning.set_map(*grid_map);
        Eigen::Vector2d start(current_pose.p.x(), current_pose.p.y());
        Eigen::Vector2d goal(goal_pose.p.x(), goal_pose.p.y());
        //logger::fsm_replan->info("start planning");
        path_planning.set_use_jps(true);
        auto trajectory=path_planning.path_planning(start, goal, 5000);
        if(!trajectory.has_value()){
            logger::fsm_replan->info("planning failed");
            return tl::make_unexpected(PathError::FAILED);
        }
        utils::TimeConsuming timer_("MINCO", true); // true 表示允许打印
        // MINCO 五阶轨迹优化(时间 + 平滑 + ESDF 避障 + 速度/加速度软约束)。
        // 优化失败或安全检查不过时保持 path_planning 原始轨迹,安全兜底。
        auto traj = trajectory.value();
        auto  minco_path = minco_optimize(traj, grid_map, current_pose);
        if(!minco_path){
            return tl::make_unexpected(PathError::FAILED);
        }
        return std::make_pair(traj, minco_path.value());
    };

private:
    
    path_planning::PathPlanning path_planning;
    minco_opt::MincoOptimizer minco_; // MINCO 五阶优化器(非线程安全,单线程调用)
    enum MincoError{
        FAIL
    };
    // MINCO 轨迹优化:成功返回 true 并原地更新 traj;失败返回 false,traj 保持原样。
    auto minco_optimize(
        path_planning::PathPostProcessing::Trajectory& traj,
        std::shared_ptr<grid_map::GridMap> grid_map,
        const utils::RobotState& current_pose) -> tl::expected<Trajectory<5, 2>,MincoError>;

    // 把优化后的五阶样条采样回填到 Trajectory(下游 MPC/可视化格式不变)。
    auto rebuild_optimized_trajectory(
        path_planning::PathPostProcessing::Trajectory& traj,
        const Trajectory<5, 2>& spline,
        const path_planning::PathPostProcessing::PathPostProcessingParams& pp_params) -> void;
    
};

// ======================= MINCO 优化接线(类外内联实现) =======================
// 注意:定义在头文件中的类外成员函数必须显式 inline,避免多 TU 包含时重复定义。

inline auto FsmReplan::minco_optimize(
    path_planning::PathPostProcessing::Trajectory& traj,
    std::shared_ptr<grid_map::GridMap> grid_map,
    const utils::RobotState& current_pose) -> tl::expected<Trajectory<5, 2>,MincoError> {
    if (!grid_map) return tl::make_unexpected(MincoError::FAIL);
    const auto& pp_params = planner_config_.path_planning_params;

    // 航点太少没有优化意义(至少 4 个点才有 3 段)
    if (traj.optimized_path.size() < 4) return tl::make_unexpected(MincoError::FAIL);;

    // 段数 N:最多 6 段(决策维 = 2(N-1)+N),首尾航点固定
    const int N = std::min(6, static_cast<int>(traj.optimized_path.size()) - 1);
    if (N < 2) return tl::make_unexpected(MincoError::FAIL);;

    // ① 子采样 N+1 个航点(首尾 = 起点/终点)
    std::vector<Eigen::Vector2d> pts;
    pts.reserve(static_cast<size_t>(N) + 1);
    const int M = static_cast<int>(traj.optimized_path.size()) - 1;
    for (int i = 0; i <= N; ++i) {
        const int idx = static_cast<int>(std::lround(static_cast<double>(i) * M / N));
        pts.push_back(traj.optimized_path[static_cast<size_t>(std::clamp(idx, 0, M))]);
    }

    // ② 初始路点(2 x (N-1))
    Eigen::Matrix2Xd init_points(2, N - 1);
    for (int i = 0; i < N - 1; ++i) {
        init_points.col(i) = pts[static_cast<size_t>(i + 1)];
    }

    // ③ 初始时间:均匀分配 path_planning 的总时间(暂不考虑重规划热启动)
    const double total_T = std::max(traj.total_time, 0.5);
    Eigen::VectorXd init_times(N);
    for (int i = 0; i < N; ++i) init_times(i) = total_T / N;

    // ④ PVA 边界(行 = x/y,列 = P/V/A):起点速度取当前速度,终点静止
    Eigen::Matrix<double, 2, 3> head_pva, tail_pva;
    head_pva << traj.start_state_XYTheta.x(), current_pose.v.x(), 0.0,
                traj.start_state_XYTheta.y(), current_pose.v.y(), 0.0;
    tail_pva << traj.final_state_XYTheta.x(), 0.0, 0.0,
                traj.final_state_XYTheta.y(), 0.0, 0.0;

    // ⑤ 优化器配置(权重先用合理默认值,后续可挪到 yaml)
    minco_opt::MincoOptimizerConfig cfg;
    cfg.weight_smooth = 1.0;          // jerk 平滑
    cfg.weight_obstacle = 100.0;      // ESDF 避障
    cfg.weight_feasibility = 10.0;    // 速度/加速度可行性
    cfg.weight_time = 10.0;           // 时间正则
    cfg.weight_mean_time = 10.0;      // 平均时间约束
    cfg.max_vel = pp_params.max_vel;
    cfg.max_acc = pp_params.max_acc;
    cfg.safe_distance = pp_params.safe_threshold;
    cfg.integral_resolution = 8;      // 每段避障/可行性采样点数
    cfg.max_iterations = 100;
    cfg.g_epsilon = 1e-4;
    cfg.min_time = 0.1;

    // ⑥ 执行优化(每次重建 ESDF 适配器;MincoOptimizer 非线程安全,单线程使用)
    minco_.setESDFInterface(std::make_shared<minco_opt::GridMapESDF>(grid_map));
    minco_.setConfig(cfg);
    minco_.initialize(head_pva, tail_pva, N);
    if (!minco_.optimize(init_points, init_times)) {
        logger::fsm_replan->warn("MINCO optimize failed, keep raw trajectory");
        return tl::make_unexpected(MincoError::FAIL);;
    }

    Trajectory<5, 2> spline;
    minco_.getTrajectory(spline);

    // ⑦ 密集采样安全检查:净空 / 速度 / 加速度,任一超限 -> 回退原始轨迹
    const double hard_clearance = planner_config_.replan_params.hard_clearance;
    const double vel_lim = cfg.max_vel * 1.05;
    const double acc_lim = cfg.max_acc * 1.05;
    const double dt_check = 0.02;
    for (int i = 0; i < spline.getPieceNum(); ++i) {
        const auto& piece = spline[i];
        const double dur = piece.getDuration();
        for (double t = 0.0; t <= dur; t += dt_check) {
            const Eigen::Vector2d p = piece.getPos(t);
            if (!grid_map->isInsideMap(p) || grid_map->getDistance(p) < hard_clearance) {
                logger::fsm_replan->warn(
                    "MINCO traj unsafe (clearance {:.3f} < {:.3f}), keep raw trajectory",
                    grid_map->getDistance(p), hard_clearance);
                return tl::make_unexpected(MincoError::FAIL);;
            }
            if (piece.getVel(t).norm() > vel_lim || piece.getAcc(t).norm() > acc_lim) {
                logger::fsm_replan->warn("MINCO traj unsafe (vel/acc), keep raw trajectory");
                return tl::make_unexpected(MincoError::FAIL);;
            }
        }
    }

    return spline;
}
} // namespace replan

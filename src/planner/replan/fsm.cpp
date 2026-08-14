#include "utils/logger.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <planner/fsm/fsm.hpp>
#include <limits>
namespace planner {

// steady_clock 秒（供路径年龄 / 防抖计时用）
static double now_sec() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}



auto FSM::set_astar_param(path_planning::AStar& astar) {
    astar.setMaxVelocity(planner_config_.astar_param.max_vel);
    astar.setMaxAcceleration(planner_config_.astar_param.max_acc);
    astar.setTimeResolution(planner_config_.astar_param.time_resolution);
    astar.setMinTrajectoryNumber(planner_config_.astar_param.min_traj_num);
    astar.setTrajectoryCutLength(planner_config_.astar_param.traj_cut_length);
    astar.setDistanceWeight(planner_config_.astar_param.distance_weight);
    astar.setYawWeight(planner_config_.astar_param.yaw_weight);
}

auto FSM::plan(
    const utils::RobotState& goal_pose,
    const utils::RobotState& current_pose,
    std::shared_ptr<grid_map::GridMap> grid_map
) -> astar_opt_path {
    if (checkPointEqual(old_goal_pose_.p, goal_pose.p, planner_config_.fsm_param.deviation)) {
        // 目标没变，但需要检查当前已规划路径是否仍然有效
        if (path_state_ == PathState::running || path_state_ == PathState::successed) {
            // 触发重规划的三个条件（满足任一即重规划）：
            //   1. 路径碰障（A* 原始网格路径 + 优化后航点路径，见 checkCollision）
            //   2. 路径年龄超过 replan_interval_
            //   3. 机器人偏离参考路径超过 replan_lateral_dev_
            const double now = now_sec();
            bool need_replan = false;

            // 重规划原因细分(调试可观测):分别计算三个触发条件并打日志
            const bool path_collision = astar_traj_.optimized_path.empty() || checkCollision(astar_traj_);
            const double path_age = now - last_plan_time_;
            const double lateral_dev =
                lateralDeviation(Eigen::Vector2d(current_pose.p.x(), current_pose.p.y()),
                                 last_opt_path_.empty() ? astar_traj_.optimized_path
                                                        : last_opt_path_);

            if (path_collision) {
                logger::fsm_replan->info(
                              "[replan] trigger=path_collision astar_pts={} opt_pts={} safe_thr={:.3f}",
                              astar_traj_.raw_path.size(),
                              astar_traj_.optimized_path.size(),
                              planner_config_.safe_threshold);
                need_replan = true;
            }
            else if (path_age > planner_config_.fsm_param.replan_interval) {
                logger::fsm_replan->info(
                              "[replan] trigger=stale age={:.2f}s > interval={:.2f}s",
                              path_age, planner_config_.fsm_param.replan_interval);
                need_replan = true;
            }
            else if (lateral_dev > planner_config_.fsm_param.replan_lateral_dev) {
                logger::fsm_replan->info(
                              "[replan] trigger=lateral_dev dev={:.3f}m > threshold={:.3f}m",
                              lateral_dev, planner_config_.fsm_param.replan_lateral_dev);
                need_replan = true;
            }
            if (!need_replan) {
                // 路径仍然有效，且目标未变，可以直接返回成功（避免重复规划）
                return tl::make_unexpected(path_error::success);
            }
            // 防抖：距上次重规划不足 min_replan_interval_ 时保留旧路径，下一拍再试
            if (now - last_replan_time_ < planner_config_.fsm_param.min_replan_interval) {
                logger::fsm_replan->info(
                              "[replan] debounce skip: {:.2f}s < min_interval={:.2f}s",
                              now - last_replan_time_, planner_config_.fsm_param.min_replan_interval);
                return tl::make_unexpected(path_error::success);
            }
            path_state_ = PathState::idle;
            // 继续执行后续的重规划逻辑（不返回）
        } else {
            // 其他状态（如 idle, failed）也应该继续尝试规划
        }
    }
    if (!checkPointEqual(old_goal_pose_.p, goal_pose.p, Eigen::Vector3d::Zero())) {
        logger::fsm_replan->debug("plan goal is change");
        old_goal_pose_ = goal_pose;
        path_state_ = PathState::idle;
    }

    // 起点≈终点（机器人已到达目标附近）：直接退出，不规划。
    // 否则 A* 返回单点退化路径、样条优化无意义，且周期重规划会反复触发失败日志
    if ((current_pose.p.head<2>() - goal_pose.p.head<2>()).norm() < planner_config_.fsm_param.goal_reached_radius) {
        return tl::make_unexpected(path_error::success);
    }

    // failed 状态也限频重试：A*/优化失败时不要每 33ms 空转刷屏
    if (path_state_ == PathState::failed && now_sec() - last_replan_time_ < planner_config_.fsm_param.min_replan_interval) {
        return tl::make_unexpected(path_error::success);
    }

    // 2. 准备数据
    grid_map_ = grid_map;

    Eigen::Vector2d start(current_pose.p.x(), current_pose.p.y());
    Eigen::Vector2d goal(goal_pose.p.x(), goal_pose.p.y());

    // 若起点/终点过于靠近障碍物，沿 ESDF 梯度外推到安全点，保证可规划
    start = getSafeStart(start);
    goal = getSafeStart(goal);
    Eigen::Vector2d start_vel(current_pose.v.x(), current_pose.v.y());
    // A* 梯形速度剖面的末端速度(标量,与方向无关)
    const double goal_speed = 0.3;
    // 目标 yaw 方向(用于末端速度方向约束,见循环内 end_vel 计算)
    const Eigen::Vector2d goal_dir(std::cos(goal_pose.yaw), std::sin(goal_pose.yaw));
    // 3. 带重试的规划循环
    const int max_retries = 100;
    int retry_count = 0;
    path_planning::AStar astar(*grid_map, planner_config_.safe_threshold);
    set_astar_param(astar);
    astar.setStartVelocity(start_vel.norm());
    astar.setEndVelocity(goal_speed);
    while (path_state_ != PathState::running && retry_count < max_retries) {
        auto astar_traj = astar.planWithPostProcessing(start, goal, 5000);
        astar_traj_ = astar_traj;
        old_goal_pose_ = goal_pose;

        if (astar_traj.optimized_path.empty()) {
            logger::fsm_replan->error("A* planning failed!");
            logger::fsm_replan->error(
                "  [diag] safe_threshold={:.3f} map_size=({:.2f},{:.2f}) "
                "resolution={:.4f}",
                planner_config_.safe_threshold,
                grid_map_->getMapSize().x(),
                grid_map_->getMapSize().y(),
                grid_map_->getResolution()
            );
            logger::fsm_replan->error(
                "  [diag] start=({:.3f},{:.3f}) goal=({:.3f},{:.3f}) "
                "start_dist={:.3f} goal_dist={:.3f}",
                start.x(),
                start.y(),
                goal.x(),
                goal.y(),
                grid_map_->getDistance(start),
                grid_map_->getDistance(goal)
            );
            path_state_ = PathState::failed;
            last_replan_time_ = now_sec(); // 失败也吃防抖，避免 30Hz 空转
            return tl::make_unexpected(path_error::astar_path_empty);
        }

        // 碰撞检测
        if (checkCollision(astar_traj_)) {
            logger::fsm_replan->warn("Collision detected, retrying... (attempt {}/{})", retry_count + 1, max_retries);
            path_state_ = PathState::idle; // 重置为 idle 以继续循环
            retry_count++;
            continue; // 重新执行 A*
        } else {
            // logger::fsm_replan->info("collision is not failed");
        }

        // A* 诊断:点数/长度/时间/速度剖面
        logger::fsm_replan->info(
                      "[astar] raw_pts={} opt_pts={} len={:.3f}m weighted={:.3f}m time={:.3f}s "
                      "avg_v={:.3f}m/s v_max={:.2f} end_v={:.2f}",
                      astar_traj.raw_path.size(),
                      astar_traj.optimized_path.size(),
                      astar_traj.total_length,
                      astar_traj.weighted_length,
                      astar_traj.total_time,
                      astar_traj.total_length / std::max(astar_traj.total_time, 1e-6),
                      planner_config_.astar_param.max_vel,
                      goal_speed);

        // 无碰撞，进入优化阶段
        path_state_ = PathState::running;

        auto start_time = std::chrono::high_resolution_clock::now();

        // ===== 旧优化器 (TrajOpt::TrajectoryOptimizer, cubic) 已停用 =====
        // 防止 total_time 为 0 时产生 NaN（短/退化路径的兜底，正常情况下不会被触发）
        // const double safe_total_time = (astar_traj.total_time > 1e-6) ? astar_traj.total_time : 1e-6;
        // fsm_config_.params_.piece_len = astar_traj.total_length / safe_total_time;
        // fsm_config_.params_.total_time = astar_traj.total_time;
        // fsm_config_.params_.total_len = astar_traj.total_length;
        //
        // TrajOpt::TrajectoryOptimizer optimizer(grid_map, astar_traj.optimized_path, fsm_config_.params_);
        // optimizer.set_start_vel(start_vel);
        // optimizer.set_end_vel(goal_vel);
        // if (!optimizer.plan()) { ... }

        // ===== 新优化器 (Opt::EsdfTrajectoryOptimizer, quintic) =====
        // 参数先用结构体默认值(不在 yaml 加载),仅把 A* 的 total_time 作为时间初值传入
        Opt::OptimizerParams opt_params = planner_config_.opt_params;
        const double safe_total_time = (astar_traj.total_time > 1e-6) ? astar_traj.total_time : 1e-6;
        opt_params.total_time = safe_total_time;

        // 末端速度方向约束 = 目标 yaw,但仅当目标 yaw 与 A* 到达方向差异不大时生效:
        // 差异过大(如掉头)时 quintic 必须用空间弯曲表达方向变化,末段会扭曲/绕行,
        // 此时放弃 yaw 约束(默认沿末段方向停车),朝向改由控制层“到位旋转”完成
        Eigen::Vector2d end_vel = Eigen::Vector2d::Zero();
        if (astar_traj.optimized_path.size() >= 2)
        {
            const auto &p_last = astar_traj.optimized_path.back();
            const auto &p_prev = astar_traj.optimized_path[astar_traj.optimized_path.size() - 2];
            const double approach_yaw = std::atan2(p_last.y() - p_prev.y(), p_last.x() - p_prev.x());
            const double yaw_err = std::remainder(goal_pose.yaw - approach_yaw, 2.0 * M_PI);
            if (std::abs(yaw_err) < M_PI / 3.0) // 60° 以内才约束,避免末段扭曲
            {
                end_vel = 0.2 * goal_dir; // 幅值 0.2:只约束方向(末端切线 = 目标 yaw)
            }
        }

        Opt::EsdfTrajectoryOptimizer optimizer(grid_map, astar_traj.optimized_path, opt_params);
        optimizer.setStartVel(start_vel);
        optimizer.setEndVel(end_vel);
        if (!optimizer.plan()) {
            logger::fsm_replan->warn(
                "[opt-fail] astar: {} pts, len={:.3f} m, t={:.3f} s, start=({:.2f},{:.2f}), goal=({:.2f},{:.2f}) err={}",
                astar_traj.optimized_path.size(),
                astar_traj.total_length,
                astar_traj.total_time,
                start.x(), start.y(), goal.x(), goal.y(),
                optimizer.lastError()
            );
            path_state_ = PathState::failed;
            last_replan_time_ = now_sec(); // 失败也吃防抖，避免 30Hz 空转
            return tl::make_unexpected(path_error::optimizer_failed);
        }

        // 优化结果安全校验：软惩罚可能收敛到“贴墙/穿障”解，密集采样复核
        // （采样点硬净空 + 采样点之间线段碰撞），不合格直接拒收，
        // 保证穿障轨迹永远不被下发（旧轨迹继续执行，下一拍再试）
        {
            const double hard_clearance = planner_config_.fsm_param.hard_clearance; // 硬净空（m）：低于此值视为穿障
            double min_dist = std::numeric_limits<double>::max();
            Eigen::Vector2d min_pos = Eigen::Vector2d::Zero();
            bool opt_safe = true;
            auto dense = optimizer.sampleTrajectory(0.02);
            for (size_t i = 0; i < dense.size(); ++i) {
                if (!grid_map_->isInsideMap(dense[i])) {
                    min_pos = dense[i];
                    opt_safe = false;
                    break;
                }
                const double d = grid_map_->getDistance(dense[i]);
                if (d < min_dist) {
                    min_dist = d;
                    min_pos = dense[i];
                }
                if (d < hard_clearance) {
                    opt_safe = false;
                    break;
                }
                if (i > 0 && grid_map_->isLineOccupancy(dense[i - 1], dense[i])) {
                    min_pos = dense[i];
                    opt_safe = false;
                    break;
                }
            }
            if (!opt_safe) {
                logger::fsm_replan->warn(
                    "[opt-check] optimized trajectory unsafe at ({:.2f},{:.2f}) min_clearance={:.3f}m < {:.2f}m, rejected",
                    min_pos.x(),
                    min_pos.y(),
                    min_dist,
                    hard_clearance
                );
                path_state_ = PathState::failed;
                last_replan_time_ = now_sec(); // 拒收也吃防抖，避免 30Hz 空转
                return tl::make_unexpected(path_error::optimizer_failed);
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        auto metrics = optimizer.evaluate();
        logger::fsm_replan->info(
                      "[opt-diag] dur={:.3f}s v_min={:.3f} v_max={:.3f} v_avg={:.3f} a_max={:.3f} "
                      "clearance={:.3f}m energy={:.3f} rms_jerk={:.3f} opt_ms={} astar_len={:.3f}m",
                      metrics.duration,
            metrics.min_velocity,
            metrics.max_velocity,
            metrics.avg_velocity,
            metrics.max_acceleration,
            metrics.min_clearance,
            metrics.trajectory_energy,
            metrics.rms_jerk,
            duration.count(),
            astar_traj.total_length
        );

        std::vector<Eigen::Vector2d> opt_path = optimizer.sampleTrajectory(0.1);
        // 记录成功规划时间（供路径年龄 / 防抖使用）与优化轨迹采样点（供横向偏差检测使用）
        last_plan_time_ = now_sec();
        last_replan_time_ = last_plan_time_;
        last_opt_path_ = optimizer.sampleTrajectory(0.2);
        return std::make_pair(astar_traj.optimized_path, optimizer);
    }

    // 循环结束仍未成功（状态不是 running 或重试耗尽）
    if (retry_count >= max_retries) {
        logger::fsm_replan->error("Max retries reached, planning failed due to collisions");
        path_state_ = PathState::failed;
        return tl::make_unexpected(path_error::astar_path_empty);
    }

    return tl::make_unexpected(path_error::none);
}

auto FSM::getSafeStart(const Eigen::Vector2d& pos) -> Eigen::Vector2d {
    if (!grid_map_ || !grid_map_->isInsideMap(pos)) {
        return pos;
    }
    if (grid_map_->getDistance(pos) >= planner_config_.safe_threshold) {
        return pos;
    }
    Eigen::Vector2d safe = pos;
    const double max_push = 1.5; // 最大外推距离，避免把起点推得太远
    double pushed = 0.0;
    for (int i = 0; i < 200; ++i) {
        double d = 0.0;
        Eigen::Vector2d g;
        grid_map_->getDistanceAndGradient(safe, d, g);
        double gn = g.norm();
        if (gn < 1e-6) {
            break; // 梯度退化，无法继续外推
        }
        Eigen::Vector2d dir = g / gn; // 梯度指向远离障碍方向
        double need = (planner_config_.safe_threshold - d) + 0.05;
        double step = std::min(need, max_push - pushed);
        if (step <= 0) {
            break;
        }
        safe += dir * step;
        pushed += step;
        if (grid_map_->getDistance(safe) >= planner_config_.safe_threshold || pushed >= max_push) {
            return safe;
        }
    }
    return safe;
}

auto FSM::checkCollision(path_planning::AStar::Trajectory astar_traj) -> bool {
    // 同时检查 A* 原始网格路径与优化后航点路径：
    // 原始路径逐格密集，可发现两个优化航点之间被新障碍挡住的情况
    std::vector<std::vector<Eigen::Vector2d>> paths;
    paths.push_back(astar_traj.optimized_path);
    paths.push_back(astar_traj.raw_path);
    for (const auto& path : paths) {
        // Points outside map are considered collision-free
        for (auto pos : path) {
            if (!grid_map_->isInsideMap(pos)) {
                continue;
            }

            // Points inside map use safety distance check
            if (grid_map_->getDistance(pos) < planner_config_.safe_threshold) {
                return true;
            };
        }
    }
    return false;
};

auto FSM::lateralDeviation(const Eigen::Vector2d& pos, const std::vector<Eigen::Vector2d>& path) -> double {
    if (path.size() < 2) {
        return std::numeric_limits<double>::max();
    }
    // 点到折线各段的最短距离（逐段投影并夹紧到线段内）
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 1; i < path.size(); ++i) {
        const Eigen::Vector2d a = path[i - 1];
        const Eigen::Vector2d b = path[i];
        const Eigen::Vector2d ab = b - a;
        const double len2 = ab.squaredNorm();
        double t = 0.0;
        if (len2 > 1e-12) {
            t = std::max(0.0, std::min(1.0, (pos - a).dot(ab) / len2));
        }
        const Eigen::Vector2d closest = a + t * ab;
        min_dist = std::min(min_dist, (pos - closest).norm());
    }
    return min_dist;
}

auto FSM::checkPointEqual(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, const Eigen::Vector3d& deviation)
    -> bool {
    if (deviation == Eigen::Vector3d::Zero()) {
        if (std::abs(pos1.x() - pos2.x()) < std::numeric_limits<double>::epsilon()
            && std::abs(pos1.y() - pos2.y()) < std::numeric_limits<double>::epsilon())
        {
            return true;
        }
        return false;
    }
    if (std::abs(pos1.x() - pos2.x()) < deviation.x() && std::abs(pos1.y() - pos2.y()) < deviation.y()) {
        return true;
    }
    return false;
}
} // namespace planner
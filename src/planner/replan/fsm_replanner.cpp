#include "planner/replan/fsm_replanner.h"
#include "utils/expected.hpp"
#include "utils/logger.hpp"
#include "utils/type_utils.hpp"
namespace replan {
// 折线碰撞检测辅助函数
static bool check_path_collision(
    const std::vector<Eigen::Vector2d>& path,
    const grid_map::GridMap& grid_map,
    const double& safe_threshold,
    const double step
);
auto FsmReplan::plan(
    const utils::RobotState& goal_pose,
    const utils::RobotState& current_pose,
    std::shared_ptr<grid_map::GridMap> grid_map
) -> path {
    // 起点≈终点（机器人已到达目标附近）：直接退出，不规划。
    if ((current_pose.p.head<2>() - goal_pose.p.head<2>()).norm() < planner_config_.replan_params.goal_reached_radius) {
        path_state_ = PathState::SUCCESSED;
        result_.path_state = path_state_;
        return result_;
    }
    
    // if(grid_map->is_tunnel(current_pose.p.head<2>())){
    //     logger::fsm_replan->error("now robot in tunnel");
    // }


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
        utils::TimeConsuming timer("planner_fsm", true); // true 表示允许打印
        path_planning.set_map(*grid_map);

        Eigen::Vector2d start(current_pose.p.x(), current_pose.p.y());
        Eigen::Vector2d goal(goal_pose.p.x(), goal_pose.p.y());

        // 若起点/终点过于靠近障碍物，沿 ESDF 梯度外推到安全点，保证可规划
        start = get_safe_pos(start, *grid_map, safe_threshold);
        goal = get_safe_pos(goal, *grid_map, safe_threshold);

        path_planning.set_use_jps(true);

        const Eigen::Vector3d& current_vel = Eigen::Vector3d(current_pose.v.x(), current_pose.v.y(), current_pose.wz);
        path_planning.set_velocity(current_vel, Eigen::Vector3d::Zero());

        auto trajectory = path_planning.path_planning(start, goal, current_pose.yaw, goal_pose.yaw, 5000);

        if (!trajectory.has_value()) {
            logger::warn(logger::fsm_replan,"planning failed");
            need_replan_ = true;
            return tl::make_unexpected(PathError::PLANNING_FAILED);
        }

        // MINCO 五阶轨迹优化(时间 + 平滑 + ESDF 避障 + 速度/加速度软约束)。
        // 优化失败或安全检查不过时保持 path_planning 原始轨迹,安全兜底。
        result_.planning_traj = trajectory.value();
        // 打印 raw_path / optimized_path 点数,确认直线/近距离时的短路情况
        // logger::info(logger::fsm_replan,
        //     "path points: raw={} optimized={} total_time={:.2f}s",
        //     result_.planning_traj.raw_path.size(),
        //     result_.planning_traj.optimized_path.size(),
        //     result_.planning_traj.total_time
        // );
        minco_opt::GridMapESDF grid_map_esdf(grid_map);
        ma_opt_.set_esdf_interface(&grid_map_esdf);
        auto ma_intput = ma_spline_opt::from_path_planning_trajectory(trajectory.value());
        
        
        ma_intput.model=ma_spline_opt::OptModel::OMNI_XY;
        
        
        auto ma_output = ma_opt_.optimize(ma_intput);
        if(ma_output.success==false){
            logger::info(logger::fsm_replan,"ma opt failed");
            return tl::make_unexpected(PathError::MINCO_OPT_FIALED);
        }
        result_.ma_spline_traj = ma_output;

        result_.path_state = PathState::SUCCESSED;
        old_goal_pose_ = goal_pose;
        need_replan_ = false;
        return result_;
    }
    return result_;
}
auto FsmReplan::one_plan(
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

    auto trajectory = path_planning.path_planning(start, goal, current_pose.yaw, goal_pose.yaw, 5000);
    if (!trajectory.has_value()) {
        logger::warn(logger::fsm_replan,"planning failed");
        return tl::make_unexpected(PathError::PLANNING_FAILED);
    }
    logger::info(logger::fsm_replan,
        "path points: raw={} optimized={} total_time={:.2f}s",
        result_.planning_traj.raw_path.size(),
        result_.planning_traj.optimized_path.size(),
        result_.planning_traj.total_time
    );
    const auto& timed = result_.planning_traj.timed_trajectory;
    logger::info(logger::fsm_replan,
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
auto FsmReplan::minco_plan(
    const utils::RobotState& goal_pose,
    const utils::RobotState& current_pose,
    std::shared_ptr<grid_map::GridMap> grid_map
) -> path {
    // 起点≈终点（机器人已到达目标附近）：直接退出，不规划。
    if ((current_pose.p.head<2>() - goal_pose.p.head<2>()).norm() < planner_config_.replan_params.goal_reached_radius) {
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
    if (diff.cwiseAbs().x() > threshold.x() || diff.cwiseAbs().y() > threshold.y()) {
        // 任一轴超出阈值即触发重规划
        need_replan_ = true;
    }
    //路径碰障（jps优化后航点路径 + 实际跟踪的 MINCO 样条轨迹）
    const double collision_step = std::max(planner_config_.path_planning_params.dense_sample_resolution, 0.02);
    const double hard_clearance = std::max(0.05, safe_threshold * 0.5);

    const bool path_collision = result_.planning_traj.optimized_path.empty()
        || check_collision(result_.planning_traj, *grid_map, hard_clearance)
        || (!last_opt_path_.empty() && check_path_collision(last_opt_path_, *grid_map, hard_clearance, collision_step));

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

        const Eigen::Vector3d& current_vel = Eigen::Vector3d(current_pose.v.x(), current_pose.v.y(), current_pose.wz);
        path_planning.set_velocity(current_vel, Eigen::Vector3d::Zero());

        auto trajectory = path_planning.path_planning(start, goal, current_pose.yaw, goal_pose.yaw, 5000);
        if (!trajectory.has_value()) {
            logger::warn(logger::fsm_replan,"planning failed");
            need_replan_ = true;
            return tl::make_unexpected(PathError::PLANNING_FAILED);
        }

        // MINCO 五阶轨迹优化(时间 + 平滑 + ESDF 避障 + 速度/加速度软约束)。
        // 优化失败或安全检查不过时保持 path_planning 原始轨迹,安全兜底。
        result_.planning_traj = trajectory.value();
        // [临时诊断] 打印 raw_path / optimized_path 点数,确认直线/近距离时的短路情况
        logger::info(logger::fsm_replan,
            "path points: raw={} optimized={} total_time={:.2f}s",
            result_.planning_traj.raw_path.size(),
            result_.planning_traj.optimized_path.size(),
            result_.planning_traj.total_time
        );

        auto minco_path = minco_optimize(result_.planning_traj, grid_map, current_pose);
        if (!minco_path) {
            need_replan_ = true;
            return tl::make_unexpected(PathError::MINCO_OPT_FIALED);
        }
        result_.path_state = PathState::SUCCESSED;
        old_goal_pose_ = goal_pose;
        last_opt_path_ = sample_minco_trajectory(minco_path.value(), 0.02);
        result_.minco_opt_traj = minco_path.value();
        need_replan_ = false;
        return result_;
    }
    return result_;
}

auto FsmReplan::lateral_deviation(const Eigen::Vector2d& pos, const std::vector<Eigen::Vector2d>& path) -> double {
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
            //计算点 pos 到线段 [a, b] 的投影参数 t，并将其限制在 [0, 1] 区间内
            t = std::max(0.0, std::min(1.0, (pos - a).dot(ab) / len2));
        }
        //点 pos 到线段 [a, b] 的投影点
        const Eigen::Vector2d closest = a + t * ab;
        min_dist = std::min(min_dist, (pos - closest).norm());
    }
    return min_dist;
}

static bool check_path_collision(
    const std::vector<Eigen::Vector2d>& path,
    const grid_map::GridMap& grid_map,
    const double& safe_threshold,
    const double step
) {
    if (path.empty()) {
        return false;
    }
    auto is_unsafe = [&](const Eigen::Vector2d& pos) -> bool {
        // Points outside map are considered collision-free
        if (!grid_map.isInsideMap(pos)) {
            return false;
        }
        // Points inside map use safety distance check
        return grid_map.getDistance(pos) < safe_threshold;
    };

    for (size_t i = 0; i < path.size(); ++i) {
        if (is_unsafe(path[i])) {
            return true;
        }
        if (i + 1 >= path.size()) {
            continue;
        }
        // 只检查航点会漏掉长直线段中间新增的动态障碍物，
        // 因此沿每一段按 dense_sample_resolution 再采样检查。
        const Eigen::Vector2d a = path[i];
        const Eigen::Vector2d b = path[i + 1];
        const double len = (b - a).norm();
        if (len < 1e-9) {
            continue;
        }
        const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
        for (int j = 1; j < n; ++j) {
            const double r = static_cast<double>(j) / n;
            if (is_unsafe(a + r * (b - a))) {
                return true;
            }
        }
    }
    return false;
}

auto FsmReplan::check_collision(
    path_planning::PathPostProcessing::Trajectory traj,
    const grid_map::GridMap& grid_map,
    const double& safe_threshold
) -> bool {
    // 同时检查 planning原始网格路径与优化后航点路径：
    // 原始路径可发现两个优化航点之间被新障碍挡住的情况；
    // 再对每段折线做密集采样，避免长直线中间漏检。
    const double step = std::max(planner_config_.path_planning_params.dense_sample_resolution, 0.02);
    return check_path_collision(traj.optimized_path, grid_map, safe_threshold, step)
        || check_path_collision(traj.raw_path, grid_map, safe_threshold, step);
}
auto FsmReplan::get_safe_pos(
    const Eigen::Vector2d& pos,
    const grid_map::GridMap& grid_map,
    const double& safe_threshold
) -> Eigen::Vector2d {
    if (!grid_map.isInsideMap(pos)) {
        return pos;
    }
    if (grid_map.getDistance(pos) >= safe_threshold) {
        return pos;
    }
    Eigen::Vector2d safe = pos;
    const double max_push = 1.5; // 最大外推距离，避免把起点推得太远
    double pushed = 0.0;
    for (int i = 0; i < 200; ++i) {
        double d = 0.0;
        Eigen::Vector2d g;
        grid_map.getDistanceAndGradient(safe, d, g);
        double gn = g.norm();
        if (gn < 1e-6) {
            break; // 梯度退化，无法继续外推
        }
        Eigen::Vector2d dir = g / gn; // 梯度指向远离障碍方向
        double need = (safe_threshold - d) + 0.05;
        double step = std::min(need, max_push - pushed);
        if (step <= 0) {
            break;
        }
        safe += dir * step;
        pushed += step;
        if (grid_map.getDistance(safe) >= safe_threshold || pushed >= max_push) {
            return safe;
        }
    }
    return safe;
};
auto FsmReplan::check_point_equal(
    const Eigen::Vector3d& pos1,
    const Eigen::Vector3d& pos2,
    const Eigen::Vector3d& deviation
) -> bool {
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
auto FsmReplan::find_projection_on_trajectory(
    const Trajectory<5, 2>& traj,
    const Eigen::Vector2d& robot_pos,
    const std::chrono::system_clock::time_point& traj_start_time,
    const std::chrono::system_clock::time_point& current_time
) -> TrajectoryProjectionResult {
    TrajectoryProjectionResult result;

    // 1. 计算当前相对时间（相对于轨迹起始时间）
    double elapsed_time = std::chrono::duration<double>(current_time - traj_start_time).count();
    double total_duration = traj.getTotalDuration();

    // 如果轨迹已经完全过期，返回无效结果
    if (elapsed_time >= total_duration) {
        logger::warn(logger::fsm_replan,"轨迹已过期: elapsed={:2f} s, duration={:2f} s", elapsed_time, total_duration);
        return result; // valid = false
    }

    // 2. 确定搜索范围
    //    - 起始时间: max(0, elapsed_time) - 已经经过的部分没有参考意义
    //    - 结束时间: total_duration
    //    - 考虑到重规划预测时间，稍微向前看一点
    double search_start = std::max(0.0, elapsed_time);
    double search_end = total_duration;

    // 如果可搜索范围太小，返回无效
    if (search_end - search_start < planner_config_.replan_params.projection_search_resolution) {
        logger::warn(logger::fsm_replan,"轨迹剩余时间太短，无法进行投影搜索");
        return result; // valid = false
    }

    // 3. 在轨迹上搜索距离机器人最近的投影点
    //    采用均匀采样搜索，找到最小距离点
    double min_distance_sq = std::numeric_limits<double>::max();
    double best_time = search_start;

    // 计算搜索步数
    int num_samples =
        static_cast<int>((search_end - search_start) / planner_config_.replan_params.projection_search_resolution) + 1;
    num_samples = std::max(num_samples, 10); // 至少采样 10 个点

    for (int i = 0; i < num_samples; ++i) {
        // 计算当前采样时间
        double t = search_start + (search_end - search_start) * i / (num_samples - 1);

        // 获取轨迹上该时间点的位置
        Eigen::Vector2d traj_pos = traj.getPos(t);

        // 计算到机器人的距离平方
        double dist_sq = (traj_pos - robot_pos).squaredNorm();

        // 更新最小距离
        if (dist_sq < min_distance_sq) {
            min_distance_sq = dist_sq;
            best_time = t;
        }
    }

    // 4. 局部精细搜索（可选，提高精度）
    //    在粗搜索结果附近进行更精细的搜索
    double refine_range = planner_config_.replan_params.projection_search_resolution * 2.0;
    double refine_start = std::max(search_start, best_time - refine_range);
    double refine_end = std::min(search_end, best_time + refine_range);
    double refine_step = planner_config_.replan_params.projection_search_resolution * 0.1;

    for (double t = refine_start; t <= refine_end; t += refine_step) {
        Eigen::Vector2d traj_pos = traj.getPos(t);
        double dist_sq = (traj_pos - robot_pos).squaredNorm();

        if (dist_sq < min_distance_sq) {
            min_distance_sq = dist_sq;
            best_time = t;
        }
    }

    // 5. 填充结果
    result.valid = true;
    result.projection_time = best_time;
    result.distance = std::sqrt(min_distance_sq);
    result.position = traj.getPos(best_time);
    result.velocity = traj.getVel(best_time);
    result.acceleration = traj.getAcc(best_time); // S=3 热启动：获取加速度
    logger::info(logger::fsm_replan,
        "轨迹投影结果: t={:3f} s, dist={:3f} m, pos=({:2f}, {:2f}), "
        "vel=({:2f}, {:2f}), acc=({:2f}, {:2f})",
        result.projection_time,
        result.distance,
        result.position.x(),
        result.position.y(),
        result.velocity.x(),
        result.velocity.y(),
        result.acceleration.x(),
        result.acceleration.y()
    );

    return result;
}

auto FsmReplan::minco_optimize(
    path_planning::PathPostProcessing::Trajectory& traj,
    std::shared_ptr<grid_map::GridMap> grid_map,
    const utils::RobotState& current_pose
) -> tl::expected<Trajectory<5, 2>, MincoError> {
    auto start_time = std::chrono::steady_clock::now();
    utils::TimeConsuming timer_("MINCO", true);
    if (!grid_map) return tl::make_unexpected(MincoError::OPTFAIL);

    // ========== 初始状态 (Head State) ==========
    Eigen::Vector2d head_pos;
    Eigen::Vector2d head_vel;
    Eigen::Vector2d head_acc = Eigen::Vector2d::Zero();
    bool use_projection = false;

    if (has_valid_trajectory_) {
        auto now = std::chrono::system_clock::now();
        double traj_age = std::chrono::duration<double>(now - last_trajectory_time_).count();
        if (traj_age < planner_config_.replan_params.minco_traj_validity_duration) {
            const Eigen::Vector2d& robot_pos = current_pose.p.head<2>();
            TrajectoryProjectionResult proj =
                find_projection_on_trajectory(last_trajectory_, robot_pos, last_trajectory_time_, now);
            if (proj.valid && proj.distance < planner_config_.replan_params.minco_traj_continuity_threshold) {
                head_pos = proj.position;
                head_vel = proj.velocity;
                head_acc = proj.acceleration;
                use_projection = true;
            }
        }
    }

    if (!use_projection) {
        head_pos = traj.start_state_XYTheta.head<2>();
        head_vel = current_pose.v.head<2>();
        head_acc = Eigen::Vector2d::Zero();
    }

    // ========== 方案 B：optimized_path 关键点 + timed_trajectory 补点 ==========

    // 1. 关键点骨架：optimized_path 去重
    std::vector<Eigen::Vector2d> keyPoints;
    keyPoints.reserve(traj.optimized_path.size());
    for (const auto& p: traj.optimized_path) {
        if (keyPoints.empty() || (p - keyPoints.back()).squaredNorm() > 1e-12) {
            keyPoints.push_back(p);
        }
    }

    // 关键点太少时先补中点，保证至少 2 段
    while (keyPoints.size() == 2) {
        std::vector<Eigen::Vector2d> tmp;
        tmp.reserve(keyPoints.size() * 2 - 1);
        for (size_t i = 0; i + 1 < keyPoints.size(); ++i) {
            tmp.push_back(keyPoints[i]);
            tmp.push_back(0.5 * (keyPoints[i] + keyPoints[i + 1]));
        }
        tmp.push_back(keyPoints.back());
        keyPoints.swap(tmp);
    }

    if (keyPoints.size() < 2) {
        return tl::make_unexpected(MincoError::OPTFAIL);
    }

    const double traj_T = traj.total_time > 1e-6 ? traj.total_time : 0.5;

    // 2. 从 timed_trajectory 按时间插值取点
    auto sampleTimedPosition = [&](double t) -> Eigen::Vector2d {
        if (t <= 0.0) return traj.start_state_XYTheta.head<2>();
        if (t >= traj_T - 1e-9) return traj.final_state_XYTheta.head<2>();

        const auto& tp = traj.timed_trajectory;
        if (tp.empty()) return keyPoints.front();

        size_t idx = 0;
        while (idx < tp.size() && tp[idx].time < t)
            ++idx;

        if (idx == 0) {
            const Eigen::Vector2d p0 = traj.start_state_XYTheta.head<2>();
            const Eigen::Vector2d p1 = tp.front().state.head<2>();
            const double t0 = 0.0;
            const double t1 = tp.front().time;
            if (t1 - t0 < 1e-9) return p1;
            return p0 + ((t - t0) / (t1 - t0)) * (p1 - p0);
        }
        if (idx >= tp.size()) {
            const Eigen::Vector2d p0 = tp.back().state.head<2>();
            const Eigen::Vector2d p1 = traj.final_state_XYTheta.head<2>();
            const double t0 = tp.back().time;
            const double t1 = traj_T;
            if (t1 - t0 < 1e-9) return p1;
            return p0 + ((t - t0) / (t1 - t0)) * (p1 - p0);
        }

        const Eigen::Vector2d p0 = tp[idx - 1].state.head<2>();
        const Eigen::Vector2d p1 = tp[idx].state.head<2>();
        const double t0 = tp[idx - 1].time;
        const double t1 = tp[idx].time;
        if (t1 - t0 < 1e-9) return p1;
        return p0 + ((t - t0) / (t1 - t0)) * (p1 - p0);
    };

    // 3. 给关键点估计一个在 timed_trajectory 上的时间，用于排序和后续补点
    auto nearestTimedTime = [&](const Eigen::Vector2d& p) -> double {
        if (traj.timed_trajectory.empty()) return 0.0;
        double bestTime = 0.0;
        double bestDist = std::numeric_limits<double>::max();
        for (const auto& tp: traj.timed_trajectory) {
            const double d = (tp.state.head<2>() - p).squaredNorm();
            if (d < bestDist) {
                bestDist = d;
                bestTime = tp.time;
            }
        }
        if ((p - traj.start_state_XYTheta.head<2>()).squaredNorm() < 1e-6) return 0.0;
        if ((p - traj.final_state_XYTheta.head<2>()).squaredNorm() < 1e-6) return traj_T;
        return bestTime;
    };

    std::vector<Eigen::Vector2d> pts = keyPoints;

    // 首尾精确化
    if (!pts.empty()) {
        pts.front() = use_projection ? head_pos : traj.start_state_XYTheta.head<2>();
        pts.back() = traj.final_state_XYTheta.head<2>();
    }

    if (pts.size() < 3) {
        return tl::make_unexpected(MincoError::OPTFAIL);
    }

    const int N = static_cast<int>(pts.size()) - 1;
    const double total_T = std::max(traj.total_time, 0.5);
    Eigen::VectorXd initialTimes = Eigen::VectorXd::Constant(N, total_T / N);

    // ========== PVA 边界 ==========
    Eigen::Matrix<double, 2, 3> headPVA, tailPVA;
    tailPVA.col(0) = traj.final_state_XYTheta.head<2>();
    tailPVA.col(1) = Eigen::Vector2d::Zero();
    tailPVA.col(2) = Eigen::Vector2d::Zero();
    headPVA.col(0) = head_pos;
    headPVA.col(1) = head_vel;
    headPVA.col(2) = head_acc;

    // ========== 内部路点 ==========
    Eigen::Matrix2Xd innerPoints(2, N - 1);
    for (int i = 0; i < N - 1; ++i) {
        innerPoints.col(i) = pts[static_cast<size_t>(i + 1)];
    }

    // ========== 执行优化 ==========
    minco_.setESDFInterface(std::make_shared<minco_opt::GridMapESDF>(grid_map));
    minco_.setConfig(planner_config_.minco_opt_params);
    minco_.initialize(headPVA, tailPVA, N);

    auto elapsed = std::chrono::steady_clock::now() - start_time;
    const double max_time = 2.0;
    if (elapsed > std::chrono::duration<double>(max_time)) {
        return tl::make_unexpected(MincoError::TIMEDOUT);
    }

    if (!minco_.optimize(innerPoints, initialTimes)) {
        return tl::make_unexpected(MincoError::OPTFAIL);
    }

    // 获取优化后的轨迹
    Trajectory<5, 2> optimizedTraj;
    minco_.getTrajectory(optimizedTraj);

    // ========== 打印 MINCO 路径和速度统计 ==========
    {
        const double dt_log = 0.02;
        double max_spd = 0.0;
        double min_spd = std::numeric_limits<double>::max();
        double sum_spd = 0.0;
        int cnt = 0;
        std::string path_str;

        for (int i = 0; i < optimizedTraj.getPieceNum(); ++i) {
            const auto& piece = optimizedTraj[i];
            const double dur = piece.getDuration();

            for (double t = 0.0; t <= dur + 1e-9; t += dt_log) {
                const Eigen::Vector2d p = piece.getPos(t);
                const Eigen::Vector2d v = piece.getVel(t);
                const double spd = v.norm();

                max_spd = std::max(max_spd, spd);
                min_spd = std::min(min_spd, spd);
                sum_spd += spd;
                ++cnt;

                // 只打印前 30 个点，避免日志太长
                if (cnt <= 30 || cnt % 200 == 0) {
                    path_str += "(" + std::to_string(p.x()) + "," + std::to_string(p.y()) + ") ";
                }
            }
        }

        if (cnt > 0) {
            logger::info(logger::fsm_replan,"MINCO path: max_spd={:.3f}, min_spd={:.3f}, avg_spd={:.3f}", max_spd, min_spd, sum_spd / cnt);
        } else {
            logger::warn(logger::fsm_replan,"MINCO trajectory has no samples");
        }
    }
    last_trajectory_ = optimizedTraj;
    last_trajectory_time_ = std::chrono::system_clock::now();
    has_valid_trajectory_ = true;
    return optimizedTraj;
}
// auto FsmReplan::minco_optimize(
//     path_planning::PathPostProcessing::Trajectory& traj,
//     std::shared_ptr<grid_map::GridMap> grid_map,
//     const utils::RobotState& current_pose
// ) -> tl::expected<Trajectory<5, 2>, MincoError> {
//     auto start_time = std::chrono::steady_clock::now();
//     utils::TimeConsuming timer_("MINCO", true);
//     if (!grid_map) return tl::make_unexpected(MincoError::OPTFAIL);

//     // ========== 初始状态 (Head State) ==========
//     Eigen::Vector2d head_pos;
//     Eigen::Vector2d head_vel;
//     Eigen::Vector2d head_acc = Eigen::Vector2d::Zero();
//     bool use_projection = false;

//     if (has_valid_trajectory_) {
//         auto now = std::chrono::system_clock::now();
//         double traj_age = std::chrono::duration<double>(now - last_trajectory_time_).count();
//         if (traj_age < planner_config_.replan_params.minco_traj_validity_duration) {
//             const Eigen::Vector2d& robot_pos = current_pose.p.head<2>();
//             TrajectoryProjectionResult proj =
//                 find_projection_on_trajectory(last_trajectory_, robot_pos, last_trajectory_time_, now);
//             if (proj.valid && proj.distance < planner_config_.replan_params.minco_traj_continuity_threshold) {
//                 head_pos = proj.position;
//                 head_vel = proj.velocity;
//                 head_acc = proj.acceleration;
//                 use_projection = true;
//             }
//         }
//     }

//     if (!use_projection) {
//         head_pos = traj.start_state_XYTheta.head<2>();
//         head_vel = current_pose.v.head<2>();
//         head_acc = Eigen::Vector2d::Zero();
//     }

//     // ========== 方案 A：全部使用 timed_trajectory ==========
//     std::vector<Eigen::Vector2d> pts;
//     std::vector<double> timePoints;
//     pts.reserve(traj.timed_trajectory.size() + 2);
//     timePoints.reserve(traj.timed_trajectory.size() + 2);

//     pts.push_back(traj.start_state_XYTheta.head<2>());
//     timePoints.push_back(0.0);

//     for (const auto& tp : traj.timed_trajectory) {
//         Eigen::Vector2d p = tp.state.head<2>();
//         if ((p - pts.back()).squaredNorm() > 1e-12) {
//             pts.push_back(p);
//             timePoints.push_back(tp.time);
//         }
//     }

//     Eigen::Vector2d goal = traj.final_state_XYTheta.head<2>();
//     if ((goal - pts.back()).squaredNorm() > 1e-12) {
//         pts.push_back(goal);
//         timePoints.push_back(traj.total_time);
//     } else {
//         timePoints.back() = traj.total_time;
//     }

//     if (pts.size() < 3) {
//         return tl::make_unexpected(MincoError::OPTFAIL);
//     }

//     if (use_projection) {
//         pts.front() = head_pos;
//     }

//     int N = static_cast<int>(pts.size()) - 1;
//     Eigen::VectorXd initialTimes(N);
//     for (int i = 0; i < N; ++i) {
//         initialTimes(i) = std::max(
//             timePoints[static_cast<size_t>(i + 1)] - timePoints[static_cast<size_t>(i)],
//             1e-3
//         );
//     }

//     // ========== PVA 边界 ==========
//     Eigen::Matrix<double, 2, 3> headPVA, tailPVA;
//     tailPVA.col(0) = traj.final_state_XYTheta.head<2>();
//     tailPVA.col(1) = Eigen::Vector2d::Zero();
//     tailPVA.col(2) = Eigen::Vector2d::Zero();
//     headPVA.col(0) = head_pos;
//     headPVA.col(1) = head_vel;
//     headPVA.col(2) = head_acc;

//     // ========== 内部路点 ==========
//     Eigen::Matrix2Xd innerPoints(2, N - 1);
//     for (int i = 0; i < N - 1; ++i) {
//         innerPoints.col(i) = pts[static_cast<size_t>(i + 1)];
//     }

//     // ========== 执行优化 ==========
//     minco_.setESDFInterface(std::make_shared<minco_opt::GridMapESDF>(grid_map));
//     minco_.setConfig(planner_config_.minco_opt_params);
//     minco_.initialize(headPVA, tailPVA, N);

//     auto elapsed = std::chrono::steady_clock::now() - start_time;
//     const double max_time = 2.0;
//     if (elapsed > std::chrono::duration<double>(max_time)) {
//         return tl::make_unexpected(MincoError::TIMEDOUT);
//     }

//     if (!minco_.optimize(innerPoints, initialTimes)) {
//         return tl::make_unexpected(MincoError::OPTFAIL);
//     }

//     Trajectory<5, 2> optimizedTraj;
//     minco_.getTrajectory(optimizedTraj);
//     last_trajectory_ = optimizedTraj;
//     last_trajectory_time_ = std::chrono::system_clock::now();
//     has_valid_trajectory_ = true;
//     return optimizedTraj;
// }
// auto FsmReplan::minco_optimize(
//     path_planning::PathPostProcessing::Trajectory& traj,
//     std::shared_ptr<grid_map::GridMap> grid_map,
//     const utils::RobotState& current_pose
// ) -> tl::expected<Trajectory<5, 2>, MincoError> {
//     utils::TimeConsuming timer_("MINCO", true); // true 表示允许打印
//     if (!grid_map) return tl::make_unexpected(MincoError::OPTFAIL);

//     // 采样源:optimized_path。直线/近距离时可能被短路成 2~3 个点,
//     // 在相邻航点中点等距插值补足(所有点都在原折线上,形状不变;
//     // 用局部副本,不修改输出的 optimized_path)。
//     while (traj.optimized_path.size() == 2) {
//         std::vector<Eigen::Vector2d> tmp;
//         tmp.reserve(traj.optimized_path.size() * 2 - 1);
//         for (size_t i = 0; i + 1 < traj.optimized_path.size(); ++i) {
//             tmp.push_back(traj.optimized_path[i]);
//             tmp.push_back(0.5 * (traj.optimized_path[i] + traj.optimized_path[i + 1])); // 段中点
//         }
//         tmp.push_back(traj.optimized_path.back());
//         traj.optimized_path.swap(tmp);
//     }

//     // 段数 N:最多 6 段(决策维 = 2(N-1)+N),首尾航点固定
//     const int N = std::min(6, static_cast<int>(traj.optimized_path.size()) - 1);
//     if (N < 2) return tl::make_unexpected(MincoError::OPTFAIL);

//     // ① 子采样 N+1 个航点(首尾 = 起点/终点)
//     std::vector<Eigen::Vector2d> pts;
//     pts.reserve(static_cast<size_t>(N) + 1);
//     const int M = static_cast<int>(traj.optimized_path.size()) - 1;
//     for (int i = 0; i <= N; ++i) {
//         const int idx = static_cast<int>(std::lround(static_cast<double>(i) * M / N));
//         pts.push_back(traj.optimized_path[static_cast<size_t>(std::clamp(idx, 0, M))]);
//     }
//     // 首尾用精确 start/goal(代替栅格中心,避免与 PVA 边界产生小台阶)
//     pts.front() = traj.start_state_XYTheta.head<2>();
//     pts.back() = traj.final_state_XYTheta.head<2>();

//     // ② 初始路点(2 x (N-1))
//     Eigen::Matrix2Xd init_points(2, N - 1);
//     for (int i = 0; i < N - 1; ++i) {
//         init_points.col(i) = pts[static_cast<size_t>(i + 1)];
//     }

//     // ③ 初始时间:均匀分配 path_planning 的总时间(暂不考虑重规划热启动)
//     const double total_T = std::max(traj.total_time, 0.5);
//     Eigen::VectorXd init_times(N);
//     for (int i = 0; i < N; ++i)
//         init_times(i) = total_T / N;

//     // ④ PVA 边界(行 = x/y,列 = P/V/A):起点速度取当前速度,终点静止
//     Eigen::Matrix<double, 2, 3> head_pva, tail_pva;
//     head_pva << traj.start_state_XYTheta.x(), current_pose.v.x(), 0.0, traj.start_state_XYTheta.y(), current_pose.v.y(),
//         0.0;
//     tail_pva << traj.final_state_XYTheta.x(), 0.0, 0.0, traj.final_state_XYTheta.y(), 0.0, 0.0;

//     // ⑥ 执行优化(每次重建 ESDF 适配器;MincoOptimizer 非线程安全,单线程使用)
//     minco_.setESDFInterface(std::make_shared<minco_opt::GridMapESDF>(grid_map));
//     minco_.setConfig(planner_config_.minco_opt_params);
//     minco_.initialize(head_pva, tail_pva, N);
//     if (!minco_.optimize(init_points, init_times)) {
//         logger::fsm_replan->warn("MINCO optimize failed, keep raw trajectory");
//         return tl::make_unexpected(MincoError::OPTFAIL);
//     }

//     Trajectory<5, 2> spline;
//     minco_.getTrajectory(spline);

//     // ⑦ 密集采样安全检查:净空 / 速度 / 加速度,任一超限 -> 回退原始轨迹
//     const double hard_clearance = planner_config_.replan_params.hard_clearance;
//     const double vel_lim = planner_config_.minco_opt_params.max_vel * 1.05;
//     const double acc_lim = planner_config_.minco_opt_params.max_acc * 1.05;
//     const double dt_check = 0.02;
//     for (int i = 0; i < spline.getPieceNum(); ++i) {
//         const auto& piece = spline[i];
//         const double dur = piece.getDuration();
//         for (double t = 0.0; t <= dur; t += dt_check) {
//             const Eigen::Vector2d p = piece.getPos(t);
//             if (!grid_map->isInsideMap(p) || grid_map->getDistance(p) < hard_clearance) {
//                 logger::fsm_replan->warn(
//                     "MINCO traj unsafe (clearance {:.3f} < {:.3f}), keep raw trajectory",
//                     grid_map->getDistance(p),
//                     hard_clearance
//                 );
//                 return tl::make_unexpected(MincoError::OPTFAIL);
//                 ;
//             }
//             if (piece.getVel(t).norm() > vel_lim || piece.getAcc(t).norm() > acc_lim) {
//                 logger::fsm_replan->warn("MINCO traj unsafe (vel/acc), keep raw trajectory");
//                 return tl::make_unexpected(MincoError::OPTFAIL);
//                 ;
//             }
//         }
//     }

//     return spline;
// }
auto FsmReplan::sample_minco_trajectory(Trajectory<5, 2> spline, const double dt_check)
    -> std::vector<Eigen::Vector2d> {
    std::vector<Eigen::Vector2d> sample_trajectory;
    for (int i = 0; i < spline.getPieceNum(); ++i) {
        const auto& piece = spline[i];
        const double dur = piece.getDuration();
        for (double t = 0.0; t <= dur; t += dt_check) {
            sample_trajectory.emplace_back(piece.getPos(t));
        }
    }
    return sample_trajectory;
}

// sample_ma_spline_trajectory is intentionally not defined here:
// it would require a matching declaration in the read-only header.

}

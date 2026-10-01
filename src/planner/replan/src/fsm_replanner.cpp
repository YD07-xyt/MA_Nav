#include "replan/fsm_replanner.h"
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
    result_.is_new_trajectory = false;

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
        result_.is_new_trajectory = true; 
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

}

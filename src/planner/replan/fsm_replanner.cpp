#include "planner/replan/fsm_replanner.h"
namespace replan {

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

auto FsmReplan::check_collision(
    path_planning::PathPostProcessing::Trajectory traj,
    const grid_map::GridMap& grid_map,
    const double& safe_threshold
) -> bool {
    // 同时检查 planning原始网格路径与优化后航点路径：
    // 原始路径逐格密集，可发现两个优化航点之间被新障碍挡住的情况
    std::vector<std::vector<Eigen::Vector2d>> paths;
    paths.push_back(traj.optimized_path);
    //paths.push_back(traj.raw_path);
    for (const auto& path: paths) {
        // Points outside map are considered collision-free
        for (auto pos: path) {
            if (!grid_map.isInsideMap(pos)) {
                continue;
            }

            // Points inside map use safety distance check
            if (grid_map.getDistance(pos) < safe_threshold) {
                return true;
            };
        }
    }
    return false;
};
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
auto FsmReplan::minco_optimize(
    path_planning::PathPostProcessing::Trajectory& traj,
    std::shared_ptr<grid_map::GridMap> grid_map,
    const utils::RobotState& current_pose
) -> tl::expected<Trajectory<5, 2>, MincoError> {
    utils::TimeConsuming timer_("MINCO", true); // true 表示允许打印
    if (!grid_map) return tl::make_unexpected(MincoError::OPTFAIL);

    // 采样源:optimized_path。直线/近距离时可能被短路成 2~3 个点,
    // 在相邻航点中点等距插值补足(所有点都在原折线上,形状不变;
    // 用局部副本,不修改输出的 optimized_path)。
    while (traj.optimized_path.size() == 2) {
        std::vector<Eigen::Vector2d> tmp;
        tmp.reserve(traj.optimized_path.size() * 2 - 1);
        for (size_t i = 0; i + 1 < traj.optimized_path.size(); ++i) {
            tmp.push_back(traj.optimized_path[i]);
            tmp.push_back(0.5 * (traj.optimized_path[i] + traj.optimized_path[i + 1])); // 段中点
        }
        tmp.push_back(traj.optimized_path.back());
        traj.optimized_path.swap(tmp);
    }
    //if (traj.optimized_path.size() < 4) return tl::make_unexpected(MincoError::OPTFAIL);

    // 段数 N:最多 6 段(决策维 = 2(N-1)+N),首尾航点固定
    const int N = std::min(6, static_cast<int>(traj.optimized_path.size()) - 1);
    if (N < 2) return tl::make_unexpected(MincoError::OPTFAIL);

    // ① 子采样 N+1 个航点(首尾 = 起点/终点)
    std::vector<Eigen::Vector2d> pts;
    pts.reserve(static_cast<size_t>(N) + 1);
    const int M = static_cast<int>(traj.optimized_path.size()) - 1;
    for (int i = 0; i <= N; ++i) {
        const int idx = static_cast<int>(std::lround(static_cast<double>(i) * M / N));
        pts.push_back(traj.optimized_path[static_cast<size_t>(std::clamp(idx, 0, M))]);
    }
    // 首尾用精确 start/goal(代替栅格中心,避免与 PVA 边界产生小台阶)
    pts.front() = traj.start_state_XYTheta.head<2>();
    pts.back() = traj.final_state_XYTheta.head<2>();

    // ② 初始路点(2 x (N-1))
    Eigen::Matrix2Xd init_points(2, N - 1);
    for (int i = 0; i < N - 1; ++i) {
        init_points.col(i) = pts[static_cast<size_t>(i + 1)];
    }

    // ③ 初始时间:均匀分配 path_planning 的总时间(暂不考虑重规划热启动)
    const double total_T = std::max(traj.total_time, 0.5);
    Eigen::VectorXd init_times(N);
    for (int i = 0; i < N; ++i)
        init_times(i) = total_T / N;

    // ④ PVA 边界(行 = x/y,列 = P/V/A):起点速度取当前速度,终点静止
    Eigen::Matrix<double, 2, 3> head_pva, tail_pva;
    head_pva << traj.start_state_XYTheta.x(), current_pose.v.x(), 0.0, traj.start_state_XYTheta.y(), current_pose.v.y(),
        0.0;
    tail_pva << traj.final_state_XYTheta.x(), 0.0, 0.0, traj.final_state_XYTheta.y(), 0.0, 0.0;

    // ⑥ 执行优化(每次重建 ESDF 适配器;MincoOptimizer 非线程安全,单线程使用)
    minco_.setESDFInterface(std::make_shared<minco_opt::GridMapESDF>(grid_map));
    minco_.setConfig(planner_config_.minco_opt_params);
    minco_.initialize(head_pva, tail_pva, N);
    if (!minco_.optimize(init_points, init_times)) {
        logger::fsm_replan->warn("MINCO optimize failed, keep raw trajectory");
        return tl::make_unexpected(MincoError::OPTFAIL);
    }

    Trajectory<5, 2> spline;
    minco_.getTrajectory(spline);

    // ⑦ 密集采样安全检查:净空 / 速度 / 加速度,任一超限 -> 回退原始轨迹
    const double hard_clearance = planner_config_.replan_params.hard_clearance;
    const double vel_lim = planner_config_.minco_opt_params.max_vel * 1.05;
    const double acc_lim = planner_config_.minco_opt_params.max_acc * 1.05;
    const double dt_check = 0.02;
    for (int i = 0; i < spline.getPieceNum(); ++i) {
        const auto& piece = spline[i];
        const double dur = piece.getDuration();
        for (double t = 0.0; t <= dur; t += dt_check) {
            const Eigen::Vector2d p = piece.getPos(t);
            if (!grid_map->isInsideMap(p) || grid_map->getDistance(p) < hard_clearance) {
                logger::fsm_replan->warn(
                    "MINCO traj unsafe (clearance {:.3f} < {:.3f}), keep raw trajectory",
                    grid_map->getDistance(p),
                    hard_clearance
                );
                return tl::make_unexpected(MincoError::OPTFAIL);
                ;
            }
            if (piece.getVel(t).norm() > vel_lim || piece.getAcc(t).norm() > acc_lim) {
                logger::fsm_replan->warn("MINCO traj unsafe (vel/acc), keep raw trajectory");
                return tl::make_unexpected(MincoError::OPTFAIL);
                ;
            }
        }
    }

    return spline;
}
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
}
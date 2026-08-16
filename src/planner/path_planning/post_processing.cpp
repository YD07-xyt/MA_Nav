#include "planner/path_planning/post_processing.h"

namespace path_planning {
auto PathPostProcessing::optimize_path(const std::vector<Eigen::Vector2d>& path) -> std::vector<Eigen::Vector2d> {
    if (path.size() < 2) {
        return path;
    }
    std::vector<Eigen::Vector2d> optimized_path;
    optimized_path.push_back(path[0]);
    Eigen::Vector2d prev_pose = path[0];
    double cost1, cost2, cost3;

    // Check first path segment
    if (!check_line_collision(path[0], path[1])) {
        cost1 = (path[0] - path[1]).norm();
    } else {
        cost1 = std::numeric_limits<double>::infinity();
    }
    for (size_t i = 1; i < path.size() - 1; ++i) {
        const auto& pose1 = path[i];
        const auto& pose2 = path[i + 1];

        // Calculate current segment cost
        if (!check_line_collision(pose1, pose2)) {
            cost2 = (pose1 - pose2).norm();
        } else {
            cost2 = std::numeric_limits<double>::infinity();
        }
        // Calculate skip current point cost
        if (!check_line_collision(prev_pose, pose2)) {
            cost3 = (prev_pose - pose2).norm();
        } else {
            cost3 = std::numeric_limits<double>::infinity();
        }
        // Decide whether to skip current point
        if (cost3 < cost1 + cost2) {
            cost1 = cost3;
        } else {
            optimized_path.push_back(path[i]);
            cost1 = (pose1 - pose2).norm();
            prev_pose = pose1;
        }
    }

    optimized_path.push_back(path.back());
    return optimized_path;
}
auto PathPostProcessing::check_collision(const Eigen::Vector2d& pos) const -> bool {
    // Points outside map are considered collision-free
    if (!map_.isInsideMap(pos)) return false;

    // Points inside map use safety distance check
    return map_.getDistance(pos) < params_.safe_threshold;
}

/*  Bresenham 直线算法 枚举线段经过的所有栅格索引，
    并逐一调用 check_collision，判断两点间的直线段是否穿过障碍物，
    用于后续路径优化时的跳过可行性判断*/
auto PathPostProcessing::check_line_collision(const Eigen::Vector2d& start, const Eigen::Vector2d& end) const -> bool {
    Eigen::Vector2i start_idx, end_idx;
    map_.posToIndex(start, start_idx);
    map_.posToIndex(end, end_idx);

    int dx = abs(end_idx.x() - start_idx.x());
    int dy = abs(end_idx.y() - start_idx.y());
    int sx = start_idx.x() < end_idx.x() ? 1 : -1;
    int sy = start_idx.y() < end_idx.y() ? 1 : -1;
    int err = dx - dy;

    while (true) {
        Eigen::Vector2d pos;
        map_.indexToPos(start_idx, pos);
        if (check_collision(pos)) return true;
        if (start_idx == end_idx) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            start_idx.x() += sx;
        }
        if (e2 < dx) {
            err += dx;
            start_idx.y() += sy;
        }
    }
    return false;
}

auto PathPostProcessing::sample_path_states(const std::vector<Eigen::Vector2d>& path) -> std::vector<PathState> {
    std::vector<PathState> states;
    if (path.empty()) return states;

    // Initial state
    states.push_back({path[0], 0, 0, 0});

    for (size_t i = 1; i < path.size(); ++i) {
        const auto& prev = states.back();
        const auto& curr_pos = path[i];

        // Calculate orientation angle
        double theta = atan2(curr_pos.y() - prev.position.y(), curr_pos.x() - prev.position.x());

        // Normalize angle
        normalize_angle(prev.theta, theta);

        // Calculate segment length
        double delta_s = (curr_pos - prev.position).norm();

        states.push_back({curr_pos, theta, theta - prev.theta, delta_s});
    }
    return states;
}

auto PathPostProcessing::normalize_angle(double ref_angle, double& angle) const -> void {
    while (ref_angle - angle > M_PI)
        angle += 2 * M_PI;
    while (ref_angle - angle < -M_PI)
        angle -= 2 * M_PI;
}

auto PathPostProcessing::assign_trajectory_timing(Trajectory& traj) -> void {
    if (traj.path_states.empty()) return;

    // Calculate weighted path length (considering angle changes)
    traj.total_length = 0;
    traj.weighted_length = 0;
    for (const auto& state: traj.path_states) {
        traj.total_length += state.delta_s;
        traj.weighted_length += state.delta_s * params_.distance_weight + fabs(state.delta_theta) * params_.yaw_weight;
    }

    // Calculate total time (trapezoidal velocity profile, uses actual start/end vel)
    traj.total_time =
        evaluate_duration(traj.weighted_length, params_.start_vel, params_.end_vel, params_.max_vel, params_.max_acc);

    // Sample time points
    int num_segments = std::max(
        static_cast<int>(traj.total_time / params_.time_resolution + 0.5), 
        params_.min_traj_num
    );
    double sample_time = traj.total_time / num_segments; // Δt

    // 直接生成均匀的段间时间向量
    traj.time_segments = Eigen::VectorXd::Constant(num_segments, sample_time);

    // Generate timed trajectory points
    double accumulated_s = 0;
    size_t state_idx = 0;

    for (double t = sample_time; t < traj.total_time - 1e-3; t += sample_time) {
        double s = evaluate_length(
            t,
            traj.weighted_length,
            traj.total_time,
            params_.start_vel,
            params_.end_vel,
            params_.max_vel,
            params_.max_acc
        );

        // Find corresponding state
        while (state_idx < traj.path_states.size() - 1 && accumulated_s + traj.path_states[state_idx].delta_s < s) {
            accumulated_s += traj.path_states[state_idx].delta_s;
            state_idx++;
        }

        if (state_idx > 0) {
            double ratio = (s - accumulated_s) / traj.path_states[state_idx].delta_s;

            Eigen::Vector2d pos = traj.path_states[state_idx - 1].position
                + ratio * (traj.path_states[state_idx].position - traj.path_states[state_idx - 1].position);
            double theta = traj.path_states[state_idx - 1].theta + ratio * (traj.path_states[state_idx].delta_theta);

            traj.timed_trajectory.push_back({Eigen::Vector3d(pos.x(), pos.y(), theta), t});
        }
    }
}

auto PathPostProcessing::fill_additional_trajectory_info(
    Trajectory& traj,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal
) -> void {
    // Set start/goal states
    traj.start_state_XYTheta << start.x(), start.y(), 0;
    traj.final_state_XYTheta << goal.x(), goal.y(), traj.path_states.back().theta;

    // Generate unoccupied positions sequence
    for (const auto& state: traj.path_states) {
        traj.UnOccupied_positions.emplace_back(state.position.x(), state.position.y(), state.theta);
    }

    // Set initial time
    traj.UnOccupied_initT = traj.total_time / traj.timed_trajectory.size();

    // Check if needs truncation
    traj.if_cut = (traj.total_length > params_.traj_cut_length);

    // Set default velocity/acceleration (can be adjusted as needed)
    traj.start_state << 0, 0, 0; // Initial velocity 0
    traj.final_state << 0, 0, 0; // Final velocity 0
}

auto PathPostProcessing::evaluate_duration(
    double length,
    double start_vel,
    double end_vel,
    double max_vel,
    double max_acc
) const -> double {
    double critical_len = (max_vel * max_vel - start_vel * start_vel) / (2 * max_acc)
        + (max_vel * max_vel - end_vel * end_vel) / (2 * max_acc);

    if (length >= critical_len) {
        return (max_vel - start_vel) / max_acc + (max_vel - end_vel) / max_acc + (length - critical_len) / max_vel;
    } else {
        double tmp_vel = sqrt(0.5 * (start_vel * start_vel + end_vel * end_vel + 2 * max_acc * length));
        return (tmp_vel - start_vel) / max_acc + (tmp_vel - end_vel) / max_acc;
    }
}

auto PathPostProcessing::evaluate_length(
    double t,
    double total_length,
    double total_time,
    double start_vel,
    double end_vel,
    double max_vel,
    double max_acc
) const -> double {
    double critical_len = (max_vel * max_vel - start_vel * start_vel) / (2 * max_acc)
        + (max_vel * max_vel - end_vel * end_vel) / (2 * max_acc);

    if (total_length >= critical_len) {
        double t1 = (max_vel - start_vel) / max_acc;
        double t2 = t1 + (total_length - critical_len) / max_vel;

        if (t <= t1) {
            return start_vel * t + 0.5 * max_acc * t * t;
        } else if (t <= t2) {
            return start_vel * t1 + 0.5 * max_acc * t1 * t1 + (t - t1) * max_vel;
        } else {
            return start_vel * t1 + 0.5 * max_acc * t1 * t1 + (t2 - t1) * max_vel + max_vel * (t - t2)
                - 0.5 * max_acc * (t - t2) * (t - t2);
        }
    } else {
        double tmp_vel = sqrt(0.5 * (start_vel * start_vel + end_vel * end_vel + 2 * max_acc * total_length));
        double tmp_t = (tmp_vel - start_vel) / max_acc;

        if (t <= tmp_t) {
            return start_vel * t + 0.5 * max_acc * t * t;
        } else {
            return start_vel * tmp_t + 0.5 * max_acc * tmp_t * tmp_t + tmp_vel * (t - tmp_t)
                - 0.5 * max_acc * (t - tmp_t) * (t - tmp_t);
        }
    }
}
}
#pragma once
#include "astar.h"
#include "post_processing.h"
#include <optional>
namespace path_planning {
class PathPlanning {
public:
    auto set_map(const grid_map::GridMap& grid_map) -> void {
        astar_.set_map(grid_map);
        post_processing_.set_map(grid_map);
    };
    auto set_param(const PathPostProcessing::PathPostProcessingParams& params) -> void {
        astar_.set_safe_threshold(params.safe_threshold);
        post_processing_.set_params(params);
    }
    auto path_planning(const Eigen::Vector2d& start, const Eigen::Vector2d& goal, int timeout_ms)
        -> std::optional<PathPostProcessing::Trajectory> {
        utils::TimeConsuming timer("Astar", true); // true 表示允许打印

        PathPostProcessing::Trajectory traj;

        // 1. Original A* search
        traj.raw_path = astar_.original_astar_search(start, goal, timeout_ms);

        if (traj.raw_path.empty()) {
            return std::nullopt;
        }
        // 2. Path optimization (remove redundant waypoints)
        traj.optimized_path = post_processing_.optimize_path(traj.raw_path);

        // 3. State sampling (generate 5D states)
        traj.path_states = post_processing_.sample_path_states(traj.optimized_path);

        // 4. Time allocation (trapezoidal velocity profile)
        post_processing_.assign_trajectory_timing(traj);

        // 5. Fill additional information
        post_processing_.fill_additional_trajectory_info(traj, start, goal);
        if (traj.timed_trajectory.empty() || traj.total_time <= 1e-6) {
            logger::planning->warn("生成的轨迹没有有效时间，已丢弃");
            return std::nullopt;
        }
        return traj;
    };

private:
    AStar astar_;
    PathPostProcessing post_processing_;
};
}
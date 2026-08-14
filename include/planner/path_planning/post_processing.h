#pragma once
#include "map/grid_map.hpp"
#include <Eigen/Core>
namespace path_planning {
class PathPostProcessing {
private:
    // 5D state (x,y,theta,dtheta,ds)
    struct PathState {
        Eigen::Vector2d position;
        double theta = 0; // Orientation angle
        double delta_theta = 0; // Angle change
        double delta_s = 0; // Path segment length
    };

    // Timed trajectory point
    struct TimedTrajectoryPoint {
        Eigen::Vector3d state; // x, y, theta
        double time = 0;
    };
public:
    // Trajectory data structure
    struct Trajectory {
        // Path data
        std::vector<Eigen::Vector2d> raw_path; // Original path
        std::vector<Eigen::Vector2d> optimized_path; // Optimized path
        std::vector<PathState> path_states; // Path states
        std::vector<TimedTrajectoryPoint> timed_trajectory; // Timed trajectory

        // State information
        Eigen::Vector3d start_state = Eigen::Vector3d::Zero(); // Start state (x_vel, y_vel, omega)
        Eigen::Vector3d final_state = Eigen::Vector3d::Zero(); // Final state
        Eigen::Vector3d start_state_XYTheta = Eigen::Vector3d::Zero(); // Start pose (x,y,theta)
        Eigen::Vector3d final_state_XYTheta = Eigen::Vector3d::Zero(); // Final pose

        // Path characteristics
        double total_time = 0; // Total time
        double total_length = 0; // Total length
        double weighted_length = 0; // Weighted length (considering angle changes)

        // Additional information
        bool if_cut = false; // Whether trajectory is truncated
        std::vector<Eigen::Vector3d> UnOccupied_positions; // Unoccupied positions sequence
        double UnOccupied_initT = 0.0; // Initial time
    };
public:
    struct PathPostProcessingParams {
        double safe_threshold;
        // Trajectory parameters
        double max_vel;
        double max_acc;
        double time_resolution;
        int min_traj_num;
        double traj_cut_length;
        double distance_weight;
        double yaw_weight;
        double start_vel {0.0};
        double end_vel {0.0};
    };
private:
    PathPostProcessingParams params_;
    grid_map::GridMap map_;
public: 
    auto set_map(const grid_map::GridMap& map) -> void {
        map_ = map;
    };
    auto set_params(const PathPostProcessingParams& params)->void{
        params_=params;
    }

public:
    auto check_collision(const Eigen::Vector2d& pos) const -> bool;
    auto optimize_path(const std::vector<Eigen::Vector2d>& path) -> std::vector<Eigen::Vector2d>;

    auto check_line_collision(const Eigen::Vector2d& start, const Eigen::Vector2d& end) const -> bool;

    auto sample_path_states(const std::vector<Eigen::Vector2d>& path) -> std::vector<PathState>;

    auto normalize_angle(double ref_angle, double& angle) const -> void;
    auto assign_trajectory_timing(Trajectory& traj) -> void;

    auto fill_additional_trajectory_info(Trajectory& traj, const Eigen::Vector2d& start, const Eigen::Vector2d& goal)
        -> void;

    auto evaluate_duration(double length, double start_vel, double end_vel, double max_vel, double max_acc) const
        -> double;

    auto evaluate_length(
        double t,
        double total_length,
        double total_time,
        double start_vel,
        double end_vel,
        double max_vel,
        double max_acc
    ) const -> double;
};
}
#pragma once

#include "planner/path_planning/post_processing.h"
#include "planner/traj_optimize/ma_spline_opt/SplineTrajectory/SplineTrajectory.hpp"
#include "utils/logger.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <vector>

namespace ma_spline_opt {

struct MaSplineOptimizerConfig
{
    enum class Mode
    {
        OMNI_XY,
        OMNI_XY_YAW,
        OMNI_XY_YAW_JOINT
    };

    Mode mode = Mode::OMNI_XY;

    // 平滑
    double rho_energy = 5.0;

    // 时间
    double weight_time = 20.0;
    double weight_mean_time = 40.0;
    double mean_lower = 0.9;
    double mean_upper = 1.1;
    double min_time = 0.1;
    double weight_min_time = 100.0;

    // ESDF
    double weight_obstacle = 8000.0;
    double safe_distance = 0.3;

    // 动力学
    double weight_vel = 40.0;
    double weight_acc = 40.0;
    double v_max = 3.0;
    double a_max = 5.0;

    // yaw 动力学
    double weight_yaw_vel = 10.0;
    double weight_yaw_acc = 5.0;
    double yaw_rate_max = 1.0;
    double yaw_acc_max = 2.0;

    // 积分采样
    int integral_num_steps = 8;

    // 重采样段数，参考 minco 的 pieceNum
    int num_segments = 8;

    // L-BFGS
    int max_iterations = 2000;
    double g_epsilon = 1e-4;
    int lbfgs_mem_size = 64;
    double lbfgs_delta = 5e-3;
};

struct TimedReferencePoint
{
    double t = 0.0;
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();
    double yaw = 0.0;
};

struct MaSplineInput
{
    std::vector<TimedReferencePoint> timed_trajectory;

    Eigen::Vector2d start_vel = Eigen::Vector2d::Zero();
    Eigen::Vector2d start_acc = Eigen::Vector2d::Zero();
    Eigen::Vector2d end_vel = Eigen::Vector2d::Zero();
    Eigen::Vector2d end_acc = Eigen::Vector2d::Zero();

    double start_yaw_rate = 0.0;
    double start_yaw_acc = 0.0;
    double end_yaw_rate = 0.0;
    double end_yaw_acc = 0.0;
};

inline auto from_path_planning_trajectory(
    const path_planning::PathPostProcessing::Trajectory& traj)
    -> MaSplineInput
{
    MaSplineInput input;

    std::vector<TimedReferencePoint> pts;
    pts.reserve(traj.timed_trajectory.size() + 2);

    // 起点
    if (traj.timed_trajectory.empty() ||
        traj.timed_trajectory.front().time > 1e-6)
    {
        pts.push_back({
            0.0,
            traj.start_state_XYTheta.head<2>(),
            traj.start_state_XYTheta.z()
        });
    }

    // 中间点
    for (const auto& p : traj.timed_trajectory)
    {
        pts.push_back({
            p.time,
            p.state.head<2>(),
            p.state.z()
        });
    }

    // 终点
    if (traj.timed_trajectory.empty() ||
        traj.timed_trajectory.back().time < traj.total_time - 1e-6)
    {
        pts.push_back({
            traj.total_time,
            traj.final_state_XYTheta.head<2>(),
            traj.final_state_XYTheta.z()
        });
    }

    // 排序
    std::sort(pts.begin(), pts.end(),
              [](const TimedReferencePoint& a,
                 const TimedReferencePoint& b)
              {
                  return a.t < b.t;
              });

    // 去重
    input.timed_trajectory.clear();
    for (const auto& p : pts)
    {
        if (input.timed_trajectory.empty() ||
            p.t - input.timed_trajectory.back().t > 1e-6)
        {
            input.timed_trajectory.push_back(p);
        }
    }

    input.start_vel = traj.start_state.head<2>();
    input.end_vel = traj.final_state.head<2>();
    input.start_acc = Eigen::Vector2d::Zero();
    input.end_acc = Eigen::Vector2d::Zero();

    input.start_yaw_rate = traj.start_state.z();
    input.end_yaw_rate = traj.final_state.z();
    input.start_yaw_acc = 0.0;
    input.end_yaw_acc = 0.0;

    return input;
}

struct MAsplineOutput
{
    std::vector<double> time_segments;
    double start_time = 0.0;

    SplineTrajectory::QuinticSplineND<2> xy_spline;
    SplineTrajectory::QuinticSplineND<1> yaw_spline;

    double cost = 0.0;
    bool success = false;
};

} // namespace ma_spline_opt
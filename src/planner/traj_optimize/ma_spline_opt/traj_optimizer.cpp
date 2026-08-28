#include "planner/traj_optimize/ma_spline_opt/traj_optimizer.h"
namespace ma_spline_opt {
bool MaSplineTrajectoryOptimizer::check_trajectory_collision(
    const MAsplineOutput& out,
    const ESDFInterface* esdf,
    double safe_distance
) {
    if (!out.success || !out.xy_spline.isInitialized()) return false;

    const auto& traj = out.xy_spline.getTrajectory();

    for (double t = traj.getStartTime(); t <= traj.getEndTime() + 1e-6; t += 0.05) {
        Eigen::Vector2d p = traj.evaluate(t, 0);

        if (esdf && esdf->isInside(p.x(), p.y())) {
            if (esdf->getDistance(p.x(), p.y()) < safe_distance) return false;
        }
    }

    return true;
}

auto MaSplineTrajectoryOptimizer::optimize(const MaSplineInput& input) -> MAsplineOutput {
    MAsplineOutput out = optimize_xy(input);

    // 碰障时降时间权重，重新优化一次
    if (out.success && esdf_ && !check_trajectory_collision(out, esdf_, config_.safe_distance)) {
        auto relaxed = config_;

        // 碰撞重试时，主要放宽 Stage2 的严格参数
        relaxed.stage2.weight_time *= 0.6;
        relaxed.stage2.weight_obstacle *= 0.4;

        // 如果也希望 Stage1 更保守，可以同步改：
        // relaxed.stage1.weight_time *= 0.6;
        // relaxed.stage1.weight_obstacle *= 1.5;

        auto old = config_;
        config_ = relaxed;
        out = optimize_xy(input);
        config_ = old;
    }

    return out;
}

// ========================================================================
// 2D xy 轨迹优化
// 直接使用 timed_trajectory，不降采样
// 点密度由上游 time_resolution 控制
// ========================================================================
auto MaSplineTrajectoryOptimizer::optimize_xy(const MaSplineInput& input) -> MAsplineOutput {
    MAsplineOutput out;

    const auto& timed = input.timed_trajectory;
    if (timed.size() < 2) return out;

    // 构造 problem
    std::vector<double> time_points;
    Eigen::Matrix<double, Eigen::Dynamic, 2> waypoints(timed.size(), 2);
    for (size_t i = 0; i < timed.size(); ++i) {
        time_points.push_back(timed[i].t);
        waypoints.row(i) = timed[i].pos.transpose();
    }

    SplineTrajectory::BoundaryConditions<2> bc;
    bc.start_velocity = input.start_vel;
    bc.end_velocity = input.end_vel;
    bc.start_acceleration = input.start_acc;
    bc.end_acceleration = input.end_acc;

    const int N = static_cast<int>(timed.size()) - 1;

    SplineTrajectory::OptimizationMask mask;
    mask.time.assign(N, 1);
    mask.waypoints.assign(N + 1, 1);
    mask.waypoints.front() = 0;
    mask.waypoints.back() = 0;

    auto problem = Opt2D::makeProblemFromTimePoints(time_points, waypoints, bc, mask);

    // ============================================================
    // Stage 1：使用 config_.stage1
    // ============================================================
    const auto& s1 = config_.stage1;

    Opt2D::OptimizerConfig opt_cfg1;
    opt_cfg1.rho_energy = s1.rho_energy;
    opt_cfg1.integral_num_steps = config_.integral_num_steps;

    optimizer_.setConfig(opt_cfg1);
    auto status1 = optimizer_.prepareContext(problem, ctx_);
    if (!status1) return out;

    TimeCost time_cost1;
    time_cost1.w_time = s1.weight_time;
    time_cost1.w_mean = s1.weight_mean_time;
    time_cost1.mean_lower = s1.mean_lower;
    time_cost1.mean_upper = s1.mean_upper;
    time_cost1.w_min_time = s1.weight_min_time;
    time_cost1.min_time = s1.min_time;

    RobotIntegralCost<2> integral_cost1;
    integral_cost1.esdf = esdf_;
    integral_cost1.w_obs = s1.weight_obstacle;
    integral_cost1.safe_distance = config_.safe_distance;
    integral_cost1.w_vel = s1.weight_vel;
    integral_cost1.w_acc = s1.weight_acc;
    integral_cost1.v_max = config_.v_max;
    integral_cost1.a_max = config_.a_max;

    auto spec1 = Opt2D::makeEvaluateSpec(time_cost1, integral_cost1);
    CallbackCtx cb1 {&optimizer_, &ctx_, &spec1};

    Eigen::VectorXd x = optimizer_.generateInitialGuess(ctx_);
    double cost1 = 0.0;

    lbfgs::lbfgs_parameter_t params1;
    params1.max_iterations = s1.max_iterations;
    params1.g_epsilon = s1.g_epsilon;
    params1.mem_size = s1.lbfgs_mem_size;
    params1.past = 3;
    params1.delta = s1.lbfgs_delta;
    params1.min_step = 1e-32;

    lbfgs::lbfgs_optimize(x, cost1, &MaSplineTrajectoryOptimizer::cost_callback, nullptr, nullptr, &cb1, params1);

    // ============================================================
    // Stage 2：使用 config_.stage2，用 Stage1 的 x 作为初值
    // ============================================================
    const auto& s2 = config_.stage2;

    Opt2D::OptimizerConfig opt_cfg2;
    opt_cfg2.rho_energy = s2.rho_energy;
    opt_cfg2.integral_num_steps = config_.integral_num_steps;

    optimizer_.setConfig(opt_cfg2);
    auto status2 = optimizer_.prepareContext(problem, ctx_);
    if (!status2) return out;

    TimeCost time_cost2;
    time_cost2.w_time = s2.weight_time;
    time_cost2.w_mean = s2.weight_mean_time;
    time_cost2.mean_lower = s2.mean_lower;
    time_cost2.mean_upper = s2.mean_upper;
    time_cost2.w_min_time = s2.weight_min_time;
    time_cost2.min_time = s2.min_time;

    RobotIntegralCost<2> integral_cost2;
    integral_cost2.esdf = esdf_;
    integral_cost2.w_obs = s2.weight_obstacle;
    integral_cost2.safe_distance = config_.safe_distance;
    integral_cost2.w_vel = s2.weight_vel;
    integral_cost2.w_acc = s2.weight_acc;
    integral_cost2.v_max = config_.v_max;
    integral_cost2.a_max = config_.a_max;

    auto spec2 = Opt2D::makeEvaluateSpec(time_cost2, integral_cost2);
    CallbackCtx cb2 {&optimizer_, &ctx_, &spec2};

    double cost2 = 0.0;

    lbfgs::lbfgs_parameter_t params2;
    params2.max_iterations = s2.max_iterations;
    params2.g_epsilon = s2.g_epsilon;
    params2.mem_size = s2.lbfgs_mem_size;
    params2.past = 3;
    params2.delta = s2.lbfgs_delta;
    params2.min_step = 1e-32;

    lbfgs::lbfgs_optimize(x, cost2, &MaSplineTrajectoryOptimizer::cost_callback, nullptr, nullptr, &cb2, params2);

    // 同步最终结果
    optimizer_.synchronizeWorkingState(ctx_, x);

    out.xy_spline = optimizer_.getWorkingSpline(ctx_);
    out.time_segments = ctx_.runtime.state.times;
    out.start_time = ctx_.runtime.state.start_time;
    out.cost = cost2;
    out.success = true;

    return out;
}

auto MaSplineTrajectoryOptimizer::filter_timed_trajectory(
    const std::vector<TimedReferencePoint>& input,
    double min_spacing
) -> std::vector<TimedReferencePoint> {
    if (input.size() < 2) return input;

    std::vector<TimedReferencePoint> result;
    result.reserve(input.size());

    // 起点必须保留
    result.push_back(input.front());

    for (size_t i = 1; i + 1 < input.size(); ++i) {
        const double dist = (input[i].pos - result.back().pos).norm();

        // 距离太近就跳过，避免控制点堆积
        if (dist >= min_spacing) {
            result.push_back(input[i]);
        }
    }

    // 终点必须保留
    if ((input.back().pos - result.back().pos).norm() > 1e-6) {
        result.push_back(input.back());
    }

    return result;
}

}
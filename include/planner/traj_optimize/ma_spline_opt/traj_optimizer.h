#pragma once

#include "SplineTrajectory/SplineOptimizer.hpp"
#include "optimizer_config.h"
#include "cost.hpp"
#include "planner/traj_optimize/esdf_Interface.hpp"
#include "utils/lbfgs.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ma_spline_opt {

// ============================================================================
// yaw 单独优化时的积分代价
// 只约束 yaw rate / yaw acc
// ============================================================================
struct YawIntegralCost {
    double w_yaw_vel = 10.0;
    double w_yaw_acc = 5.0;
    double yaw_rate_max = 1.0;
    double yaw_acc_max = 2.0;

    static inline void smoothedL1(double x, double& f, double& df) {
        constexpr double pe = 1e-2;
        const double f3c = 1.0 / (pe * pe);
        const double f4c = -0.5 * f3c / pe;

        if (x <= 0.0) {
            f = 0.0;
            df = 0.0;
        } else if (x < pe) {
            f = (f4c * x + f3c) * x * x * x;
            df = (4.0 * f4c * x + 3.0 * f3c) * x * x;
        } else {
            f = x - 0.5 * pe;
            df = 1.0;
        }
    }

    double operator()(
        const SplineTrajectory::IntegralPointInfo& /*point*/,
        const Eigen::Matrix<double, 1, 1>& /*p*/,
        const Eigen::Matrix<double, 1, 1>& v,
        const Eigen::Matrix<double, 1, 1>& a,
        const Eigen::Matrix<double, 1, 1>& /*j*/,
        const Eigen::Matrix<double, 1, 1>& /*s*/,
        Eigen::Matrix<double, 1, 1>& gp,
        Eigen::Matrix<double, 1, 1>& gv,
        Eigen::Matrix<double, 1, 1>& ga,
        Eigen::Matrix<double, 1, 1>& /*gj*/,
        Eigen::Matrix<double, 1, 1>& /*gs*/,
        double& /*gt*/
    ) const {
        gp.setZero();
        gv.setZero();
        ga.setZero();

        double cost = 0.0;

        double yaw_rate = v(0);
        double yaw_acc = a(0);

        double f, df;

        // yaw rate 约束
        smoothedL1(std::fabs(yaw_rate) - yaw_rate_max, f, df);
        cost += w_yaw_vel * f;
        gv(0) += w_yaw_vel * df * (yaw_rate > 0.0 ? 1.0 : -1.0);

        // yaw acc 约束
        smoothedL1(std::fabs(yaw_acc) - yaw_acc_max, f, df);
        cost += w_yaw_acc * f;
        ga(0) += w_yaw_acc * df * (yaw_acc > 0.0 ? 1.0 : -1.0);

        return cost;
    }
};

// ============================================================================
// 主优化器
// ============================================================================
class MaSplineTrajectoryOptimizer {
public:
    using Opt2D = SplineTrajectory::SplineOptimizer<2>;
    using OptYaw = SplineTrajectory::SplineOptimizer<1>;

    void set_config(const MaSplineOptimizerConfig& config) {
        config_ = config;
    }

    void set_esdf_interface(const ESDFInterface* esdf) {
        esdf_ = esdf;
    }

    // ========================================================================
    // 统一入口
    // ========================================================================
    MAsplineOutput optimize(const MaSplineInput& input) {
        MAsplineOutput out;

        switch (config_.mode) {
            case MaSplineOptimizerConfig::Mode::OMNI_XY:
                out = optimize_xy(input);
                break;

            case MaSplineOptimizerConfig::Mode::OMNI_XY_YAW:
                out = optimize_xy(input);
                if (out.success) {
                    out.yaw_spline = optimize_yaw(input, out.time_segments);
                }
                break;

            case MaSplineOptimizerConfig::Mode::OMNI_XY_YAW_JOINT:
                out = optimize_xy_yaw_joint(input);
                break;
        }

        return out;
    }

private:
    // ========================================================================
    // 2D xy 优化 L-BFGS 回调
    // ========================================================================
    struct CallbackCtx {
        Opt2D* optimizer = nullptr;
        Opt2D::OptimizationContext* ctx = nullptr;

        using Spec =
            decltype(Opt2D::makeEvaluateSpec(std::declval<TimeCost&>(), std::declval<RobotIntegralCost<2>&>()));

        Spec* spec = nullptr;
    };

    static double costCallback(void* instance, const Eigen::VectorXd& x, Eigen::VectorXd& g) {
        auto* c = static_cast<CallbackCtx*>(instance);
        return c->optimizer->evaluatePrepared(*c->ctx, x, g, *c->spec);
    }
    inline auto filterTimedTrajectory(const std::vector<TimedReferencePoint>& input, double min_spacing = 0.3)
        -> std::vector<TimedReferencePoint> {
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
    // ========================================================================
    // 2D xy 轨迹优化
    // 直接使用 timed_trajectory，不降采样
    // 点密度由上游 time_resolution 控制
    // ========================================================================
    MAsplineOutput optimize_xy(const MaSplineInput& input) {
        MAsplineOutput out;
        auto timed = filterTimedTrajectory(
            input.timed_trajectory,
            0.3 // 最小间距，可配置
        );

        if (timed.size() < 2) return out;
        // 1. 直接使用 timed_trajectory 全部点
        //const auto& timed = input.timed_trajectory;

        if (timed.size() < 2) return out;

        // 2. 构造绝对时间点 / waypoints
        std::vector<double> time_points;
        Eigen::Matrix<double, Eigen::Dynamic, 2> waypoints(timed.size(), 2);

        for (size_t i = 0; i < timed.size(); ++i) {
            time_points.push_back(timed[i].t);
            waypoints.row(i) = timed[i].pos.transpose();
        }

        // 3. 边界条件
        SplineTrajectory::BoundaryConditions<2> bc;
        bc.start_velocity = input.start_vel;
        bc.end_velocity = input.end_vel;
        bc.start_acceleration = input.start_acc;
        bc.end_acceleration = input.end_acc;

        // 4. 优化掩码：首尾固定，中间路点和时间优化
        const int N = static_cast<int>(timed.size()) - 1;

        SplineTrajectory::OptimizationMask mask;
        mask.time.assign(N, 1);
        mask.waypoints.assign(N + 1, 1);
        mask.waypoints.front() = 0;
        mask.waypoints.back() = 0;

        auto problem = Opt2D::makeProblemFromTimePoints(time_points, waypoints, bc, mask);

        // 5. 配置优化器
        Opt2D::OptimizerConfig opt_cfg;
        opt_cfg.rho_energy = config_.rho_energy;
        opt_cfg.integral_num_steps = config_.integral_num_steps;

        optimizer_.setConfig(opt_cfg);

        auto status = optimizer_.prepareContext(problem, ctx_);
        if (!status) return out;

        // 6. 代价函数
        TimeCost time_cost;
        time_cost.w_time = config_.weight_time;
        time_cost.w_mean = config_.weight_mean_time;
        time_cost.mean_lower = config_.mean_lower;
        time_cost.mean_upper = config_.mean_upper;
        time_cost.w_min_time = config_.weight_min_time;
        time_cost.min_time = config_.min_time;

        RobotIntegralCost<2> integral_cost;
        integral_cost.esdf = esdf_;
        integral_cost.w_obs = config_.weight_obstacle;
        integral_cost.safe_distance = config_.safe_distance;
        integral_cost.w_vel = config_.weight_vel;
        integral_cost.w_acc = config_.weight_acc;
        integral_cost.v_max = config_.v_max;
        integral_cost.a_max = config_.a_max;

        auto spec = Opt2D::makeEvaluateSpec(time_cost, integral_cost);

        // 7. L-BFGS 优化
        CallbackCtx cb {&optimizer_, &ctx_, &spec};

        Eigen::VectorXd x = optimizer_.generateInitialGuess(ctx_);
        double cost = 0.0;

        lbfgs::lbfgs_parameter_t params;
        params.max_iterations = config_.max_iterations;
        params.g_epsilon = config_.g_epsilon;
        params.mem_size = config_.lbfgs_mem_size;
        params.past = 3;
        params.delta = config_.lbfgs_delta;
        params.min_step = 1e-32;

        int ret =
            lbfgs::lbfgs_optimize(x, cost, &MaSplineTrajectoryOptimizer::costCallback, nullptr, nullptr, &cb, params);
        (void)ret;

        // 8. 同步最终样条
        optimizer_.synchronizeWorkingState(ctx_, x);

        out.xy_spline = optimizer_.getWorkingSpline(ctx_);
        out.time_segments = ctx_.runtime.state.times;
        out.start_time = ctx_.runtime.state.start_time;
        out.cost = cost;
        out.success = true;

        return out;
    }

    // ========================================================================
    // yaw 单独优化
    // 固定 xy 优化后的时间，只优化 yaw waypoints
    // ========================================================================
    SplineTrajectory::QuinticSplineND<1> optimize_yaw(const MaSplineInput& input, const std::vector<double>& fixed_Ts) {
        SplineTrajectory::QuinticSplineND<1> empty;

        if (fixed_Ts.empty()) return empty;

        const int N = static_cast<int>(fixed_Ts.size());

        // 直接使用 timed_trajectory，和 xy 点数一致
        if (input.timed_trajectory.size() != static_cast<size_t>(N + 1)) return empty;

        // 由 fixed_Ts 构造绝对时间点
        std::vector<double> time_points;
        time_points.reserve(N + 1);

        double t = 0.0;
        time_points.push_back(t);
        for (double T: fixed_Ts) {
            t += T;
            time_points.push_back(t);
        }

        // yaw unwrap，防止 ±π 跳变
        Eigen::Matrix<double, Eigen::Dynamic, 1> yaw_waypoints(N + 1, 1);

        double prev_yaw = input.timed_trajectory.front().yaw;
        for (int i = 0; i <= N; ++i) {
            double yaw = input.timed_trajectory[i].yaw;

            while (yaw - prev_yaw > M_PI)
                yaw -= 2.0 * M_PI;
            while (yaw - prev_yaw < -M_PI)
                yaw += 2.0 * M_PI;

            yaw_waypoints(i, 0) = yaw;
            prev_yaw = yaw;
        }

        // 边界条件
        SplineTrajectory::BoundaryConditions<1> bc;
        bc.start_velocity(0) = input.start_yaw_rate;
        bc.end_velocity(0) = input.end_yaw_rate;
        bc.start_acceleration(0) = input.start_yaw_acc;
        bc.end_acceleration(0) = input.end_yaw_acc;

        // 时间固定，只优化 yaw 路点
        SplineTrajectory::OptimizationMask mask;
        mask.time.assign(N, 0);
        mask.waypoints.assign(N + 1, 1);
        mask.waypoints.front() = 0;
        mask.waypoints.back() = 0;

        auto problem = OptYaw::makeProblemFromTimePoints(time_points, yaw_waypoints, bc, mask);

        // 配置优化器
        OptYaw::OptimizerConfig opt_cfg;
        opt_cfg.rho_energy = config_.rho_energy;
        opt_cfg.integral_num_steps = config_.integral_num_steps;

        yaw_optimizer_.setConfig(opt_cfg);

        auto status = yaw_optimizer_.prepareContext(problem, yaw_ctx_);
        if (!status) return empty;

        // 时间固定，因此时间代价设 0
        TimeCost time_cost;
        time_cost.w_time = 0.0;
        time_cost.w_mean = 0.0;

        YawIntegralCost yaw_cost;
        yaw_cost.w_yaw_vel = config_.weight_yaw_vel;
        yaw_cost.w_yaw_acc = config_.weight_yaw_acc;
        yaw_cost.yaw_rate_max = config_.yaw_rate_max;
        yaw_cost.yaw_acc_max = config_.yaw_acc_max;

        auto spec = OptYaw::makeEvaluateSpec(time_cost, yaw_cost);

        struct YawCallbackCtx {
            OptYaw* optimizer;
            OptYaw::OptimizationContext* ctx;
            decltype(spec)* spec_ptr;
        } ycb {&yaw_optimizer_, &yaw_ctx_, &spec};

        static auto yawCallback = [](void* instance, const Eigen::VectorXd& x, Eigen::VectorXd& g) -> double {
            auto* c = static_cast<YawCallbackCtx*>(instance);
            return c->optimizer->evaluatePrepared(*c->ctx, x, g, *c->spec_ptr);
        };

        Eigen::VectorXd x = yaw_optimizer_.generateInitialGuess(yaw_ctx_);
        double cost = 0.0;

        lbfgs::lbfgs_parameter_t params;
        params.max_iterations = config_.max_iterations;
        params.g_epsilon = config_.g_epsilon;
        params.mem_size = config_.lbfgs_mem_size;
        params.past = 3;
        params.delta = config_.lbfgs_delta;
        params.min_step = 1e-32;

        int ret = lbfgs::lbfgs_optimize(x, cost, yawCallback, nullptr, nullptr, &ycb, params);
        (void)ret;

        yaw_optimizer_.synchronizeWorkingState(yaw_ctx_, x);

        return yaw_optimizer_.getWorkingSpline(yaw_ctx_);
    }

    // ========================================================================
    // 联合优化 (x,y,yaw)
    // 当前先留空
    // ========================================================================
    MAsplineOutput optimize_xy_yaw_joint(const MaSplineInput& input) {
        MAsplineOutput out;
        (void)input;
        return out;
    }

private:
    MaSplineOptimizerConfig config_;
    const ESDFInterface* esdf_ = nullptr;

    Opt2D optimizer_;
    Opt2D::OptimizationContext ctx_;

    OptYaw yaw_optimizer_;
    OptYaw::OptimizationContext yaw_ctx_;
};

} // namespace ma_spline_opt
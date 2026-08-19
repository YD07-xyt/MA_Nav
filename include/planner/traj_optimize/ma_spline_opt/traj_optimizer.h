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
    bool check_trajectory_collision(const MAsplineOutput& out, const ESDFInterface* esdf, double safe_distance) {
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

    MAsplineOutput optimize(const MaSplineInput& input) {
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

private:
    SplineTrajectory::QuinticSplineND<2> last_xy_spline_;
    bool has_last_traj_ = false;

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

        lbfgs::lbfgs_optimize(x, cost1, &MaSplineTrajectoryOptimizer::costCallback, nullptr, nullptr, &cb1, params1);

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

        lbfgs::lbfgs_optimize(x, cost2, &MaSplineTrajectoryOptimizer::costCallback, nullptr, nullptr, &cb2, params2);

        // 同步最终结果
        optimizer_.synchronizeWorkingState(ctx_, x);

        out.xy_spline = optimizer_.getWorkingSpline(ctx_);
        out.time_segments = ctx_.runtime.state.times;
        out.start_time = ctx_.runtime.state.start_time;
        out.cost = cost2;
        out.success = true;

        return out;
    }

    // ========================================================================
    // yaw 单独优化
    // 固定 xy 优化后的时间，只优化 yaw waypoints
    // ========================================================================

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
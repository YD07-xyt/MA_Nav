#pragma once

#include "SplineTrajectory/SplineOptimizer.hpp"
#include "optimizer_config.h"
#include "cost.hpp"
#include "planner/traj_optimize/esdf_Interface.hpp"
#include "utils/expected.hpp"
#include "utils/lbfgs.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ma_spline_opt {

// ============================================================================
// 主优化器
// ============================================================================
class MaSplineTrajectoryOptimizer {
public:
    using Opt2D = SplineTrajectory::SplineOptimizer<2>;
    using Opt3D = SplineTrajectory::SplineOptimizer<3>;

    void set_config(const MaSplineOptimizerConfig& config) {
        config_ = config;
    }

    void set_esdf_interface(const ESDFInterface* esdf) {
        esdf_ = esdf;
    }

    // ========================================================================
    // 统一入口
    // ========================================================================
    auto optimize(const MaSplineInput& input) -> MAsplineOutput;

    bool check_trajectory_collision(const MAsplineOutput& out, const ESDFInterface* esdf, double safe_distance);

private:
    enum MaOptError{
        TIMED_TRAJ_NULL,
        LBFGS_OPT_FAILED,
        SYNC_WORKING_STATE_FAILED,
    };
    using output= tl::expected<MAsplineOutput,MaOptError>;

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

    static double cost_callback_2d(void* instance, const Eigen::VectorXd& x, Eigen::VectorXd& g) {
        auto* c = static_cast<CallbackCtx*>(instance);
        return c->optimizer->evaluatePrepared(*c->ctx, x, g, *c->spec);
    }
    auto optimize_2d_stage(
        const SplineTrajectory::SplineOptimizer<2>::ProblemDefinition& problem,
        const StageOptimizerConfig& stage,
        Eigen::VectorXd& x,
        double& final_cost
    ) -> bool;
    // ========================================================================
    // 2D xy 轨迹优化
    // 直接使用 timed_trajectory，不降采样
    // 点密度由上游 time_resolution 控制
    // ========================================================================
    auto optimize_xy(const MaSplineInput& input) -> output;

private:
    struct CallbackCtx3D {
        Opt3D* optimizer = nullptr;
        Opt3D::OptimizationContext* ctx = nullptr;

        using Spec =
            decltype(Opt3D::makeEvaluateSpec(std::declval<TimeCost&>(), std::declval<RobotIntegralCost<3>&>()));

        Spec* spec = nullptr;
    };
    static double cost_callback_3d(void* instance, const Eigen::VectorXd& x, Eigen::VectorXd& g) {
        auto* c = static_cast<CallbackCtx3D*>(instance);
        return c->optimizer->evaluatePrepared(*c->ctx, x, g, *c->spec);
    }
    auto optimize_3d_stage(
        const SplineTrajectory::SplineOptimizer<3>::ProblemDefinition& problem,
        const StageOptimizerConfig& stage,
        Eigen::VectorXd& x,
        double& final_cost
    ) -> bool;

    // ========================================================================
    // 联合优化 (x,y,yaw)
    // ========================================================================
    auto optimize_xy_yaw_joint(const MaSplineInput& input) -> output;

private:
    MaSplineOptimizerConfig config_;
    const ESDFInterface* esdf_ = nullptr;

    Opt2D optimizer_2d_;
    Opt2D::OptimizationContext ctx_2d_;

    Opt3D optimizer_3d_;
    Opt3D::OptimizationContext ctx_3d_;
};

} // namespace ma_spline_opt
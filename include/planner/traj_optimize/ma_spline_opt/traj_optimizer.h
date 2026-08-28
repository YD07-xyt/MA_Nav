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
    auto optimize(const MaSplineInput& input) -> MAsplineOutput;
    
    bool check_trajectory_collision(const MAsplineOutput& out, const ESDFInterface* esdf, double safe_distance);

    

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

    static double cost_callback(void* instance, const Eigen::VectorXd& x, Eigen::VectorXd& g) {
        auto* c = static_cast<CallbackCtx*>(instance);
        return c->optimizer->evaluatePrepared(*c->ctx, x, g, *c->spec);
    }
    auto filter_timed_trajectory(const std::vector<TimedReferencePoint>& input, double min_spacing = 0.3)
        -> std::vector<TimedReferencePoint>;
    // ========================================================================
    // 2D xy 轨迹优化
    // 直接使用 timed_trajectory，不降采样
    // 点密度由上游 time_resolution 控制
    // ========================================================================
    auto optimize_xy(const MaSplineInput& input) -> MAsplineOutput;

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
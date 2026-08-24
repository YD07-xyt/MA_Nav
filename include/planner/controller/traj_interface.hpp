#pragma once

#include "planner/controller/mpc.h"
#include "planner/traj_optimize/ma_spline_opt/traj_optimizer.h"

namespace control {

class MaSplineTrajectoryInterface: public TarjectoryInterfaces {
public:
    explicit MaSplineTrajectoryInterface(ma_spline_opt::MAsplineOutput output): output_(std::move(output)) {}

    bool valid() const override {
        return output_.success && output_.xy_spline.isInitialized();
    }

    double duration() const override {
        if (!valid()) return 0.0;
        return output_.xy_spline.getDuration();
    }

    bool sample(double t, ReferencePoint& ref) const override {
        if (!valid()) return false;

        // 防止越界
        double t_clamped = std::clamp(t, 0.0, duration());

        const auto& traj = output_.xy_spline.getTrajectory();

        Eigen::Vector2d p = traj.evaluate(t_clamped, 0);
        Eigen::Vector2d v = traj.evaluate(t_clamped, 1);
        Eigen::Vector2d a = traj.evaluate(t_clamped, 2);

        ref.time = t_clamped;
        ref.state << p.x(), p.y(), 0.0, v.x(), v.y(), 0.0;
        ref.input << a.x(), a.y(), 0.0;

        // 如果以后要 yaw/wz/az：
        // 可以从 yaw_spline 里取，或者用 atan2(vy, vx) 算 yaw
        return true;
    }
    
double nearest_time(const Eigen::Vector2d& pos) const override {
    if (!valid()) return 0.0;

    const double dur = duration();
    double best_t = 0.0;
    double best_d = std::numeric_limits<double>::max();

    // 只有新轨迹才全局搜索
    for (double t = 0.0; t <= dur + 1e-6; t += 0.1) {
        double d = (sample_pos(t) - pos).squaredNorm();
        if (d < best_d) {
            best_d = d;
            best_t = t;
        }
    }

    return best_t;
}

double update_track_time(
    const Eigen::Vector2d& pos,
    double hint) const override {

    if (!valid()) return 0.0;

    const double dur = duration();
    double best_t = std::clamp(hint, 0.0, dur);
    double best_d = (sample_pos(best_t) - pos).squaredNorm();

    // 每帧只搜索 hint 附近的小窗口
    double t_lo = std::max(0.0, hint - 0.3);
    double t_hi = std::min(dur, hint + 3.0);

    for (double t = t_lo; t <= t_hi + 1e-6; t += 0.02) {
        double d = (sample_pos(t) - pos).squaredNorm();
        if (d < best_d) {
            best_d = d;
            best_t = t;
        }
    }

    return best_t;
}


private:
    Eigen::Vector2d sample_pos(double t) const {
        const auto& traj = output_.xy_spline.getTrajectory();
        return traj.evaluate(std::clamp(t, 0.0, duration()), 0);
    }
    bool sample_sequence(double t_start, double dt, int N, std::vector<ReferencePoint>& refs) const override {
        if (!valid()) return false;

        refs.clear();
        refs.reserve(N + 1);

        for (int k = 0; k <= N; ++k) {
            ReferencePoint ref;
            if (!sample(t_start + k * dt, ref)) {
                return false;
            }
            refs.push_back(ref);
        }

        return true;
    }

private:
    ma_spline_opt::MAsplineOutput output_;
};

} // namespace control
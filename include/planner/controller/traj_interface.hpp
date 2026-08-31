#pragma once

#include "planner/controller/mpc.h"
#include "planner/traj_optimize/ma_spline_opt/traj_optimizer.h"

#include <algorithm>
#include <limits>
#include <utility>

namespace control {

class MaSplineTrajectoryInterface: public TarjectoryInterfaces {
public:
    explicit MaSplineTrajectoryInterface(ma_spline_opt::MAsplineOutput output): output_(std::move(output)) {}

    bool valid() const override {
        return output_.success && output_.trajectory.isInitialized();
    }

    double duration() const override {
        if (!valid()) return 0.0;
        return output_.trajectory.getDuration();
    }

    bool sample(double t, ReferencePoint& ref) const override {
        if (!valid()) return false;

        const double local_time = std::clamp(t, 0.0, duration());
        const double spline_time = output_.trajectory.getStartTime() + local_time;
        const auto& traj = output_.trajectory.getTrajectory();

        const Eigen::Vector3d p = traj.evaluate(spline_time, 0);
        const Eigen::Vector3d v = traj.evaluate(spline_time, 1);
        const Eigen::Vector3d a = traj.evaluate(spline_time, 2);

        ref.time = local_time;
        ref.state << p.x(), p.y(), p.z(), v.x(), v.y(), v.z();
        ref.input << a.x(), a.y(), a.z();
        return true;
    }

    double nearest_time(const Eigen::Vector2d& pos) const override {
        if (!valid()) return 0.0;

        const double dur = duration();
        double best_t = 0.0;
        double best_d = std::numeric_limits<double>::max();

        for (double t = 0.0; t <= dur + 1e-6; t += 0.1) {
            const double d = (sample_pos(t) - pos).squaredNorm();
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

        const double t_lo = std::max(0.0, hint - 0.3);
        const double t_hi = std::min(dur, hint + 3.0);

        for (double t = t_lo; t <= t_hi + 1e-6; t += 0.02) {
            const double d = (sample_pos(t) - pos).squaredNorm();
            if (d < best_d) {
                best_d = d;
                best_t = t;
            }
        }

        return best_t;
    }

    bool sample_sequence(
        double t_start,
        double dt,
        int N,
        std::vector<ReferencePoint>& refs) const override {
        if (!valid() || dt < 0.0 || N < 0) return false;

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
    Eigen::Vector2d sample_pos(double t) const {
        const double local_time = std::clamp(t, 0.0, duration());
        const double spline_time = output_.trajectory.getStartTime() + local_time;
        return output_.trajectory.getTrajectory().evaluate(spline_time, 0).head<2>();
    }

    ma_spline_opt::MAsplineOutput output_;
};

} // namespace control

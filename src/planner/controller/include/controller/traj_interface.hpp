#pragma once


#include "traj_optimize/ma_spline_opt/traj_optimizer.h"

#include <algorithm>
#include <limits>
#include <utility>

namespace control {
// ============================================================
// 轨迹接口
// ============================================================
struct TrajectoryTypes {
    static constexpr int K_STATE_DIM = 4;  // x, y, vx, vy
    static constexpr int K_INPUT_DIM = 2;  // ax, ay

    using StateVector = Eigen::Matrix<double, K_STATE_DIM, 1>;
    using InputVector = Eigen::Matrix<double, K_INPUT_DIM, 1>;

    struct ReferencePoint {
        double time = 0.0;
        StateVector state = StateVector::Zero();
        InputVector input = InputVector::Zero();
    };
};

template <typename Derived>
class TrajectoryInterface : public TrajectoryTypes {
public:
    bool valid() const {
        return static_cast<const Derived*>(this)->validImpl();
    }
    double duration() const {
        return static_cast<const Derived*>(this)->durationImpl();
    }

    bool sample(double t, ReferencePoint& ref) const {
        return static_cast<const Derived*>(this)->sampleImpl(t, ref);
    }

    double nearest_time(const Eigen::Vector2d& pos) const {
        return static_cast<const Derived*>(this)->nearestTimeImpl(pos);
    }

    double update_track_time(const Eigen::Vector2d& pos, double hint) const {
        return static_cast<const Derived*>(this)->updateTrackTimeImpl(pos, hint);
    }

    bool sample_sequence(double t_start, double dt, int N,
                         std::vector<ReferencePoint>& refs) const {
        return static_cast<const Derived*>(this)->sampleSequenceImpl(
            t_start, dt, N, refs);
    }

protected:
    TrajectoryInterface() = default;
    ~TrajectoryInterface() = default;   // 非虚
};
class MaSplineTrajectoryInterface
    : public TrajectoryInterface<MaSplineTrajectoryInterface> {
public:
    explicit MaSplineTrajectoryInterface(ma_spline_opt::MAsplineOutput output)
        : output_(std::move(output)) {}

    bool validImpl() const {
        return output_.success && output_.trajectory.isInitialized();
    }



    double durationImpl() const {
        if (!validImpl()) return 0.0;
        return output_.trajectory.getDuration();
    }

    bool sampleImpl(double t, ReferencePoint& ref) const {
        if (!validImpl()) return false;
        const double local_time = std::clamp(t, 0.0, durationImpl());
        const double spline_time = output_.trajectory.getStartTime() + local_time;
        const auto& traj = output_.trajectory.getTrajectory();
        const Eigen::Vector3d p = traj.evaluate(spline_time, 0);
        const Eigen::Vector3d v = traj.evaluate(spline_time, 1);
        const Eigen::Vector3d a = traj.evaluate(spline_time, 2);
        ref.time = local_time;
        ref.state << p.x(), p.y(), v.x(), v.y();
        ref.input << a.x(), a.y();
        return true;
    }

    double nearestTimeImpl(const Eigen::Vector2d& pos) const {
        if (!validImpl()) return 0.0;
        const double dur = durationImpl();
        double best_t = 0.0;
        double best_d = std::numeric_limits<double>::max();
        for (double t = 0.0; t <= dur + 1e-6; t += 0.1) {
            const double d = (samplePos(t) - pos).squaredNorm();
            if (d < best_d) { best_d = d; best_t = t; }
        }
        return best_t;
    }

    double updateTrackTimeImpl(const Eigen::Vector2d& pos, double hint) const {
        if (!validImpl()) return 0.0;
        const double dur = durationImpl();
        double best_t = std::clamp(hint, 0.0, dur);
        double best_d = (samplePos(best_t) - pos).squaredNorm();
        const double t_lo = std::max(0.0, hint - 0.3);
        const double t_hi = std::min(dur, hint + 3.0);
        for (double t = t_lo; t <= t_hi + 1e-6; t += 0.02) {
            const double d = (samplePos(t) - pos).squaredNorm();
            if (d < best_d) { best_d = d; best_t = t; }
        }
        return best_t;
    }

    bool sampleSequenceImpl(double t_start, double dt, int N,
                            std::vector<ReferencePoint>& refs) const {
        if (!validImpl() || dt < 0.0 || N < 0) return false;
        refs.clear();
        refs.reserve(N + 1);
        for (int k = 0; k <= N; ++k) {
            ReferencePoint ref;
            if (!sampleImpl(t_start + k * dt, ref)) return false;
            refs.push_back(ref);
        }
        return true;
    }

private:
    Eigen::Vector2d samplePos(double t) const {
        const double local_time = std::clamp(t, 0.0, durationImpl());
        const double spline_time = output_.trajectory.getStartTime() + local_time;
        return output_.trajectory.getTrajectory().evaluate(spline_time, 0).head<2>();
    }

    ma_spline_opt::MAsplineOutput output_;
};


} // namespace control

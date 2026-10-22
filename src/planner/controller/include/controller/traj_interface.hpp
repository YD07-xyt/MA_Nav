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



    // 时长 = 刹车段 + 样条（无刹车段时等于样条时长）
    double durationImpl() const {
        if (!validImpl()) return 0.0;
        return ma_spline_opt::total_duration(output_);
    }

    // 刹车段时长（局部时间 [0, brake_duration) 内是刹车段）
    double brake_duration() const { return output_.brake.duration; }
    // 刹车距离不够时为 true：刹车段内控制器直接发零速
    bool brake_emergency() const { return output_.brake.emergency; }

    bool sampleImpl(double t, ReferencePoint& ref) const {
        if (!validImpl()) return false;
        const double local_time = std::clamp(t, 0.0, durationImpl());
        Eigen::Vector2d p, v, a;
        ma_spline_opt::sample_xy(output_, local_time, p, v, a);
        ref.time = local_time;
        ref.state << p.x(), p.y(), v.x(), v.y();
        ref.input << a.x(), a.y();
        return true;
    }

    double nearestTimeImpl(const Eigen::Vector2d& pos) const {
        if (!validImpl()) return 0.0;
        // 有刹车段时新轨迹从规划时的机器人位置开始，游标从 0 起按时间推进（原因同 updateTrackTimeImpl）
        if (output_.brake.duration > 0.0) return 0.0;
        const double dur = durationImpl();
        double best_t = 0.0;
        double best_d = std::numeric_limits<double>::max();
        for (double t = 0.0; t <= dur + 1e-6; t += 0.1) {
            const double d = (samplePos(t) - pos).squaredNorm();
            if (d < best_d) { best_d = d; best_t = t; }
        }
        return best_t;
    }

    // 只在 [t_from, 终点] 上找最近时刻（急停结束后跳过刹车段用）
    double nearest_time_from(const Eigen::Vector2d& pos, double t_from) const {
        if (!validImpl()) return 0.0;
        const double dur = durationImpl();
        double best_t = std::clamp(t_from, 0.0, dur);
        double best_d = std::numeric_limits<double>::max();
        for (double t = best_t; t <= dur + 1e-6; t += 0.02) {
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
        // 反向样条会原路经过刹车段走过的位置，按位置搜索会落回刹车段（参考速度又指向前方）而卡住。
        // 刹车段内游标由调用方按时间推进（这里原样返回）；刹车段结束后只在样条上搜索。
        if (hint < output_.brake.duration) {
            return std::max(0.0, hint);
        }
        const double t_lo = std::max(output_.brake.duration, hint - 0.3);
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
        Eigen::Vector2d p, v, a;
        ma_spline_opt::sample_xy(output_, local_time, p, v, a);
        return p;
    }

    ma_spline_opt::MAsplineOutput output_;
};


} // namespace control

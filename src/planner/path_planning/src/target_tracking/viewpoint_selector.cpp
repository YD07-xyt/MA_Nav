#include "path_planning/target_tracking/viewpoint_selector.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace path_planning::target_tracking {

namespace {
auto in_ignore(const Eigen::Vector2d& p, const std::vector<IgnoreCircle>& ignore) -> bool {
    for (const auto& c : ignore) {
        if ((p - c.center).squaredNorm() <= c.radius * c.radius) return true;
    }
    return false;
}
} // namespace

auto ViewpointSelector::line_of_sight(
    const grid_map::GridMap& map,
    const Eigen::Vector2d& p0,
    const Eigen::Vector2d& p1,
    const std::vector<IgnoreCircle>& ignore
) const -> bool {
    // 暂时处理：所有占据格都视为遮挡（不区分障碍高度），见头文件说明。
    if (params_.los_clearance > 0.0) {
        const double len = (p1 - p0).norm();
        const double step = 0.5 * map.getResolution();
        const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
        for (int i = 0; i <= n; ++i) {
            const Eigen::Vector2d p = p0 + (p1 - p0) * (static_cast<double>(i) / n);
            if (!map.isInsideMap(p) || in_ignore(p, ignore)) continue;
            if (map.getDistance(p) < params_.los_clearance) return false;
        }
        return true;
    }

    // 2D DDA (Amanatides & Woo)：逐格遍历线段经过的所有栅格
    const double res = map.getResolution();
    const Eigen::Vector2d origin = map.getOrigin();
    const Eigen::Vector2d a = (p0 - origin) / res; // 栅格坐标（连续）
    const Eigen::Vector2d b = (p1 - origin) / res;
    Eigen::Vector2i idx(static_cast<int>(std::floor(a.x())), static_cast<int>(std::floor(a.y())));
    const Eigen::Vector2i end(static_cast<int>(std::floor(b.x())), static_cast<int>(std::floor(b.y())));
    const Eigen::Vector2d d = b - a;

    Eigen::Vector2i step;
    Eigen::Vector2d t_max, t_delta;
    for (int i = 0; i < 2; ++i) {
        if (d(i) > 0) {
            step(i) = 1;
            t_delta(i) = 1.0 / d(i);
            t_max(i) = (std::floor(a(i)) + 1.0 - a(i)) * t_delta(i);
        } else if (d(i) < 0) {
            step(i) = -1;
            t_delta(i) = -1.0 / d(i);
            t_max(i) = (a(i) - std::floor(a(i))) * t_delta(i);
        } else {
            step(i) = 0;
            t_delta(i) = std::numeric_limits<double>::infinity();
            t_max(i) = std::numeric_limits<double>::infinity();
        }
    }

    const Eigen::Vector2i voxel_num = map.getVoxelNum();
    auto blocked = [&](const Eigen::Vector2i& id) {
        // 地图外视为未知，不遮挡
        if (id.x() < 0 || id.y() < 0 || id.x() >= voxel_num.x() || id.y() >= voxel_num.y()) return false;
        if (!map.isOccupied(id)) return false;
        Eigen::Vector2d c;
        map.indexToPos(id, c);
        return !in_ignore(c, ignore);
    };

    const int max_iter = std::abs(end.x() - idx.x()) + std::abs(end.y() - idx.y()) + 2;
    for (int it = 0; it < max_iter; ++it) {
        if (blocked(idx)) return false;
        if (idx == end) break;
        if (t_max.x() < t_max.y()) {
            idx.x() += step.x();
            t_max.x() += t_delta.x();
        } else {
            idx.y() += step.y();
            t_max.y() += t_delta.y();
        }
    }
    return true;
}

auto ViewpointSelector::is_safe(const grid_map::GridMap& map, const Eigen::Vector2d& p) const -> bool {
    // 观测点必须在地图内：地图外没有障碍信息，不能作为可达目标
    if (!map.isInsideMap(p)) return false;
    return map.getDistance(p) >= params_.viewpoint_clearance;
}

auto ViewpointSelector::valid_viewpoint(
    const grid_map::GridMap& map,
    const Eigen::Vector2d& p,
    const Eigen::Vector2d& center,
    const std::vector<IgnoreCircle>& ignore
) const -> bool {
    return is_safe(map, p) && line_of_sight(map, p, center, ignore);
}

auto ViewpointSelector::predict(
    const grid_map::GridMap& map,
    const Eigen::Vector2d& p0,
    const Eigen::Vector2d& v0,
    double t0
) const -> std::vector<Eigen::Vector2d> {
    // 暂时处理：匀速外推 + 障碍截断，不预测绕障/转弯（后续可换成加速度空间搜索）。
    const double dt = std::max(params_.tracking_dt, 1e-3);
    const int n = std::max(1, static_cast<int>(std::round(params_.tracking_dur / dt)));
    // 目标自身会被 mid360 建成障碍：起始位置附近的占据格不参与截断
    const std::vector<IgnoreCircle> ignore{{p0, params_.target_ignore_radius}};

    auto occupied = [&](const Eigen::Vector2d& p) {
        if (!map.isInsideMap(p) || in_ignore(p, ignore)) return false;
        Eigen::Vector2i id;
        map.posToIndex(p, id);
        return map.isOccupied(id);
    };

    std::vector<Eigen::Vector2d> out;
    out.reserve(n + 1);
    // 从 p0 起以小步长推进到 t0，再按 dt 输出，保证截断检测不跳过薄墙
    const double check_step = 0.5 * map.getResolution();
    Eigen::Vector2d p = p0;
    bool stopped = false;
    auto advance = [&](double t_from, double t_to) {
        if (stopped) return;
        const Eigen::Vector2d delta = v0 * (t_to - t_from);
        const int k = std::max(1, static_cast<int>(std::ceil(delta.norm() / check_step)));
        for (int i = 1; i <= k; ++i) {
            const Eigen::Vector2d q = p + delta / k;
            if (occupied(q)) {
                stopped = true;
                return;
            }
            p = q;
        }
    };

    advance(0.0, std::max(t0, 0.0));
    out.push_back(p);
    double t = std::max(t0, 0.0);
    for (int i = 0; i < n; ++i) {
        advance(t, t + dt);
        t += dt;
        out.push_back(p);
    }
    return out;
}

auto ViewpointSelector::visible_arc(
    const grid_map::GridMap& map,
    const Eigen::Vector2d& center,
    double radius,
    double angle,
    const std::vector<IgnoreCircle>& ignore,
    double& t_l,
    double& t_r
) const -> void {
    const double dth = std::max(params_.search_angle_step, 1e-3);
    auto visible = [&](double th) {
        const Eigen::Vector2d p = center + radius * Eigen::Vector2d(std::cos(th), std::sin(th));
        return line_of_sight(map, p, center, ignore);
    };
    // 与 visible_pair 一致：各向一侧最多扫 pi，遇到遮挡退回上一步
    for (t_l = angle - dth; t_l > angle - M_PI; t_l -= dth) {
        if (!visible(t_l)) {
            t_l += dth;
            break;
        }
    }
    t_l = std::max(t_l, angle - M_PI);
    for (t_r = angle + dth; t_r < angle + M_PI; t_r += dth) {
        if (!visible(t_r)) {
            t_r -= dth;
            break;
        }
    }
    t_r = std::min(t_r, angle + M_PI);
}

auto ViewpointSelector::select(
    const grid_map::GridMap& map,
    const Eigen::Vector2d& center,
    const Eigen::Vector2d& robot,
    double fallback_dir,
    const std::vector<IgnoreCircle>& ignore
) const -> ViewpointInfo {
    ViewpointInfo info;
    info.center = center;

    const Eigen::Vector2d dir = robot - center;
    const double angle0 = dir.norm() > 1e-3 ? std::atan2(dir.y(), dir.x()) : fallback_dir;
    const double r_min = std::max(params_.camera_min_dist, 1e-2);
    const double r_max = std::max(params_.camera_max_dist, r_min);
    const double r_des = std::clamp(params_.tracking_dist, r_min, r_max);
    auto point_at = [&](double r, double th) -> Eigen::Vector2d {
        return center + r * Eigen::Vector2d(std::cos(th), std::sin(th));
    };

    // 1. 弹簧点
    double best_r = r_des;
    double best_th = angle0;
    bool found = valid_viewpoint(map, point_at(r_des, angle0), center, ignore);
    info.spring_used = found;

    // 2. 弹簧点不可用：在环 [r_min, r_max] 上找离弹簧点最近的可用点
    if (!found) {
        std::vector<double> radii{r_des};
        const double dr = std::max(params_.tolerance_d, 0.05);
        for (int k = 1;; ++k) {
            const double lo = r_des - k * dr;
            const double hi = r_des + k * dr;
            if (lo < r_min && hi > r_max) break;
            if (lo >= r_min) radii.push_back(lo);
            if (hi <= r_max) radii.push_back(hi);
        }
        const double dth = std::max(params_.search_angle_step, 1e-3);
        const int n_ang = static_cast<int>(std::ceil(M_PI / dth));
        double best_cost = std::numeric_limits<double>::max();
        for (const double r: radii) {
            // 固定半径下，离弹簧点的距离随 |Δθ| 单调增加：两侧各找第一个可用角即为该半径的最优
            for (int k = 0; k <= n_ang; ++k) {
                const double dth_k = std::min(k * dth, M_PI);
                const double cost = std::sqrt(std::max(0.0, r * r + r_des * r_des - 2.0 * r * r_des * std::cos(dth_k)));
                if (cost >= best_cost) break;
                bool hit = false;
                for (const double sgn: {1.0, -1.0}) {
                    const double th = angle0 + sgn * dth_k;
                    if (valid_viewpoint(map, point_at(r, th), center, ignore)) {
                        best_cost = cost;
                        best_r = r;
                        best_th = th;
                        hit = true;
                        break;
                    }
                    if (k == 0) break; // Δθ=0 两侧相同
                }
                if (hit) break;
            }
        }
        found = best_cost < std::numeric_limits<double>::max();
    }
    if (!found) return info;

    // 3. 可见扇形 + theta_clearance：观测点离遮挡边缘太近时推向扇形内部（同 visible_pair）
    double t_l = best_th, t_r = best_th;
    visible_arc(map, center, best_r, best_th, ignore, t_l, t_r);
    const double half = 0.5 * (t_r - t_l);
    const double theta_c = std::min(half, params_.theta_clearance);
    double th = best_th;
    if (best_th - t_l < theta_c) {
        th = t_l + theta_c;
    } else if (t_r - best_th < theta_c) {
        th = t_r - theta_c;
    }
    if (th != best_th) {
        if (valid_viewpoint(map, point_at(best_r, th), center, ignore)) {
            info.spring_used = false;
        } else {
            th = best_th; // 推移后的点不安全（扇形只检查视线），保留原点
        }
    }

    info.valid = true;
    info.radius = best_r;
    info.angle = th;
    info.arc_left = t_l;
    info.arc_right = t_r;
    info.viewpoint = point_at(best_r, th);
    return info;
}

} // namespace path_planning::target_tracking

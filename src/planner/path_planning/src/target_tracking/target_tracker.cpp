#include "path_planning/target_tracking/target_tracker.h"
#include <algorithm>
#include <cmath>

namespace path_planning::target_tracking {

auto TargetTracker::set_param(const TargetTrackingParams& params) -> void {
    params_ = params;
    estimator_.set_param(params);
    selector_.set_param(params);
}

auto TargetTracker::reset() -> void {
    estimator_.reset();
    state_ = TrackingState::IDLE;
    last_goal_.reset();
    has_gimbal_yaw_ = false;
}

auto TargetTracker::on_target_measurement(const Eigen::Vector2d& target_pos, double stamp) -> bool {
    return estimator_.update(target_pos, stamp);
}

auto TargetTracker::compute_gimbal(
    const RobotState2D& robot,
    const Eigen::Vector2d& target_pos,
    const Eigen::Vector2d& target_vel,
    bool visible,
    double obs_age
) -> GimbalCommand {
    GimbalCommand cmd;
    const Eigen::Vector2d aim = target_pos + target_vel * params_.gimbal_lead_time;
    const Eigen::Vector2d rel = aim - robot.pos;
    if (rel.norm() > 1e-3) {
        cmd.yaw = std::atan2(rel.y(), rel.x());
        // 视线角速度前馈：ω = (r × v_rel) / |r|^2
        const Eigen::Vector2d v_rel = target_vel - robot.vel;
        cmd.yaw_rate = (rel.x() * v_rel.y() - rel.y() * v_rel.x()) / rel.squaredNorm();
    } else {
        cmd.yaw = has_gimbal_yaw_ ? last_gimbal_yaw_ : robot.yaw;
    }
    cmd.yaw = normalize_angle(cmd.yaw);
    // 观测新鲜 + 视线畅通 → 视觉能看见目标，释放 yaw 给自瞄；否则由导航按预测指向
    cmd.release = params_.enable_yaw_release && state_ == TrackingState::TRACKING && visible
        && obs_age < params_.release_fresh_time;
    last_gimbal_yaw_ = cmd.yaw;
    has_gimbal_yaw_ = true;
    return cmd;
}

auto TargetTracker::update(const grid_map::GridMap& map, const RobotState2D& robot, double now) -> TrackingOutput {
    TrackingOutput out;
    if (!estimator_.initialized()) {
        state_ = TrackingState::IDLE;
        out.state = state_;
        out.gimbal.yaw = has_gimbal_yaw_ ? last_gimbal_yaw_ : normalize_angle(robot.yaw);
        return out;
    }

    // ---------- 1. 状态机（按最近有效观测的年龄） ----------
    const double age = std::max(0.0, now - estimator_.last_update_time());
    if (age < params_.target_timeout) {
        state_ = TrackingState::TRACKING;
    } else if (age < params_.lost_timeout) {
        state_ = TrackingState::PREDICTING;
    } else if (age < params_.search_timeout) {
        state_ = TrackingState::SEARCHING;
    } else {
        state_ = TrackingState::IDLE;
    }
    out.state = state_;
    if (state_ == TrackingState::IDLE) {
        last_goal_.reset();
        out.gimbal.yaw = has_gimbal_yaw_ ? last_gimbal_yaw_ : normalize_angle(robot.yaw);
        return out;
    }

    // ---------- 2. 预测 ----------
    Eigen::Vector2d center;
    if (state_ == TrackingState::SEARCHING) {
        // 暂时处理：长时丢失后不再外推，直接去最后看见的位置附近观察（不做主动搜索）
        out.target_pos = estimator_.pos();
        out.target_vel.setZero();
        out.prediction = {out.target_pos};
        center = out.target_pos;
    } else {
        out.prediction = selector_.predict(map, estimator_.pos(), estimator_.vel(), age);
        out.target_pos = out.prediction.front();
        out.target_vel = estimator_.vel();
        const double dt = std::max(params_.tracking_dt, 1e-3);
        const int idx = std::clamp(
            static_cast<int>(std::round(params_.lookahead_time / dt)),
            0,
            static_cast<int>(out.prediction.size()) - 1
        );
        center = out.prediction[idx];
    }

    // 目标本体会被 mid360 建成障碍：视线检测时排除当前位置与观测中心附近的占据
    const std::vector<IgnoreCircle> ignore{
        {out.target_pos, params_.target_ignore_radius},
        {center, params_.target_ignore_radius},
    };
    const bool visible_now = selector_.line_of_sight(map, robot.pos, out.target_pos, ignore);

    // ---------- 3. 云台 ----------
    out.gimbal = compute_gimbal(robot, out.target_pos, out.target_vel, visible_now, age);

    // ---------- 4. 平衡位置判定（同 Elastic-Tracker 悬停条件；云台可 360° 转，不要求底盘朝向） ----------
    const double dist_now = (robot.pos - out.target_pos).norm();
    out.hold = state_ == TrackingState::TRACKING && visible_now
        && std::abs(dist_now - params_.tracking_dist) < params_.tolerance_d
        && out.target_vel.norm() < params_.hover_target_vel && robot.vel.norm() < params_.hover_robot_vel;
    if (out.hold) {
        last_goal_.reset(); // 离开平衡后强制重新下发
        return out;
    }

    // ---------- 5. 选观测点 ----------
    const double fallback_dir = normalize_angle(robot.yaw + M_PI); // 与目标重合时：退到身后
    out.viewpoint = selector_.select(map, center, robot.pos, fallback_dir, ignore);
    if (!out.viewpoint.valid) {
        return out; // 无可行观测点：goal 为空，由调用方决定（执行旧轨迹/停车）
    }

    TrackingGoal goal;
    goal.pos = out.viewpoint.viewpoint;
    const Eigen::Vector2d to_center = center - goal.pos;
    goal.yaw = normalize_angle(std::atan2(to_center.y(), to_center.x()));

    if (!last_goal_ || (goal.pos - last_goal_->pos).norm() > params_.goal_update_dist) {
        last_goal_ = goal;
        out.goal_updated = true;
    }
    out.goal = last_goal_;
    return out;
}

} // namespace path_planning::target_tracking

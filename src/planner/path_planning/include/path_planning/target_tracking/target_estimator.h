#pragma once
/**
 * 目标状态估计：2D 匀速模型卡尔曼滤波，状态 x = [px, py, vx, vy]。
 * 对应 Elastic-Tracker 的 target_ekf（去掉 z、去掉相机投影，直接用世界系位置观测）。
 *
 * 暂时只用位置观测（world 系），未使用观测协方差/速度观测，观测噪声取固定参数 kf_meas_noise。
 */
#include "path_planning/target_tracking/tracking_types.h"
#include <Eigen/Core>

namespace path_planning::target_tracking {

class TargetEstimator {
public:
    using Vec4 = Eigen::Vector4d;
    using Mat4 = Eigen::Matrix4d;

    auto set_param(const TargetTrackingParams& params) -> void { params_ = params; }
    auto reset() -> void;

    /**
     * @brief 输入一次目标位置观测
     * @param z     目标位置（世界系）
     * @param stamp 观测时间戳 (s)，需与 TargetTracker::update 的 now 同一时钟
     * @return true 观测被接受；false 被新息门限 / 速度门限判为野值、或时间戳倒退而丢弃
     */
    auto update(const Eigen::Vector2d& z, double stamp) -> bool;

    auto initialized() const -> bool { return initialized_; }
    auto last_update_time() const -> double { return last_stamp_; }
    /// 最近一次有效观测时刻的滤波状态（不外推）
    auto pos() const -> Eigen::Vector2d { return x_.head<2>(); }
    auto vel() const -> Eigen::Vector2d { return x_.tail<2>(); }
    auto covariance() const -> const Mat4& { return P_; }

private:
    auto propagate(double dt, Vec4& x, Mat4& P) const -> void;

    TargetTrackingParams params_;
    bool initialized_ = false;
    double last_stamp_ = 0.0;
    Vec4 x_ = Vec4::Zero();
    Mat4 P_ = Mat4::Identity();
};

} // namespace path_planning::target_tracking

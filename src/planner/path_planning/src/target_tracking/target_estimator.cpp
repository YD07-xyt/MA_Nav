#include "path_planning/target_tracking/target_estimator.h"
#include <Eigen/Dense>

namespace path_planning::target_tracking {

auto TargetEstimator::reset() -> void {
    initialized_ = false;
    last_stamp_ = 0.0;
    x_.setZero();
    P_.setIdentity();
}

auto TargetEstimator::propagate(double dt, Vec4& x, Mat4& P) const -> void {
    Mat4 F = Mat4::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;
    // 白噪声加速度模型的离散过程噪声
    const double q = params_.kf_acc_noise * params_.kf_acc_noise;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;
    Mat4 Q = Mat4::Zero();
    for (int i = 0; i < 2; ++i) {
        Q(i, i) = 0.25 * dt4 * q;
        Q(i, i + 2) = 0.5 * dt3 * q;
        Q(i + 2, i) = 0.5 * dt3 * q;
        Q(i + 2, i + 2) = dt2 * q;
    }
    x = F * x;
    P = F * P * F.transpose() + Q;
}

auto TargetEstimator::update(const Eigen::Vector2d& z, double stamp) -> bool {
    const double meas_var = params_.kf_meas_noise * params_.kf_meas_noise;
    auto init_with = [&]() {
        x_ << z.x(), z.y(), 0.0, 0.0;
        P_.setZero();
        P_(0, 0) = P_(1, 1) = meas_var;
        // 初始速度未知：给一个与速度门限同量级的方差
        P_(2, 2) = P_(3, 3) = params_.target_vmax * params_.target_vmax;
        last_stamp_ = stamp;
        initialized_ = true;
    };

    if (!initialized_) {
        init_with();
        return true;
    }
    const double dt = stamp - last_stamp_;
    if (dt < 0.0) return false; // 时间戳倒退（乱序），丢弃
    if (dt > params_.kf_reset_dt) { // 间隔太久，旧速度已不可信
        init_with();
        return true;
    }

    Vec4 x = x_;
    Mat4 P = P_;
    if (dt > 0.0) propagate(dt, x, P);

    // H = [I 0]
    const Eigen::Matrix2d S = P.topLeftCorner<2, 2>() + meas_var * Eigen::Matrix2d::Identity();
    const Eigen::Matrix2d S_inv = S.inverse();
    const Eigen::Vector2d innov = z - x.head<2>();

    // 新息门限：跳变的观测（误检 / 换目标）直接拒绝；换目标时持续被拒，超过 kf_reset_dt 后走上面的重置分支
    if (params_.kf_gate_chi2 > 0.0 && innov.dot(S_inv * innov) > params_.kf_gate_chi2) return false;

    const Eigen::Matrix<double, 4, 2> K = P.leftCols<2>() * S_inv;
    const Vec4 x_new = x + K * innov;

    // 速度门限：更新后速度不合理 → 视为野值（与 Elastic-Tracker checkValid 一致）
    if (x_new.tail<2>().norm() > params_.target_vmax) return false;

    x_ = x_new;
    P_ = P - K * P.topRows<2>();
    last_stamp_ = stamp;
    return true;
}

} // namespace path_planning::target_tracking

#pragma once
/**
 * 目标跟踪（追击）模块的公共类型。
 *
 * 思路参考 Elastic-Tracker (3rd/Elastic-Tracker)：
 *   目标估计(KF) → 目标预测 → 以弹簧点为种子搜索“安全且可见”的观测点
 *   → 可见扇形 + theta_clearance 把观测点推离遮挡边缘 → 作为 goal 交给现有 FsmReplan。
 * 第一阶段不改优化器：可见性只在选点层保证，轨迹本身不带可见性/距离代价。
 *
 * 参数名尽量与 Elastic-Tracker 保持一致（tracking_dist / tolerance_d / theta_clearance /
 * tracking_dur / tracking_dt），方便对照调参。
 */
#include <Eigen/Core>
#include <optional>
#include <vector>

namespace path_planning::target_tracking {

struct TargetTrackingParams {
    // ---------- 跟踪距离 ----------
    double tracking_dist = 1.5; ///< 期望跟踪距离 d_des (m)，弹簧平衡长度
    double tolerance_d = 0.3; ///< 距离容差 (m)：|d - d_des| < tol 视为到位；也是半径回退搜索步长
    double camera_min_dist = 0.5; ///< 相机最小有效观测距离 (m)，观测点半径下限
    double camera_max_dist = 4.0; ///< 相机最大有效观测距离 (m)，观测点半径上限

    // ---------- 可见性 ----------
    double theta_clearance = 0.6; ///< 观测点到可见扇形边缘的最小角度余量 (rad)
    double viewpoint_clearance = 0.3; ///< 观测点的 ESDF 安全距离 (m)
    double los_clearance = 0.0; ///< 视线 ESDF 余量 (m)：0 = 只有占据格挡视线；>0 视线需离障碍更远
    double target_ignore_radius = 0.4; ///< 目标本体半径 (m)：mid360 会把目标自身建成障碍，视线/预测检测时忽略该圆内的占据
    double search_angle_step = 0.05; ///< 观测点 / 可见扇形的角度搜索步长 (rad)

    // ---------- 预测 ----------
    double tracking_dur = 2.0; ///< 预测时域 (s)
    double tracking_dt = 0.2; ///< 预测步长 (s)
    double lookahead_time = 1.0; ///< 取该时刻的预测点作为观测中心 (s)，应 ≤ tracking_dur

    // ---------- 目标估计 (匀速 KF) ----------
    double kf_acc_noise = 2.0; ///< 过程噪声：目标加速度标准差 (m/s^2)
    double kf_meas_noise = 0.1; ///< 观测噪声标准差 (m)
    double target_vmax = 4.0; ///< 速度门限 (m/s)：更新后速度超过该值的观测视为野值丢弃
    double kf_gate_chi2 = 13.8; ///< 新息马氏距离门限（2 自由度 χ²，13.8≈99.9%）；≤0 关闭。持续被拒超过 kf_reset_dt 会重置
    double kf_reset_dt = 1.0; ///< 两次有效观测间隔超过该值则重置滤波器 (s)

    // ---------- 丢失处理（按距最近一次有效观测的时间） ----------
    double target_timeout = 0.5; ///< 超过 → PREDICTING：按预测继续跟 (s)
    double lost_timeout = 2.0; ///< 超过 → SEARCHING：去最后看见的位置附近 (s)
    double search_timeout = 8.0; ///< 超过 → IDLE：放弃 (s)

    // ---------- 悬停 / 目标点更新 ----------
    double hover_target_vel = 0.2; ///< 目标速度低于该值才允许悬停 (m/s)
    double hover_robot_vel = 0.1; ///< 机器人速度低于该值才允许悬停 (m/s)
    double goal_update_dist = 0.2; ///< 新观测点与上次下发目标点距离超过该值才更新 goal (m)，抑制重规划抖动

    // ---------- 云台 ----------
    double gimbal_lead_time = 0.05; ///< 云台瞄准超前时间 (s)，补偿链路延迟
    bool enable_yaw_release = true; ///< 允许把云台 yaw 释放给视觉自瞄
    double release_fresh_time = 0.2; ///< 观测新鲜度 (s)：最近观测比该值新才释放 yaw
};

enum class TrackingState {
    IDLE, ///< 没有目标 / 已放弃
    TRACKING, ///< 观测新鲜，正常跟踪
    PREDICTING, ///< 短时丢失，按预测继续跟踪
    SEARCHING, ///< 长时丢失，去最后看见的位置附近观察
};

/// 机器人 2D 状态（世界系）
struct RobotState2D {
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d vel = Eigen::Vector2d::Zero();
    double yaw = 0.0; ///< 底盘 yaw (rad)
};

/// 下发给 FsmReplan 的跟踪目标点
struct TrackingGoal {
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();
    double yaw = 0.0; ///< 观测点指向观测中心的朝向 (rad)，可作为底盘参考朝向
};

/// 云台指令（世界系 yaw；云台可 360° 连续转，由调用方换算到云台相对角）
struct GimbalCommand {
    bool release = false; ///< true：导航释放 yaw，交给视觉自瞄；false：导航按 yaw 控制云台
    double yaw = 0.0; ///< 期望云台朝向（世界系，rad，已归一化到 [-pi, pi]）
    double yaw_rate = 0.0; ///< 视线角速度前馈 (rad/s)
};

/// 选点结果，含可见扇形（调试 / 可视化用）
struct ViewpointInfo {
    bool valid = false;
    Eigen::Vector2d center = Eigen::Vector2d::Zero(); ///< 观测中心（预测的目标位置）
    Eigen::Vector2d viewpoint = Eigen::Vector2d::Zero(); ///< 选出的观测点
    double radius = 0.0; ///< 观测点到中心的距离 (m)
    double angle = 0.0; ///< 观测点相对中心的方位角 (rad)
    double arc_left = 0.0; ///< 可见扇形左边界 t_l (rad，未归一化，t_l ≤ angle)
    double arc_right = 0.0; ///< 可见扇形右边界 t_r (rad，未归一化，t_r ≥ angle)
    bool spring_used = false; ///< true：直接用了弹簧点（未被遮挡/推移）
};

struct TrackingOutput {
    TrackingState state = TrackingState::IDLE;
    /// true：已在平衡位置（距离到位、目标与自身都几乎静止、视线畅通），调用方应停车保持
    bool hold = false;
    /// 跟踪目标点。nullopt 表示本周期没有可用目标点：
    ///   hold=true → 停车保持；hold=false → 无可行观测点或 IDLE，调用方自行决定（执行旧轨迹/停车）
    std::optional<TrackingGoal> goal;
    /// goal 相对上次下发是否变化超过 goal_update_dist（为 false 时 goal 与上次相同，无需重规划）
    bool goal_updated = false;
    GimbalCommand gimbal;

    Eigen::Vector2d target_pos = Eigen::Vector2d::Zero(); ///< 当前时刻估计的目标位置
    Eigen::Vector2d target_vel = Eigen::Vector2d::Zero(); ///< 当前估计的目标速度
    std::vector<Eigen::Vector2d> prediction; ///< 从当前时刻开始、步长 tracking_dt 的预测轨迹
    ViewpointInfo viewpoint;
};

} // namespace path_planning::target_tracking

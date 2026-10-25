#pragma once
/**
 * 目标跟踪（追击）模块的公共类型。完整说明见同目录 src/target_tracking/README.md。
 *
 * 思路参考 Elastic-Tracker (3rd/Elastic-Tracker)：
 *   目标估计(KF) → 目标预测 → 以弹簧点为种子搜索“安全且可见”的观测点
 *   → 可见扇形 + theta_clearance 把观测点推离遮挡边缘 → 作为 goal 交给现有 FsmReplan。
 * 第一阶段不改优化器：可见性只在选点层保证，轨迹本身不带可见性/距离代价。
 *
 * 参数名尽量与 Elastic-Tracker 保持一致（tracking_dist / tolerance_d / theta_clearance /
 * tracking_dur / tracking_dt），方便对照调参。原版 launch 参考值见各参数注释的“原版”。
 *
 * 单位：长度 m，时间 s，角度 rad，均为世界系。
 */
#include <Eigen/Core>
#include <optional>
#include <vector>

namespace path_planning::target_tracking {

/**
 * 跟踪参数。默认值按 RMUC 场地（0.05 m 栅格、哨兵最大速度 ~2.5 m/s、目标 ~1–3 m/s）给出，
 * 未经实车整定。调参建议顺序：
 *   1. 先定相机相关：camera_min_dist / camera_max_dist / tracking_dist（按相机 FOV、自瞄有效距离）
 *   2. 再定地图相关：viewpoint_clearance / target_ignore_radius（按车体半径、目标尺寸、地图是否膨胀）
 *   3. 再调稳定性：goal_update_dist / tolerance_d / theta_clearance（观察 goal 是否来回跳）
 *   4. 最后调估计与丢失：kf_* / *_timeout（看目标速度估计是否平滑、丢失后行为是否合理）
 */
struct TargetTrackingParams {
    // ======================================================================
    // 跟踪距离：决定“站在离目标多远的地方”
    // ======================================================================

    /// 期望跟踪距离 d_des (m)，即弹簧平衡长度；观测点优先选在离观测中心这么远的位置。
    /// - 调大：更安全（不易被撞、视野更容易覆盖目标），但更容易被障碍挡住视线、自瞄命中率下降。
    /// - 调小：更贴身，遮挡少，但目标机动时更容易跑出相机 FOV，且与目标碰撞风险增加。
    /// - 会被自动钳到 [camera_min_dist, camera_max_dist]。
    /// - 建议：自瞄最佳射击距离附近，RMUC 1.5–3.0。原版 2.5（无人机）。
    double tracking_dist = 1.5;

    /// 距离容差 (m)。两个作用：
    ///   ① 悬停判定：|当前距离 - tracking_dist| < tolerance_d 才可能 hold；
    ///   ② 弹簧点被挡时，按 tracking_dist ± k·tolerance_d 的半径序列向内/外回退搜索。
    /// - 调大：更容易 hold（少动），半径搜索更粗、更快；但站位精度降低。
    /// - 调小：站位更准，但 hold 难以触发，机器人会一直微调；半径搜索更细、略慢。
    /// - 下限 0.05（代码内部钳位）。建议 0.2–0.5。原版 0.3。
    double tolerance_d = 0.3;

    /// 相机最小有效观测距离 (m)：观测点半径下限。
    /// 近于此距离时目标可能超出相机垂直 FOV 或自瞄无法识别。按相机实测设置，建议 0.5–1.0。
    /// 注意：墙后目标会让选点向这个下限贴近（见 README“已知问题”），想站远些就调大它。
    double camera_min_dist = 0.5;

    /// 相机最大有效观测距离 (m)：观测点半径上限。
    /// 远于此距离识别率下降。按自瞄有效识别距离设置，建议 3–6。
    /// 调大会让选点有更多候选（遮挡多的地方更容易找到点），但可能站到识别不了的距离。
    double camera_max_dist = 4.0;

    // ======================================================================
    // 可见性 / 安全：决定“哪些点能站、能不能看见”
    // ======================================================================

    /// 观测点到可见扇形边缘的最小角度余量 (rad)。
    /// 选出点后，以同一半径扫描能看见目标的角度区间 [t_l, t_r]，若点离任一边缘 < theta_clearance，
    /// 就推向扇形内部。目的：目标稍微移动时不会立刻被墙角挡住。
    /// - 调大：站位更“居中”，对目标移动更鲁棒；但可能被推到离弹簧点更远的位置，多走路。
    /// - 调小：更靠近弹簧点，路径短；但容易站在墙角边缘，目标一动就丢视线。
    /// - 扇形很窄时自动退化为扇形半宽。建议 0.4–0.8。原版 0.6（debug）/ 0.8。
    double theta_clearance = 0.6;

    /// 观测点自身的 ESDF 安全距离 (m)：观测点必须离障碍至少这么远（且在地图内）。
    /// - 传入未膨胀地图：设为 车体半径 + 余量（如 0.25 + 0.1）。
    /// - 传入已膨胀地图：可设小些（0.05–0.15），避免重复膨胀导致窄通道无点可选。
    /// - 调太大：窄通道、墙边找不到观测点（output.goal 为空）。
    /// - 调太小：goal 贴墙，下游规划器可能规划失败或擦墙。
    double viewpoint_clearance = 0.3;

    /// 视线 ESDF 余量 (m)。
    /// - 0（默认）：视线只要不穿过占据格即可见，使用精确 DDA 栅格遍历，最快。
    /// - >0：视线上每点 ESDF 都要 ≥ 该值才算可见，相当于“视线必须离障碍有一定宽度”，
    ///   用于补偿定位误差 / 目标尺寸，避免擦着墙角看。建议 0~0.2；>0 时改为沿线采样，略慢。
    double los_clearance = 0.0;

    /// 目标本体半径 (m)。mid360 会把目标机器人本身扫成障碍，若不排除，视线永远被目标自己挡住、
    /// 预测也会立刻被“截断”。视线检测与预测截断时忽略以目标当前位置、观测中心为圆心的该半径内的占据。
    /// - 设为 目标车体外接圆半径 + 定位误差（RMUC 步兵/哨兵约 0.3–0.5）。
    /// - 调太小：目标被自己挡住 → 一直判不可见、gimbal 不 release、选点乱跳。
    /// - 调太大：目标旁边的真实障碍也被忽略，可能把隔着矮墙的目标判成可见。
    /// - 若传入的地图已做过膨胀，需要再加上膨胀半径。
    double target_ignore_radius = 0.4;

    /// 观测点角度搜索 / 可见扇形扫描的角度步长 (rad)。
    /// - 调小：选点更精细，但计算量 ∝ 1/step（实测默认值下 update < 0.05 ms，余量很大）。
    /// - 调大：更快但可能漏掉窄的可见缝隙。半径 r 处弧长分辨率 = r·step，建议让它 ≈ 1–2 个栅格：
    ///   r=1.5 m、栅格 0.05 m 时 step≈0.03–0.07。
    double search_angle_step = 0.05;

    // ======================================================================
    // 预测：决定“往哪里看”
    // ======================================================================

    /// 预测时域 (s)。output.prediction 覆盖 [now, now + tracking_dur]。
    /// 第一阶段只用到 lookahead_time 那一个点，时域主要供可视化和第二阶段优化器使用。
    /// 原版 3.0。匀速外推在 >2 s 后误差很大，建议 1.5–3.0。
    double tracking_dur = 2.0;

    /// 预测步长 (s)。prediction 点数 = tracking_dur / tracking_dt + 1。原版 0.2，一般不用改。
    double tracking_dt = 0.2;

    /// 观测中心取预测轨迹上 now + lookahead_time 时刻的点 (s)，应 ≤ tracking_dur。
    /// 含义：机器人走到观测点需要时间，所以对准“将来的目标”而不是“现在的目标”。
    /// - 调大：对快速、直线运动的目标更提前站位；但目标转向时预测错得更离谱，选点会跑偏。
    /// - 调小（→0）：只对准当前位置，稳定但总在目标后面追。
    /// - 经验：≈ 机器人到达新观测点的典型时间（距离变化 / 机器人速度），建议 0.5–1.0；
    ///   目标频繁变向（RMUC 小陀螺、走位）时取 0.3–0.5。
    double lookahead_time = 1.0;

    // ======================================================================
    // 目标估计（2D 匀速卡尔曼滤波）
    // ======================================================================

    /// 过程噪声：目标加速度标准差 (m/s^2)。
    /// - 调大：速度估计跟得快（目标急停/变向时响应快），但更抖。
    /// - 调小：速度平滑，但目标机动时滞后、预测跑偏。
    /// - 建议取目标机器人的典型加速度：RMUC 地面机器人 1.5–4。
    double kf_acc_noise = 2.0;

    /// 观测噪声标准差 (m)，即目标位置检测的误差。
    /// - 按检测源实测：相机解算 0.05–0.2，雷达聚类 0.1–0.3。
    /// - 调大：更信任模型（更平滑、更滞后）；调小：更信任观测（更跟手、更抖）。
    /// - 与 kf_acc_noise 是一对：两者的比值决定平滑程度。
    double kf_meas_noise = 0.1;

    /// 速度门限 (m/s)：更新后估计速度超过该值的观测视为野值丢弃（同原版 checkValid）。
    /// 也用作初始速度方差。设为目标最大速度 × 1.2–1.5，RMUC 建议 3.5–5。
    /// 调太小会把真实的快速目标当野值丢掉，导致一直进 PREDICTING。
    double target_vmax = 4.0;

    /// 新息马氏距离门限（2 自由度 χ²）。观测与预测偏差过大（误检、换目标、检测跳变）时直接拒绝。
    /// - 9.2 ≈ 99%，13.8 ≈ 99.9%（默认），18.4 ≈ 99.99%。≤0 关闭门限。
    /// - 调小：抗误检强，但目标急变向时真实观测也可能被拒（短时进入 PREDICTING）。
    /// - 换目标的情况：新目标的观测被持续拒绝，直到距上次有效观测 > kf_reset_dt 时滤波器重置并接受新目标。
    ///   如果希望换目标更快，调小 kf_reset_dt 或由上层显式调用 TargetTracker::reset()。
    double kf_gate_chi2 = 13.8;

    /// 两次有效观测间隔超过该值则重置滤波器 (s)：长时间没看到，旧速度不可信，用新观测重新初始化。
    /// 建议 0.5–1.5；应 ≤ lost_timeout，否则 SEARCHING 期间重新看到的、离旧预测较远的目标可能被新息门限拒绝。
    double kf_reset_dt = 1.0;

    // ======================================================================
    // 丢失处理：按“距最近一次有效观测的时间 age”切换状态
    //   age < target_timeout                → TRACKING
    //   target_timeout ≤ age < lost_timeout → PREDICTING
    //   lost_timeout   ≤ age < search_timeout → SEARCHING
    //   age ≥ search_timeout                → IDLE
    // 要求 target_timeout < lost_timeout < search_timeout。
    // ======================================================================

    /// 超过该时间没有有效观测 → PREDICTING（按预测继续跟踪，云台不 release）(s)。
    /// 设为观测周期的 3–5 倍：观测 30 Hz 时 0.1–0.2；检测偶尔丢帧多时 0.3–0.5。
    double target_timeout = 0.5;

    /// 超过该时间 → SEARCHING（不再外推，去最后看见的位置附近观察）(s)。
    /// 匀速外推越久越不准，建议 1.0–2.0。
    double lost_timeout = 2.0;

    /// 超过该时间 → IDLE（放弃，goal 为空，调用方应切回其他行为）(s)。按战术需要设置。
    double search_timeout = 8.0;

    // ======================================================================
    // 悬停（hold）与目标点更新：决定“什么时候不动、多久重规划一次”
    // ======================================================================

    /// 目标速度低于该值才允许 hold (m/s)。调大 → 目标慢走时也停着不动（靠云台跟）。
    double hover_target_vel = 0.2;

    /// 机器人速度低于该值才允许 hold (m/s)。防止高速经过平衡点时误判 hold 急停。
    double hover_robot_vel = 0.1;

    /// goal 更新阈值 (m)：新观测点与上次下发的 goal 距离超过该值才置 goal_updated=true。
    /// 用于抑制选点微小抖动导致的频繁重规划。
    /// - 调大：重规划少、运动平稳；但对目标移动响应变慢。
    /// - 调小：响应快，但重规划频繁（FsmReplan 每次重规划都有开销，轨迹也会抖）。
    /// - 建议 0.15–0.4，约为 2–8 个栅格。
    double goal_update_dist = 0.2;

    // ======================================================================
    // 云台（导航下发 yaw，可 360° 连续转）
    // ======================================================================

    /// 云台瞄准超前时间 (s)：瞄准 target_pos + target_vel·gimbal_lead_time，补偿 检测→下发→电机 的链路延迟。
    /// 实测方法：目标匀速横移时，若云台总落后于目标则调大。建议 0.03–0.15。
    double gimbal_lead_time = 0.05;

    /// 是否允许把云台 yaw 释放给视觉自瞄。false 时导航始终控制云台（调试导航指向时可关）。
    bool enable_yaw_release = true;

    /// 观测新鲜度 (s)：最近一次有效观测比该值新才 release（同时要求 TRACKING 且视线畅通）。
    /// 应略大于视觉检测周期，避免自瞄偶尔丢一帧就抢回控制权：30 Hz 检测建议 0.1–0.3。
    /// 调大：自瞄掌控更久（视觉短时丢失也不收回）；调小：更快由导航接管指向预测位置。
    double release_fresh_time = 0.2;
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
    Eigen::Vector2d vel = Eigen::Vector2d::Zero(); ///< 用于 hold 判定与云台角速度前馈
    double yaw = 0.0; ///< 底盘 yaw (rad)；仅在无历史云台角或与目标重合时作为兜底
};

/// 下发给 FsmReplan 的跟踪目标点
struct TrackingGoal {
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();
    double yaw = 0.0; ///< 观测点指向观测中心的朝向 (rad)，可作为底盘参考朝向（云台可 360°，非必须）
};

/// 云台指令（世界系 yaw；云台可 360° 连续转，由调用方换算到云台相对角）
struct GimbalCommand {
    bool release = false; ///< true：导航释放 yaw，交给视觉自瞄；false：导航按 yaw 控制云台
    double yaw = 0.0; ///< 期望云台朝向（世界系，rad，已归一化到 [-pi, pi]；连续转动时调用方负责解卷绕）
    double yaw_rate = 0.0; ///< 视线角速度前馈 (rad/s)，= (r × v_rel)/|r|²
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
    Eigen::Vector2d target_vel = Eigen::Vector2d::Zero(); ///< 当前估计的目标速度（SEARCHING 时为 0）
    std::vector<Eigen::Vector2d> prediction; ///< 从当前时刻开始、步长 tracking_dt 的预测轨迹
    ViewpointInfo viewpoint;
};

} // namespace path_planning::target_tracking

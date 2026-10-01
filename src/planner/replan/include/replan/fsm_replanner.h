#pragma once
#include "map/grid_map.hpp"
#include "path_planning/path_planning.hpp"
#include "traj_optimize/ma_spline_opt/optimizer_config.h"
#include "traj_optimize/ma_spline_opt/traj_optimizer.h"
#include "traj_optimize/grid_map_esdf.hpp"

#include "path_planning/post_processing.h"
#include "utils/expected.hpp"
#include "utils/logger.hpp"
#include "utils/type_utils.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>
namespace replan {
class FsmReplan {
public:
    //重规划触发参数
    struct ReplanParam {
        Eigen::Vector3d goal_deviation;
        //double replan_interval; // 路径最大年龄（s），超过则强制重规划
        double replan_lateral_dev; // 横向偏差阈值（m），机器人偏离参考路径超过则重规划
        double minco_traj_validity_duration; // minco轨迹有效期 (s)，超时则视为无效
        double goal_reached_radius; // 起点≈终点判定半径（m）：机器人距目标小于该值时直接退出不规划，
        // 避免退化路径（A* 单点）与无意义的周期重规划
        //double hard_clearance; // 优化结果硬净空（m）：轨迹采样点净空低于该值直接拒收。
        // 应 ≤ 通道允许的最小净空（≈通道半宽）；太小失去安全网，太大窄缝永远过不去
        double minco_traj_continuity_threshold; ///< 轨迹连续性距离阈值
            ///< (m)，机器人距轨迹投影点小于此值时使用投影点边界
        double projection_search_resolution; ///< 投影点搜索时间分辨率 (s)，越小越精确但计算量越大
    } replan_param;
    struct PlannerConfig {
        ReplanParam replan_params;
        ma_spline_opt::MaSplineOptimizerConfig ma_spline_opt_params;
        path_planning::PathPostProcessing::PathPostProcessingParams path_planning_params;
    } planner_config_;
    //init 1
    FsmReplan() = default;
    auto set_param(const PlannerConfig& planner_config) -> void {
        planner_config_ = planner_config;
        ma_opt_.set_config(planner_config_.ma_spline_opt_params);
        path_planning.set_param(planner_config_.path_planning_params);
    };

    //init 2
    explicit FsmReplan(const PlannerConfig& planner_config) {
        planner_config_ = planner_config;
        path_planning.set_param(planner_config_.path_planning_params);
    }

public:
    enum PathState {
        SUCCESSED,
        FAILED,
        RUNNING,
        IDLE,
    } ;
private:
   PathState  path_state_ = PathState::IDLE;
    bool need_replan_ = true;
    utils::RobotState old_goal_pose_;
    std::vector<Eigen::Vector2d> last_opt_path_;
    struct ResultPath {
        path_planning::PathPostProcessing::Trajectory planning_traj;
        ma_spline_opt::MAsplineOutput ma_spline_traj;
        PathState path_state;
        // true 表示这次返回的是新规划的轨迹
        bool is_new_trajectory = false;
    };
    ResultPath result_;

public:
    enum PathError {
        PLANNING_FAILED,
        MINCO_OPT_FIALED,
        MAX_RETRIES,
    };
    using path = tl::expected<ResultPath, PathError>;
    //ma opt
    auto plan(
        const utils::RobotState& goal_pose,
        const utils::RobotState& current_pose,
        std::shared_ptr<grid_map::GridMap> grid_map
    ) -> path;

public:
    auto one_plan(
        const utils::RobotState& goal_pose,
        const utils::RobotState& current_pose,
        std::shared_ptr<grid_map::GridMap> grid_map
    ) -> path;

private:
    path_planning::PathPlanning path_planning;
    ma_spline_opt::MaSplineTrajectoryOptimizer ma_opt_;
    enum MincoError { OPTFAIL, TIMEDOUT };

private:
    /**replan */
    bool has_valid_trajectory_ = false; ///< 是否有有效的历史轨迹
    std::chrono::system_clock::time_point last_trajectory_time_; ///< 上一条轨迹的发布时间戳
private:
    // 将过于靠近障碍物的点沿 ESDF 梯度外推到安全距离，确保 A*/碰撞检测可通过
    auto get_safe_pos(const Eigen::Vector2d& pos, const grid_map::GridMap& grid_map, const double& safe_threshold)
        -> Eigen::Vector2d;
    auto check_point_equal(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, const Eigen::Vector3d& deviation)
        -> bool;
    auto check_collision(
        path_planning::PathPostProcessing::Trajectory traj,
        const grid_map::GridMap& grid_map,
        const double& safe_threshold
    ) -> bool;
    auto lateral_deviation(const Eigen::Vector2d& pos, const std::vector<Eigen::Vector2d>& path) -> double;

private:
    /**
     * @brief 轨迹投影结果结构体
     *
     * 存储在历史轨迹上查找投影点的结果，包括投影时间、距离、
     * 以及投影点处的位置、速度和加速度信息。
     */
    struct TrajectoryProjectionResult {
        bool valid = false; ///< 投影是否有效
        double projection_time = 0.0; ///< 投影点在轨迹上的时间参数
        double distance = 0.0; ///< 机器人当前位置到投影点的距离
        Eigen::Vector2d position; ///< 投影点位置
        Eigen::Vector2d velocity; ///< 投影点速度
        Eigen::Vector2d acceleration; ///< 投影点加速度 (S=3 热启动)

        TrajectoryProjectionResult():
            position(Eigen::Vector2d::Zero()),
            velocity(Eigen::Vector2d::Zero()),
            acceleration(Eigen::Vector2d::Zero()) {}
    };


};
} // namespace replan

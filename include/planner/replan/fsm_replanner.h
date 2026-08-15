#pragma once
#include "planner/path_planning/path_planning.hpp"
#include "planner/traj_optimize/traj_optimizer.hpp"
#include "planner/path_planning/post_processing.h"
#include "utils/expected.hpp"
#include "utils/logger.hpp"
#include "utils/type_utils.hpp"
#include <optional>
namespace replan {
class FsmReplan {
public:
    //重规划触发参数
    struct ReplanParam {
        Eigen::Vector3d deviation;
        double replan_interval; // 路径最大年龄（s），超过则强制重规划
        double replan_lateral_dev; // 横向偏差阈值（m），机器人偏离参考路径超过则重规划
        double min_replan_interval; // 最小重规划间隔（s），防抖，避免多触发源共振
        double goal_reached_radius; // 起点≈终点判定半径（m）：机器人距目标小于该值时直接退出不规划，
            // 避免退化路径（A* 单点）与无意义的周期重规划
        double hard_clearance; // 优化结果硬净空（m）：轨迹采样点净空低于该值直接拒收。
            // 应 ≤ 通道允许的最小净空（≈通道半宽）；太小失去安全网，太大窄缝永远过不去
    } replan_param;
    struct PlannerConfig {
        Opt::OptimizerParams opt_params;
        ReplanParam replan_params;
        path_planning::PathPostProcessing::PathPostProcessingParams path_planning_params;
    } planner_config_;
    auto set_param(const PlannerConfig& planner_config) -> void {
        planner_config_ = planner_config;
        path_planning.set_param(planner_config_.path_planning_params);
    };
    FsmReplan()=default;
    explicit FsmReplan(const PlannerConfig& planner_config) {
        planner_config_ = planner_config;
        path_planning.set_param(planner_config_.path_planning_params);
    }
    enum PathError {
        NONE,
        SUCCESS,
        FAILED,
    };
    using path = tl::expected<std::pair<path_planning::PathPostProcessing::Trajectory, Opt::EsdfTrajectoryOptimizer>, PathError>;
    auto plan(
        const utils::RobotState& goal_pose,
        const utils::RobotState& current_pose,
        std::shared_ptr<grid_map::GridMap> grid_map
    ) -> path {
        path_planning.set_map(*grid_map);
        Eigen::Vector2d start(current_pose.p.x(), current_pose.p.y());
        Eigen::Vector2d goal(goal_pose.p.x(), goal_pose.p.y());
        //logger::fsm_replan->info("start planning");
        path_planning.set_use_jps(true);
        auto trajectory=path_planning.path_planning(start, goal, 5000);
        if(!trajectory.has_value()){
            logger::fsm_replan->info("planning failed");
            return tl::make_unexpected(PathError::FAILED);
        }
        //logger::fsm_replan->info("end planning");
        return std::make_pair(trajectory.value(), EsdfTrajectoryOptimizer);
    };

private:
    Opt::EsdfTrajectoryOptimizer EsdfTrajectoryOptimizer;
    path_planning::PathPlanning path_planning;
};

}
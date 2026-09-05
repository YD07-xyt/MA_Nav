#pragma once
#include "config.hpp"

//planner

#include "map/ma_map.hpp"
//map

//controller
#include "planner/controller/mpc.h"
//nav
#include "planner/controller/traj_interface.hpp"
#include "planner/replan/fsm_replanner.h"
//#include "planner/fsm/fsm.hpp"

//3rd
#include <Eigen/Core>
//log
#include <spdlog/spdlog.h>
//ros2

#include "utils/plotter.hpp"
#include "utils/type_utils.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "ros2/misc/visualizer.hpp"
//std
#include <optional>
#include <cstddef>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace planner {

class GlobalPlanner2d {
private:
    Config config;
    rclcpp::Node::SharedPtr nh;

private:
    rclcpp::CallbackGroup::SharedPtr map_cb_group_;
    rclcpp::CallbackGroup::SharedPtr planner_cb_group_;
    rclcpp::CallbackGroup::SharedPtr control_cb_group_; // 控制器 + 可视化

    // 保护 ma_map_ 的读写锁（读共享，写独占）
    mutable std::shared_mutex map_mutex_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clickedPoint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clicked_region_pub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_tunnel_regions_srv_;

    // rviz2 点选区域：名字 + 4 个点
    std::vector<geometry_msgs::msg::Point> clicked_points_;
    std::vector<std::string> clicked_region_names_;
    std::vector<std::vector<geometry_msgs::msg::Point>> clicked_region_polygons_;
    // 已生成的历史区域（MarkerArray 整体重发，历史区域保持显示）
    std::vector<visualization_msgs::msg::Marker> clicked_regions_;

    rclcpp::TimerBase::SharedPtr planner_timer_;
    rclcpp::TimerBase::SharedPtr controller_timer_;
    rclcpp::TimerBase::SharedPtr pub_viz_timer_;
    std::optional<utils::RobotState> current_pose = std::nullopt;
    std::optional<utils::RobotState> goal_pose = std::nullopt;
    std::optional<Eigen::Vector3d> current_XYTheta = std::nullopt;
    bool mapInitialized;
    Visualizer visualizer;

private:
    //地图
    std::shared_ptr<grid_map::GridMap> grid_map_;
    std::shared_ptr<ma_map::MaMap> ma_map_;
    //重规划
    replan::FsmReplan fsm_replanner;
    //FSM fsm_;
    tools::Plotter plotter_;
    struct FoldEvent {
        bool fold = true;
        double time = 0.0;
    };
    struct TunnelInterval {
        double entry_time = 0.0;
        double exit_time = 0.0;
    };
    bool is_print = true;
    const float kFoldTime = 1.0; //折叠时间
    const float kUnfoldTime = 1.0; //展开时间
    const float kMarginTime = 0.5;
    std::vector<FoldEvent> fold_events_;
    size_t next_fold_event_idx_ = 0;
    // 当前下发的云台状态：true=折叠，false=抬升
    bool current_fold_state_ = false;
    std::vector<TunnelInterval> detect_tunnel_intervals(
        const ma_spline_opt::MAsplineOutput& ma_traj,
        const grid_map::GridMap& map,
        double sample_dt = 0.02
    );
    std::vector<FoldEvent> generate_fold_events(
        const ma_spline_opt::MAsplineOutput& ma_traj,
        const grid_map::GridMap& map,
        double fold_time,
        double unfold_time,
        double margin,
        double sample_dt = 0.002
    );

private:
    //contorller
    double t_now_;
    Trajectory<5, 2> minco_trajectory_;

private:
    control::Mpc mpc_;
    std::shared_ptr<control::MaSplineTrajectoryInterface> ma_traj_interface_;
    double t_track_ = 0.0;
    // 保存上一帧 MPC 的 world 速度指令，作为下一帧 x0 的速度状态
    double last_vx_world_cmd_ = 0.0;
    double last_vy_world_cmd_ = 0.0;
    bool has_last_mpc_cmd_ = false;

public:
    explicit GlobalPlanner2d(rclcpp::Node::SharedPtr nh_, std::string params_path);
    void map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr& msg);
    void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr& msg);
    void clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr& msg);
    void planner_callback();
    void controller_callback();
    void plan_omni();
    void pub_callback();
    void load_global_map(const std::string& map_path);
    void save_global_map(const std::string& map_dir); // 目录路径，自动生成 .pgm + .yaml
    void save_clicked_regions(const std::string& yaml_path);
    void load_clicked_regions(const std::string& yaml_path);
    void publish_clicked_regions();
};
} // namespace planner
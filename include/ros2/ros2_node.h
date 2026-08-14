#pragma once
#include "config.hpp"


//planner

#include "map/ma_map.hpp"
//map


//controller
#include "planner/controller/omni_lmpc.hpp"
//gcopter
#include "planner/fsm/fsm.hpp"


//3rd
#include <Eigen/Core>
//log
#include <spdlog/spdlog.h>
//ros2 
#include "ros2/misc/visualizer.hpp"
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
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapSub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clickedPointSub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr OdomSub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clicked_region_pub_;

  // rviz2 点选点缓冲：每 4 个点连成一个封闭四边形区域
  std::vector<geometry_msgs::msg::Point> clicked_points_;
  // 已生成的历史区域（MarkerArray 整体重发，历史区域保持显示）
  std::vector<visualization_msgs::msg::Marker> clicked_regions_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;

  rclcpp::TimerBase::SharedPtr planner_timer_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
  rclcpp::TimerBase::SharedPtr pub_viz_timer_;
  std::optional<utils::RobotState> current_pose = std::nullopt;
  std::optional<utils::RobotState> goal_pose = std::nullopt;
  std::optional<Eigen::Vector3d> current_XYTheta=std::nullopt;
  bool mapInitialized;
  Visualizer visualizer;
  
private:
  //地图
  std::shared_ptr<grid_map::GridMap> grid_map_;
  std::shared_ptr<ma_map::MaMap> ma_map_;
  //重规划
  FSM fsm_;
  tools::Plotter plotter_;
private:
  //contorller
  controller::LMpc omni_lmpc_;
  double t_now_;
  SplineTrajectory::PPolyND<2, 6> trajectory_;
  std::chrono::steady_clock::time_point start_time_;
public:
  explicit GlobalPlanner2d(rclcpp::Node::SharedPtr nh_);
  void map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr &msg);
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr &msg);
  void clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr &msg);
  void planner_callback();
  void controller_callback();
  void plan_omni();
  void pub_callback();
  void load_global_map(const std::string& map_path);
  void save_global_map(const std::string& map_dir);  // 目录路径，自动生成 .pgm + .yaml
};
} // namespace planner
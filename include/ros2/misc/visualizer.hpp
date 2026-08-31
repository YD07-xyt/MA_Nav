#pragma once
#include "map/grid_map.hpp"
#include "map/ma_map.hpp"
// #include "planner/opt/traj_optimizer.hpp"
#include "planner/traj_optimize/ma_spline_opt/SplineTrajectory/SplineTrajectory.hpp"
#include "planner/traj_optimize/ma_spline_opt/optimizer_config.h"
#include "planner/traj_optimize/minco_opt/gcopter/trajectory.hpp"
#include "utils/eigen_alias.hpp"
#include "utils/color_msg_utils.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <spdlog/spdlog.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "pcl_conversions/pcl_conversions.h"
// Visualizer for the planner
class Visualizer {
private:
    // config contains the scale for some markers
    rclcpp::Node::SharedPtr node;

    // These are publishers for path, waypoints on the trajectory,
    // the entire trajectory, the mesh of free-space polytopes,
    // the edge of free-space polytopes, and spheres for safety radius
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr routePub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr yaw_profile_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wayPointsPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectoryPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr meshPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr edgePub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spherePub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr Mappub; // Mappub
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_occ_pub; // 占用栅格 OccupancyGrid
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_esdf_pub; // 2D ESDF OccupancyGrid
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr SurfMapPub; // SurfMappub
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr GlobalPathPub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr OptPathPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occ_pub, unknown_pub, occ_inf_pub, unknown_inf_pub,
        frontier_pub, esdf_pub, terrain_map_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mkr_arr_pub;

public:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speedPub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrPub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tiltPub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bdrPub;

public:
    Visualizer(rclcpp::Node::SharedPtr node): node(node) {
        routePub = node->create_publisher<visualization_msgs::msg::Marker>("/ma_nav/route", 10);
        yaw_profile_pub = node->create_publisher<visualization_msgs::msg::Marker>("/ma_nav/yaw_profile", 10);
        wayPointsPub = node->create_publisher<visualization_msgs::msg::Marker>("/ma_nav/waypoints", 10);
        trajectoryPub = node->create_publisher<visualization_msgs::msg::Marker>("/ma_nav/trajectory", 10);
        meshPub = node->create_publisher<visualization_msgs::msg::Marker>("/ma_nav/mesh", 10);
        edgePub = node->create_publisher<visualization_msgs::msg::Marker>("/ma_nav/edge", 10);
        spherePub = node->create_publisher<visualization_msgs::msg::Marker>("/ma_nav/spheres", 10);
        speedPub = node->create_publisher<std_msgs::msg::Float64>("/ma_nav/speed", 10);
        thrPub = node->create_publisher<std_msgs::msg::Float64>("/ma_nav/total_thrust", 10);
        tiltPub = node->create_publisher<std_msgs::msg::Float64>("/ma_nav/tilt_angle", 10);
        bdrPub = node->create_publisher<std_msgs::msg::Float64>("/ma_nav/body_rate", 10);
        Mappub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ma_nav/grid_map", 10);
        grid_occ_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/ma_nav/grid_map_occ", 10);
        grid_esdf_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/ma_nav/grid_map_esdf", 10);
        SurfMapPub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ma_nav/surf_map", 10);
        GlobalPathPub = node->create_publisher<nav_msgs::msg::Path>("/ma_nav/global_path", 10);
        OptPathPub = node->create_publisher<nav_msgs::msg::Path>("/ma_nav/opt_path", 10);
        occ_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ma_nav/map/occ", 10);
        unknown_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ma_nav/map/unk", 10);
        occ_inf_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ma_nav/map/inf_occ", 10);
        unknown_inf_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ma_nav/map/inf_unk", 10);
        esdf_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ma_nav/map/esdf", 10);
        frontier_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ma_nav/map/frontier", 10);
        mkr_arr_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("/ma_nav/map/map_bound", 10);
    }

    inline void
    visualizeYaw(const SplineTrajectory::QuinticSplineND<1>& yaw_spline, const std::string& frame_id = "world") {
        if (!yaw_spline.isInitialized() || yaw_spline.getNumSegments() <= 0) return;

        visualization_msgs::msg::Marker marker;
        marker.header.stamp = node->now();
        marker.header.frame_id = frame_id;
        marker.ns = "yaw_profile";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05;
        marker.color.r = 1.0;
        marker.color.g = 0.8;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        const double start_time = yaw_spline.getStartTime();
        const double end_time = yaw_spline.getEndTime();
        const double dt = 0.05;

        for (double t = start_time; t <= end_time + 1e-6; t += dt) {
            double yaw = yaw_spline.getTrajectory().evaluate(t, 0)(0);

            geometry_msgs::msg::Point p;
            p.x = t;
            p.y = yaw;
            p.z = 0.0;
            marker.points.push_back(p);
        }

        yaw_profile_pub->publish(marker);
    }
    // 可视化 SplineTrajectory::QuinticSplineND<D>
    // D=2: (x,y)
    // D=3: 只画前两维 (x,y)，第三维如果是 yaw 不作为 z 使用
    template<int D>
    inline void visualizeSpline(
        const SplineTrajectory::QuinticSplineND<D>& traj,
        const std::vector<Eigen::Vector3d>& route,
        const std::string& frame_id = "world"
    ) {
        visualization_msgs::msg::Marker routeMarker, wayPointsMarker, trajMarker;

        // ===================== 公共基础 =====================
        routeMarker.header.stamp = node->now();
        routeMarker.header.frame_id = frame_id;
        routeMarker.pose.orientation.w = 1.0;
        routeMarker.action = visualization_msgs::msg::Marker::ADD;

        // ===================== 前端路径 route =====================
        routeMarker.id = 0;
        routeMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.0;
        routeMarker.color.g = 0.0;
        routeMarker.color.b = 0.0;
        routeMarker.color.a = 1.0;
        routeMarker.scale.x = 0.1;

        if (route.size() > 0) {
            bool first = true;
            Eigen::Vector3d last;
            for (const auto& it: route) {
                if (first) {
                    first = false;
                    last = it;
                    continue;
                }

                geometry_msgs::msg::Point p;
                p.x = last(0);
                p.y = last(1);
                p.z = last(2);
                routeMarker.points.push_back(p);

                p.x = it(0);
                p.y = it(1);
                p.z = it(2);
                routeMarker.points.push_back(p);

                last = it;
            }

            routePub->publish(routeMarker);
        }

        // ===================== 样条路点 =====================
        wayPointsMarker = routeMarker;
        wayPointsMarker.points.clear(); 
        wayPointsMarker.id = -1;
        wayPointsMarker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 1.0;
        wayPointsMarker.color.g = 0.0;
        wayPointsMarker.color.b = 0.0;
        wayPointsMarker.scale.x = 0.35;
        wayPointsMarker.scale.y = 0.35;
        wayPointsMarker.scale.z = 0.35;

        if (traj.isInitialized()) {
            const auto& wps = traj.getSpacePoints();

            for (int i = 0; i < wps.rows(); ++i) {
                geometry_msgs::msg::Point p;
                p.x = wps(i, 0);
                p.y = wps(i, 1);
                p.z = 0.0;

                wayPointsMarker.points.push_back(p);
            }

            wayPointsPub->publish(wayPointsMarker);
        }

        // ===================== 样条轨迹 =====================
        trajMarker = routeMarker;
        trajMarker.points.clear(); // ✅ 关键修复
        trajMarker.id = 0;
        trajMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
        trajMarker.ns = "trajectory";
        trajMarker.color.r = 0.0;
        trajMarker.color.g = 0.5;
        trajMarker.color.b = 1.0;
        trajMarker.scale.x = 0.3;

        if (traj.isInitialized() && traj.getNumSegments() > 0) {
            const double start_time = traj.getStartTime();
            const double end_time = traj.getEndTime();
            const double dt = 0.05;

            Eigen::Matrix<double, D, 1> last_pos = traj.getTrajectory().evaluate(start_time, 0);

            for (double t = start_time + dt; t <= end_time + 1e-6; t += dt) {
                Eigen::Matrix<double, D, 1> pos = traj.getTrajectory().evaluate(t, 0);

                geometry_msgs::msg::Point p;
                p.x = last_pos(0);
                p.y = last_pos(1);
                p.z = 0.0;
                trajMarker.points.push_back(p);

                p.x = pos(0);
                p.y = pos(1);
                p.z = 0.0;
                trajMarker.points.push_back(p);

                last_pos = pos;
            }

            trajectoryPub->publish(trajMarker);
        }
    }
    inline void visualize(
        const ma_spline_opt::MAsplineOutput& output,
        const std::vector<Eigen::Vector3d>& route,
        const std::string& frame_id = "world"
    ) {
        if (output.success && output.trajectory.isInitialized()) {
            visualizeSpline(output.trajectory, route, frame_id);
        }

        // if (output.trajectory.isInitialized()) {
        //     visualizeYaw(output.yaw_spline, frame_id);
        // }
    }
    // Visualize the trajectory and its front-end path
    template<int D>
    inline void visualize(const Trajectory<D, 2>& traj, const std::vector<Eigen::Vector3d>& route) {
        visualization_msgs::msg::Marker routeMarker, wayPointsMarker, trajMarker;

        routeMarker.id = 0;
        routeMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
        routeMarker.header.stamp = node->now();
        routeMarker.header.frame_id = "world";
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::msg::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.1;

        wayPointsMarker = routeMarker;
        wayPointsMarker.id = -wayPointsMarker.id - 1;
        wayPointsMarker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 1.00;
        wayPointsMarker.color.g = 0.00;
        wayPointsMarker.color.b = 0.00;
        wayPointsMarker.scale.x = 0.35;
        wayPointsMarker.scale.y = 0.35;
        wayPointsMarker.scale.z = 0.35;

        trajMarker = routeMarker;
        trajMarker.header.frame_id = "world";
        trajMarker.id = 0;
        trajMarker.ns = "trajectory";
        trajMarker.color.r = 0.00;
        trajMarker.color.g = 0.50;
        trajMarker.color.b = 1.00;
        trajMarker.scale.x = 0.30;

        if (route.size() > 0) {
            bool first = true;
            Eigen::Vector3d last;
            for (auto it: route) {
                if (first) {
                    first = false;
                    last = it;
                    continue;
                }
                geometry_msgs::msg::Point point;

                point.x = last(0);
                point.y = last(1);
                point.z = last(2);
                routeMarker.points.push_back(point);
                point.x = it(0);
                point.y = it(1);
                point.z = it(2);
                routeMarker.points.push_back(point);
                last = it;
            }

            routePub->publish(routeMarker);
        }

        if (traj.getPieceNum() > 0) {
            Eigen::MatrixXd wps = traj.getPositions();
            for (int i = 0; i < wps.cols(); i++) {
                geometry_msgs::msg::Point point;
                point.x = wps.col(i)(0);
                point.y = wps.col(i)(1);
                point.z = (wps.rows() > 2) ? wps.col(i)(2) : 0.0;
                wayPointsMarker.points.push_back(point);
            }

            wayPointsPub->publish(wayPointsMarker);
        }

        if (traj.getPieceNum() > 0) {
            double T = 0.01;
            Eigen::Vector3d lastX = traj.getPos(0.0);
            for (double t = T; t < traj.getTotalDuration(); t += T) {
                geometry_msgs::msg::Point point;
                Eigen::Vector3d X = traj.getPos(t);
                point.x = lastX(0);
                point.y = lastX(1);
                point.z = lastX(2);
                trajMarker.points.push_back(point);
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                trajMarker.points.push_back(point);
                lastX = X;
            }
            trajectoryPub->publish(trajMarker);
        }
    }

    void PubWayPoints(SplineTrajectory::PPolyND<2, 6> traj) {
        // 1. 航点
        visualization_msgs::msg::Marker waypoints_marker;
        waypoints_marker.header.frame_id = "world";
        waypoints_marker.header.stamp = this->node->now();
        waypoints_marker.ns = "waypoints";
        waypoints_marker.id = 0;
        waypoints_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        waypoints_marker.action = visualization_msgs::msg::Marker::ADD;
        waypoints_marker.scale.x = 0.35;
        waypoints_marker.scale.y = 0.35;
        waypoints_marker.scale.z = 0.35;
        waypoints_marker.color.r = 1.0;
        waypoints_marker.color.g = 0.0;
        waypoints_marker.color.b = 0.0;
        waypoints_marker.color.a = 1.0;

        const auto& breakpoints = traj.getBreakpoints();
        for (double t: breakpoints) {
            Eigen::Vector2d pos = traj.evaluate(t, 0);
            geometry_msgs::msg::Point p;
            p.x = pos.x();
            p.y = pos.y();
            p.z = 0.0;
            waypoints_marker.points.push_back(p);
        }
        wayPointsPub->publish(waypoints_marker);
    };
    void PubTrajectory(SplineTrajectory::PPolyND<2, 6> traj, std::vector<Eigen::Vector2d> dense_path) {
        visualization_msgs::msg::Marker traj_marker;
        traj_marker.header.frame_id = "world";
        traj_marker.header.stamp = node->now();
        traj_marker.ns = "trajectory";
        traj_marker.id = 0;
        traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        traj_marker.action = visualization_msgs::msg::Marker::ADD;
        traj_marker.scale.x = 0.3;
        traj_marker.color.r = 0.0;
        traj_marker.color.g = 0.5;
        traj_marker.color.b = 1.0;
        traj_marker.color.a = 1.0;

        for (const auto& pt: dense_path) {
            geometry_msgs::msg::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = 0.0;
            traj_marker.points.push_back(p);
        }
        trajectoryPub->publish(traj_marker);
    }
    void PubOptPath(std::vector<Eigen::Vector2d>& path) {
        nav_msgs::msg::Path nav_path;
        nav_path.header.frame_id = "world";
        nav_path.header.stamp = rclcpp::Clock().now(); // 或使用 node->get_clock()->now()

        for (const auto& pt: path) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = nav_path.header; // 使用相同的帧ID和时间戳
            pose_stamped.pose.position.x = pt.x();
            pose_stamped.pose.position.y = pt.y();
            pose_stamped.pose.position.z = 0.0;
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0; // 单位四元数（无旋转）

            nav_path.poses.push_back(pose_stamped);
        }

        OptPathPub->publish(nav_path);
    };
    void PubGlobalPath(std::vector<Eigen::Vector2d>& path) {
        nav_msgs::msg::Path nav_path;
        nav_path.header.frame_id = "world";
        nav_path.header.stamp = rclcpp::Clock().now(); // 或使用 node->get_clock()->now()

        for (const auto& pt: path) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = nav_path.header; // 使用相同的帧ID和时间戳
            pose_stamped.pose.position.x = pt.x();
            pose_stamped.pose.position.y = pt.y();
            pose_stamped.pose.position.z = 0.0;
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0; // 单位四元数（无旋转）

            nav_path.poses.push_back(pose_stamped);
        }

        GlobalPathPub->publish(nav_path);
    };

    // // Visualize the trajectory and its front-end path
    // template<int D>
    // inline void visualize(const Trajectory<D>& traj, const std::vector<Eigen::Vector3d>& route) {
    //     visualization_msgs::msg::Marker routeMarker, wayPointsMarker, trajMarker;

    //     routeMarker.id = 0;
    //     routeMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
    //     routeMarker.header.stamp = node->now();
    //     routeMarker.header.frame_id = "world";
    //     routeMarker.pose.orientation.w = 1.00;
    //     routeMarker.action = visualization_msgs::msg::Marker::ADD;
    //     routeMarker.ns = "route";
    //     routeMarker.color.r = 1.00;
    //     routeMarker.color.g = 0.00;
    //     routeMarker.color.b = 0.00;
    //     routeMarker.color.a = 1.00;
    //     routeMarker.scale.x = 0.1;

    //     wayPointsMarker = routeMarker;
    //     wayPointsMarker.id = -wayPointsMarker.id - 1;
    //     wayPointsMarker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    //     wayPointsMarker.ns = "waypoints";
    //     wayPointsMarker.color.r = 1.00;
    //     wayPointsMarker.color.g = 0.00;
    //     wayPointsMarker.color.b = 0.00;
    //     wayPointsMarker.scale.x = 0.35;
    //     wayPointsMarker.scale.y = 0.35;
    //     wayPointsMarker.scale.z = 0.35;

    //     trajMarker = routeMarker;
    //     trajMarker.header.frame_id = "world";
    //     trajMarker.id = 0;
    //     trajMarker.ns = "trajectory";
    //     trajMarker.color.r = 0.00;
    //     trajMarker.color.g = 0.50;
    //     trajMarker.color.b = 1.00;
    //     trajMarker.scale.x = 0.30;

    //     if (route.size() > 0) {
    //         bool first = true;
    //         Eigen::Vector3d last;
    //         for (auto it: route) {
    //             if (first) {
    //                 first = false;
    //                 last = it;
    //                 continue;
    //             }
    //             geometry_msgs::msg::Point point;

    //             point.x = last(0);
    //             point.y = last(1);
    //             point.z = last(2);
    //             routeMarker.points.push_back(point);
    //             point.x = it(0);
    //             point.y = it(1);
    //             point.z = it(2);
    //             routeMarker.points.push_back(point);
    //             last = it;
    //         }

    //         routePub->publish(routeMarker);
    //     }

    //     if (traj.getPieceNum() > 0) {
    //         Eigen::MatrixXd wps = traj.getPositions();
    //         for (int i = 0; i < wps.cols(); i++) {
    //             geometry_msgs::msg::Point point;
    //             point.x = wps.col(i)(0);
    //             point.y = wps.col(i)(1);
    //             point.z = wps.col(i)(2);
    //             wayPointsMarker.points.push_back(point);
    //         }

    //         wayPointsPub->publish(wayPointsMarker);
    //     }

    //     if (traj.getPieceNum() > 0) {
    //         double T = 0.01;
    //         Eigen::Vector3d lastX = traj.getPos(0.0);
    //         for (double t = T; t < traj.getTotalDuration(); t += T) {
    //             geometry_msgs::msg::Point point;
    //             Eigen::Vector3d X = traj.getPos(t);
    //             point.x = lastX(0);
    //             point.y = lastX(1);
    //             point.z = lastX(2);
    //             trajMarker.points.push_back(point);
    //             point.x = X(0);
    //             point.y = X(1);
    //             point.z = X(2);
    //             trajMarker.points.push_back(point);
    //             lastX = X;
    //         }
    //         // RCLCPP_INFO(node->get_logger(), "Visualizing trajectory with %d
    //         // points", (int)trajMarker.points.size());
    //         trajectoryPub->publish(trajMarker);
    //     }
    // }

    inline void visualizeMap(const std::vector<Eigen::Vector3d>& map, const std::string& frame_id = "world") {
        if (!Mappub) {
            spdlog::warn("map pub  empty");
            return;
        }
        if (map.empty()) {
            // spdlog::warn("map empty");
            return;
        }
        // 创建 PointCloud2 消息
        sensor_msgs::msg::PointCloud2 pcl_msg;

        // 设置时间戳和坐标系
        pcl_msg.header.stamp = rclcpp::Clock().now();
        pcl_msg.header.frame_id = frame_id;

        // 设置点云属性
        pcl_msg.height = 1; // 无序点云
        pcl_msg.width = map.size();
        pcl_msg.is_dense = true; // 没有无效点
        pcl_msg.is_bigendian = false;

        // 定义点字段 (x, y, z)
        pcl_msg.fields.resize(3);

        // x 字段
        pcl_msg.fields[0].name = "x";
        pcl_msg.fields[0].offset = 0;
        pcl_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pcl_msg.fields[0].count = 1;

        // y 字段
        pcl_msg.fields[1].name = "y";
        pcl_msg.fields[1].offset = 4;
        pcl_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pcl_msg.fields[1].count = 1;

        // z 字段
        pcl_msg.fields[2].name = "z";
        pcl_msg.fields[2].offset = 8;
        pcl_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pcl_msg.fields[2].count = 1;

        // 计算点步长 (3个float = 12字节)
        pcl_msg.point_step = 12;
        pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;

        // 分配数据空间
        pcl_msg.data.resize(pcl_msg.row_step * pcl_msg.height);

        // 填充数据
        for (size_t i = 0; i < map.size(); ++i) {
            float x = static_cast<float>(map[i].x());
            float y = static_cast<float>(map[i].y());
            float z = static_cast<float>(map[i].z());

            memcpy(&pcl_msg.data[i * pcl_msg.point_step + 0], &x, sizeof(float));
            memcpy(&pcl_msg.data[i * pcl_msg.point_step + 4], &y, sizeof(float));
            memcpy(&pcl_msg.data[i * pcl_msg.point_step + 8], &z, sizeof(float));
        }

        // 发布消息
        Mappub->publish(pcl_msg);
    }
    inline void visualize_occupied_map(const grid_map::GridMap& grid_map, const std::string& frame_id = "world") {
        if (!Mappub) return;

        Eigen::Vector2i voxel_num = grid_map.getVoxelNum();
        // 先统计障碍物数量，以便预留空间
        std::vector<Eigen::Vector2i> occ_cells;
        for (int x = 0; x < voxel_num.x(); ++x) {
            for (int y = 0; y < voxel_num.y(); ++y) {
                if (grid_map.isOccupied(Eigen::Vector2i(x, y))) {
                    occ_cells.emplace_back(x, y);
                }
            }
        }

        sensor_msgs::msg::PointCloud2 pcl_msg;
        pcl_msg.header.stamp = rclcpp::Clock().now();
        pcl_msg.header.frame_id = frame_id;
        pcl_msg.height = 1;
        pcl_msg.width = occ_cells.size();
        pcl_msg.is_dense = true;
        pcl_msg.is_bigendian = false;

        pcl_msg.fields.resize(3);
        pcl_msg.fields[0].name = "x";
        pcl_msg.fields[0].offset = 0;
        pcl_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pcl_msg.fields[0].count = 1;
        pcl_msg.fields[1].name = "y";
        pcl_msg.fields[1].offset = 4;
        pcl_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pcl_msg.fields[1].count = 1;
        pcl_msg.fields[2].name = "z";
        pcl_msg.fields[2].offset = 8;
        pcl_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pcl_msg.fields[2].count = 1;

        pcl_msg.point_step = 12;
        pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
        pcl_msg.data.resize(pcl_msg.row_step);

        int idx = 0;
        for (const auto& id: occ_cells) {
            Eigen::Vector2d pos;
            grid_map.indexToPos(id, pos);
            float px = static_cast<float>(pos.x());
            float py = static_cast<float>(pos.y());
            float pz = 0.0f; // 障碍物点位于平面 z=0
            memcpy(&pcl_msg.data[idx * pcl_msg.point_step + 0], &px, sizeof(float));
            memcpy(&pcl_msg.data[idx * pcl_msg.point_step + 4], &py, sizeof(float));
            memcpy(&pcl_msg.data[idx * pcl_msg.point_step + 8], &pz, sizeof(float));
            ++idx;
        }

        Mappub->publish(pcl_msg);
    }
    // 将 GridMap 占用栅格发布为 nav_msgs/OccupancyGrid（rviz2 直接显示二维地图）
    inline void visualize_occupied_grid(const grid_map::GridMap& grid_map, const std::string& frame_id = "world") {
        if (!grid_occ_pub) return;

        const Eigen::Vector2i voxel_num = grid_map.getVoxelNum();
        const double res = grid_map.getResolution();
        const Eigen::Vector2d origin = grid_map.getOrigin();

        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header.stamp = node->now();
        grid_msg.header.frame_id = frame_id;
        grid_msg.info.resolution = res;
        grid_msg.info.width = voxel_num.x();
        grid_msg.info.height = voxel_num.y();
        grid_msg.info.origin.position.x = origin.x();
        grid_msg.info.origin.position.y = origin.y();
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;
        grid_msg.data.resize(voxel_num.x() * voxel_num.y(), 0);

        // OccupancyGrid 数据为行主序：data[y*width+x]，(0,0) 位于 origin 角点
        for (int x = 0; x < voxel_num.x(); ++x) {
            for (int y = 0; y < voxel_num.y(); ++y) {
                grid_msg.data[y * voxel_num.x() + x] = grid_map.isOccupied(Eigen::Vector2i(x, y)) ? 100 : 0;
            }
        }
        grid_occ_pub->publish(grid_msg);
    }
    // 将 GridMap 内部 2D ESDF 距离场发布为 nav_msgs/OccupancyGrid（灰度=距离，障碍内部为 0）
    inline void visualize_esdf_grid(const grid_map::GridMap& grid_map, const std::string& frame_id = "world") {
        if (!grid_esdf_pub) return;

        const Eigen::Vector2i voxel_num = grid_map.getVoxelNum();
        const double res = grid_map.getResolution();
        const Eigen::Vector2d origin = grid_map.getOrigin();
        const grid_map::RowMatrixXd esdf = grid_map.getMap(); // (x, y) 索引

        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header.stamp = node->now();
        grid_msg.header.frame_id = frame_id;
        grid_msg.info.resolution = res;
        grid_msg.info.width = voxel_num.x();
        grid_msg.info.height = voxel_num.y();
        grid_msg.info.origin.position.x = origin.x();
        grid_msg.info.origin.position.y = origin.y();
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;
        grid_msg.data.resize(voxel_num.x() * voxel_num.y(), 0);

        // 距离缩放：1m -> 10，饱和 100（10m）；障碍内部（d<0）按 0 显示（黑色）
        for (int x = 0; x < voxel_num.x(); ++x) {
            for (int y = 0; y < voxel_num.y(); ++y) {
                double d = esdf(x, y);
                double scaled = (d < 0.0) ? 0.0 : d * 30.0;
                if (scaled > 100.0) scaled = 100.0;
                grid_msg.data[y * voxel_num.x() + x] = static_cast<int8_t>(scaled);
            }
        }
        grid_esdf_pub->publish(grid_msg);
    }
    inline void visualizeSurfMap(const std::vector<Eigen::Vector3d>& surf_map, const std::string& frame_id = "world") {
        if (!SurfMapPub) {
            spdlog::warn("map pub  empty");
            return;
        }
        if (surf_map.empty()) {
            spdlog::warn("surf_map empty");
            return;
        }

        // 创建 PointCloud2 消息
        sensor_msgs::msg::PointCloud2 pcl_msg;

        // 设置时间戳和坐标系
        pcl_msg.header.stamp = rclcpp::Clock().now();
        pcl_msg.header.frame_id = frame_id;

        // 设置点云属性
        pcl_msg.height = 1; // 无序点云
        pcl_msg.width = surf_map.size();
        pcl_msg.is_dense = true; // 没有无效点
        pcl_msg.is_bigendian = false;

        // 定义点字段 (x, y, z)
        pcl_msg.fields.resize(3);

        // x 字段
        pcl_msg.fields[0].name = "x";
        pcl_msg.fields[0].offset = 0;
        pcl_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pcl_msg.fields[0].count = 1;

        // y 字段
        pcl_msg.fields[1].name = "y";
        pcl_msg.fields[1].offset = 4;
        pcl_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pcl_msg.fields[1].count = 1;

        // z 字段
        pcl_msg.fields[2].name = "z";
        pcl_msg.fields[2].offset = 8;
        pcl_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pcl_msg.fields[2].count = 1;

        // 计算点步长 (3个float = 12字节)
        pcl_msg.point_step = 12;
        pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;

        // 分配数据空间
        pcl_msg.data.resize(pcl_msg.row_step * pcl_msg.height);

        // 填充数据
        for (size_t i = 0; i < surf_map.size(); ++i) {
            float x = static_cast<float>(surf_map[i].x());
            float y = static_cast<float>(surf_map[i].y());
            float z = static_cast<float>(surf_map[i].z());

            memcpy(&pcl_msg.data[i * pcl_msg.point_step + 0], &x, sizeof(float));
            memcpy(&pcl_msg.data[i * pcl_msg.point_step + 4], &y, sizeof(float));
            memcpy(&pcl_msg.data[i * pcl_msg.point_step + 8], &z, sizeof(float));
        }

        // 发布消息
        SurfMapPub->publish(pcl_msg);
    }
    // // Visualize some polytopes in H-representation
    // inline void visualizePolytope(const std::vector<Eigen::MatrixX4d>& hPolys) {
    //     // Due to the fact that H-representation cannot be directly visualized
    //     // We first conduct vertex enumeration of them, then apply quickhull
    //     // to obtain triangle meshs of polyhedra
    //     Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);
    //     for (size_t id = 0; id < hPolys.size(); id++) {
    //         oldTris = mesh;
    //         Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
    //         geo_utils::enumerateVs(hPolys[id], vPoly);

    //         quickhull::QuickHull<double> tinyQH;
    //         const auto polyHull = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
    //         const auto& idxBuffer = polyHull.getIndexBuffer();
    //         int hNum = idxBuffer.size() / 3;

    //         curTris.resize(3, hNum * 3);
    //         for (int i = 0; i < hNum * 3; i++) {
    //             curTris.col(i) = vPoly.col(idxBuffer[i]);
    //         }
    //         mesh.resize(3, oldTris.cols() + curTris.cols());
    //         mesh.leftCols(oldTris.cols()) = oldTris;
    //         mesh.rightCols(curTris.cols()) = curTris;
    //     }

    //     // RVIZ support tris for visualization
    //     visualization_msgs::msg::Marker meshMarker, edgeMarker;

    //     meshMarker.id = 0;
    //     meshMarker.header.stamp = node->now();
    //     meshMarker.header.frame_id = "world";
    //     meshMarker.pose.orientation.w = 1.00;
    //     meshMarker.action = visualization_msgs::msg::Marker::ADD;
    //     meshMarker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    //     meshMarker.ns = "mesh";
    //     meshMarker.color.r = 0.00;
    //     meshMarker.color.g = 0.00;
    //     meshMarker.color.b = 1.00;
    //     meshMarker.color.a = 0.15;
    //     meshMarker.scale.x = 1.0;
    //     meshMarker.scale.y = 1.0;
    //     meshMarker.scale.z = 1.0;

    //     edgeMarker = meshMarker;
    //     edgeMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
    //     edgeMarker.ns = "edge";
    //     edgeMarker.color.r = 0.00;
    //     edgeMarker.color.g = 1.00;
    //     edgeMarker.color.b = 1.00;
    //     edgeMarker.color.a = 1.00;
    //     edgeMarker.scale.x = 0.02;

    //     geometry_msgs::msg::Point point;

    //     int ptnum = mesh.cols();

    //     for (int i = 0; i < ptnum; i++) {
    //         point.x = mesh(0, i);
    //         point.y = mesh(1, i);
    //         point.z = mesh(2, i);
    //         meshMarker.points.push_back(point);
    //     }

    //     for (int i = 0; i < ptnum / 3; i++) {
    //         for (int j = 0; j < 3; j++) {
    //             point.x = mesh(0, 3 * i + j);
    //             point.y = mesh(1, 3 * i + j);
    //             point.z = mesh(2, 3 * i + j);
    //             edgeMarker.points.push_back(point);
    //             point.x = mesh(0, 3 * i + (j + 1) % 3);
    //             point.y = mesh(1, 3 * i + (j + 1) % 3);
    //             point.z = mesh(2, 3 * i + (j + 1) % 3);
    //             edgeMarker.points.push_back(point);
    //         }
    //     }
    //     // RCLCPP_INFO(node->get_logger(), "Visualizing mesh with %d points",
    //     // ptnum);
    //     meshPub->publish(meshMarker);
    //     edgePub->publish(edgeMarker);

    //     return;
    // }

    // Visualize all spheres with centers sphs and the same radius
    inline void visualizeSphere(const Eigen::Vector3d& center, const double& radius) {
        visualization_msgs::msg::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = node->now();
        sphereMarkers.header.frame_id = "world";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::msg::Marker::ADD;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 0.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 1.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::msg::Marker::DELETE;

        geometry_msgs::msg::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);
        // RCLCPP_INFO(node->get_logger(), "Visualizing sphere with center (%.2f,
        // %.2f, %.2f) and radius %.2f", center(0), center(1), center(2), radius);
        spherePub->publish(sphereDeleter);
        spherePub->publish(sphereMarkers);
    }

    inline void visualizeStartGoal(const Eigen::Vector3d& center, const double& radius, const int sg) {
        ;
        // RCLCPP_INFO(node->get_logger(), "Visualizing start/goal point with
        // position (%.2f, %.2f, %.2f)", center(0), center(1), center(2));
        visualization_msgs::msg::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = sg;
        sphereMarkers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = node->now();
        sphereMarkers.header.frame_id = "world";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::msg::Marker::ADD;
        sphereMarkers.ns = "StartGoal";
        sphereMarkers.color.r = 1.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 0.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::msg::Marker::DELETEALL;

        geometry_msgs::msg::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        if (sg == 0) {
            spherePub->publish(sphereDeleter);
            // TODO:何意味
            std::this_thread::sleep_for(std::chrono::nanoseconds(1));
            sphereMarkers.header.stamp = node->now();
        }
        spherePub->publish(sphereMarkers);
    }

    static void visualizeBoundingBox(
        visualization_msgs::msg::MarkerArray& mkrarr,
        const double& stamp,
        const utils::Vec3f& box_min,
        const utils::Vec3f& box_max,
        const std::string& ns,
        const utils::Color& color,
        const double& size_x = 0.1,
        const double& alpha = 1.0,
        const bool& print_ns = true
    ) {
        utils::Vec3f size = (box_max - box_min) / 2;
        utils::Vec3f vis_pos_world = (box_min + box_max) / 2;
        double width = size.x();
        double length = size.y();
        double hight = size.z();

        // Publish Bounding box
        int id = 0;
        visualization_msgs::msg::Marker line_strip;
        line_strip.header.stamp = rclcpp::Time(stamp);
        line_strip.header.frame_id = "world";
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.ns = ns;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = id++; // unique id, useful when multiple markers exist.
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP; // marker
            // type
        line_strip.scale.x = size_x;

        line_strip.color = color;
        line_strip.color.a = alpha; //不透明度，设0则全透明
        geometry_msgs::msg::Point p[8];

        // vis_pos_world是目标物的坐标
        p[0].x = vis_pos_world(0) - width;
        p[0].y = vis_pos_world(1) + length;
        p[0].z = vis_pos_world(2) + hight;
        p[1].x = vis_pos_world(0) - width;
        p[1].y = vis_pos_world(1) - length;
        p[1].z = vis_pos_world(2) + hight;
        p[2].x = vis_pos_world(0) - width;
        p[2].y = vis_pos_world(1) - length;
        p[2].z = vis_pos_world(2) - hight;
        p[3].x = vis_pos_world(0) - width;
        p[3].y = vis_pos_world(1) + length;
        p[3].z = vis_pos_world(2) - hight;
        p[4].x = vis_pos_world(0) + width;
        p[4].y = vis_pos_world(1) + length;
        p[4].z = vis_pos_world(2) - hight;
        p[5].x = vis_pos_world(0) + width;
        p[5].y = vis_pos_world(1) - length;
        p[5].z = vis_pos_world(2) - hight;
        p[6].x = vis_pos_world(0) + width;
        p[6].y = vis_pos_world(1) - length;
        p[6].z = vis_pos_world(2) + hight;
        p[7].x = vis_pos_world(0) + width;
        p[7].y = vis_pos_world(1) + length;
        p[7].z = vis_pos_world(2) + hight;
        // LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
        for (int i = 0; i < 8; i++) {
            line_strip.points.push_back(p[i]);
        }
        //为了保证矩形框的八条边都存在：
        line_strip.points.push_back(p[0]);
        line_strip.points.push_back(p[3]);
        line_strip.points.push_back(p[2]);
        line_strip.points.push_back(p[5]);
        line_strip.points.push_back(p[6]);
        line_strip.points.push_back(p[1]);
        line_strip.points.push_back(p[0]);
        line_strip.points.push_back(p[7]);
        line_strip.points.push_back(p[4]);
        mkrarr.markers.push_back(line_strip);
    }

    static void visualizeText(
        visualization_msgs::msg::MarkerArray& mkr_arr,
        const double& stamp,
        const std::string& ns,
        const std::string& text,
        const utils::Vec3f& position,
        const utils::Color& c = utils::Color::White(),
        const double& size = 0.6,
        const int& id = -1
    ) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = rclcpp::Time(stamp);
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.ns = ns.c_str();
        if (id >= 0) {
            marker.id = id;
        } else {
            static int id = 0;
            marker.id = id++;
        }
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.scale.z = size;
        marker.color = c;
        marker.text = text;
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();
        marker.pose.orientation.w = 1.0;
        mkr_arr.markers.push_back(marker);
    };

    static void visualizePoint(
        visualization_msgs::msg::MarkerArray& mkr_arr,
        const double& stamp,
        const utils::Vec3f& pt,
        utils::Color color = utils::Color::Pink(),
        std::string ns = "pt",
        double size = 0.1,
        int id = -1,
        const bool& print_ns = true
    ) {
        visualization_msgs::msg::Marker marker_ball;
        static int cnt = 0;
        utils::Vec3f cur_pos = pt;
        if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
            return;
        }
        marker_ball.header.frame_id = "world";
        marker_ball.header.stamp = rclcpp::Time(stamp);
        marker_ball.ns = ns.c_str();
        marker_ball.id = id >= 0 ? id : cnt++;
        marker_ball.action = visualization_msgs::msg::Marker::ADD;
        marker_ball.pose.orientation.w = 1.0;
        marker_ball.type = visualization_msgs::msg::Marker::SPHERE;
        marker_ball.scale.x = size;
        marker_ball.scale.y = size;
        marker_ball.scale.z = size;
        marker_ball.color = color;

        geometry_msgs::msg::Point p;
        p.x = cur_pos.x();
        p.y = cur_pos.y();
        p.z = cur_pos.z();

        marker_ball.pose.position = p;
        mkr_arr.markers.push_back(marker_ball);

        // add test
        if (print_ns) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = rclcpp::Time(stamp);
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.ns = ns + "_text";
            if (id >= 0) {
                marker.id = id;
            } else {
                static int id = 0;
                marker.id = id++;
            }
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.scale.z = 0.6;
            marker.color = color;
            marker.text = ns;
            marker.pose.position.x = cur_pos.x();
            marker.pose.position.y = cur_pos.y();
            marker.pose.position.z = cur_pos.z() + 0.5;
            marker.pose.orientation.w = 1.0;
            mkr_arr.markers.push_back(marker);
        }
    }
    void vecEVec3fToPC2(const utils::vec_E<utils::Vec3f>& points, sensor_msgs::msg::PointCloud2& cloud) {
        // 设置header信息
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl_cloud.resize(points.size());
        for (long unsigned int i = 0; i < points.size(); i++) {
            pcl_cloud[i].x = static_cast<float>(points[i][0]);
            pcl_cloud[i].y = static_cast<float>(points[i][1]);
            pcl_cloud[i].z = static_cast<float>(points[i][2]);
        }
        pcl::toROSMsg(pcl_cloud, cloud);
        cloud.header.stamp = node->get_clock()->now();
        cloud.header.frame_id = "world";
    }
    void map_viz_callback(ma_map::MaMap& ma_map) {
        auto rog_map = ma_map.get_rog_map();
        auto cfg_ = ma_map.get_config();
        utils::Vec3f box_max = rog_map->robot_state_.p + cfg_.visualization_range / 2;
        utils::Vec3f box_min = rog_map->robot_state_.p - cfg_.visualization_range / 2;

        rog_map->boundBoxByLocalMap(box_min, box_max);
        if ((box_max - box_min).minCoeff() <= 0) {
            logger::ros2->warn("[ROGMap] Visualization range is too small.");
            return;
        }

        if (cfg_.pub_unknown_map_en && unknown_pub->get_subscription_count() >= 1) {
            utils::vec_E<utils::Vec3f> unknown_map, inf_unknown_map;
            rog_map->boxSearch(box_min, box_max, utils::UNKNOWN, unknown_map);
            sensor_msgs::msg::PointCloud2 cloud_msg;
            vecEVec3fToPC2(unknown_map, cloud_msg);
            cloud_msg.header.stamp = node->get_clock()->now();
            unknown_pub->publish(cloud_msg);
            if (cfg_.unk_inflation_en && unknown_inf_pub->get_subscription_count() >= 1) {
                rog_map->boxSearchInflate(box_min, box_max, utils::UNKNOWN, inf_unknown_map);
                vecEVec3fToPC2(inf_unknown_map, cloud_msg);
                cloud_msg.header.stamp = node->get_clock()->now();
                unknown_inf_pub->publish(cloud_msg);
            }
        }

        if (cfg_.frontier_extraction_en && frontier_pub->get_subscription_count() >= 1) {
            utils::vec_E<utils::Vec3f> frontier_map;
            rog_map->boxSearch(box_min, box_max, utils::FRONTIER, frontier_map);
            sensor_msgs::msg::PointCloud2 cloud_msg;
            vecEVec3fToPC2(frontier_map, cloud_msg);
            cloud_msg.header.stamp = node->get_clock()->now();
            frontier_pub->publish(cloud_msg);
        }

        utils::vec_E<utils::Vec3f> occ_map, inf_occ_map;
        sensor_msgs::msg::PointCloud2 cloud_msg;

        if (occ_pub->get_subscription_count() >= 1) {
            rog_map->boxSearch(box_min, box_max, utils::OCCUPIED, occ_map);
            vecEVec3fToPC2(occ_map, cloud_msg);
            occ_pub->publish(cloud_msg);
        }

        if (occ_inf_pub->get_subscription_count() >= 1) {
            rog_map->boxSearchInflate(box_min, box_max, utils::OCCUPIED, inf_occ_map);
            vecEVec3fToPC2(inf_occ_map, cloud_msg);
            cloud_msg.header.stamp = node->get_clock()->now();
            occ_inf_pub->publish(cloud_msg);
        }

        /* visualize ESDF Map*/
        if (cfg_.esdf_en) {
            if (esdf_pub->get_subscription_count() >= 1) {
                rog_map::PointCloud pc;
                rog_map->get_esdf_map()
                    ->getPositiveESDFPointCloud(box_min, box_max, rog_map->robot_state_.p.z() - 0.5, pc);
                pcl::toROSMsg(pc, cloud_msg);
                cloud_msg.header.frame_id = "world";
                cloud_msg.header.stamp = node->get_clock()->now();
                esdf_pub->publish(cloud_msg);
            }

            // if (vm_.esdf_neg_pub->get_subscription_count() >= 1) {
            //     PointCloud pc;
            //     esdf_map_->getNegativeESDFPointCloud(box_min, box_max,
            //     robot_state_.p.z() - 0.5, pc); pcl::toROSMsg(pc, cloud_msg);
            //     cloud_msg.header.frame_id = "world";
            //     cloud_msg.header.stamp = nh_->get_clock()->now();
            //     vm_.esdf_neg_pub->publish(cloud_msg);
            // }

#ifdef ESDF_MAP_DEBUG
            esdf_map_->getESDFOccPC2(box_min, box_max, cloud_msg);
            cloud_msg.header.stamp = nh_->get_clock()->now();
            vm_.esdf_occ_pub->publish(cloud_msg);
#endif
        }

        /* Publish visualization range */
        visualization_msgs::msg::MarkerArray mkr_arr;
        visualizeBoundingBox(
            mkr_arr,
            node->get_clock()->now().seconds(),
            box_min,
            box_max,
            "Visualization Range",
            utils::Color::Purple()
        );
        visualizeText(
            mkr_arr,
            node->get_clock()->now().seconds(),
            "Visualization Range Text",
            "Visualization Range",
            box_max + utils::Vec3f(0, 0, 0.5),
            utils::Color::Purple(),
            0.6,
            0
        );

        /* Publish local map range */
        utils::Vec3f local_map_max(999, 999, 999), local_map_min(-999, -999, -999);
        rog_map->boundBoxByLocalMap(local_map_min, local_map_max);
        visualizeBoundingBox(
            mkr_arr,
            node->get_clock()->now().seconds(),
            local_map_min,
            local_map_max,
            "Local Map Range",
            utils::Color::Orange()
        );
        visualizeText(
            mkr_arr,
            node->get_clock()->now().seconds(),
            "Local Map Range Text",
            "Local Map Range",
            local_map_max + utils::Vec3f(0, 0, 1.0),
            utils::Color::Orange(),
            0.6,
            0
        );

        /* Publish Ray-casting range */
        visualizeBoundingBox(
            mkr_arr,
            node->get_clock()->now().seconds(),
            rog_map->get_raycast_data().cache_box_min,
            rog_map->get_raycast_data().cache_box_max,
            "Updating Range",
            utils::Color::Green()
        );
        visualizeText(
            mkr_arr,
            node->get_clock()->now().seconds(),
            "Updating Range Text",
            "Updating Range",
            rog_map->get_raycast_data().cache_box_max + utils::Vec3f(0, 0, 0.5),
            utils::Color::Green(),
            0.6,
            0
        );

        /* Publish Local map origin */
        visualizePoint(
            mkr_arr,
            node->get_clock()->now().seconds(),
            rog_map->getLocalMapOrigin(),
            utils::Color::Red(),
            "Local Map Origin",
            0.2,
            0
        );

        if (cfg_.esdf_en) {
            utils::Vec3f esdf_box_max, esdf_box_min;
            rog_map->get_esdf_map()->getUpdatedBbox(esdf_box_min, esdf_box_max);
            visualizeText(
                mkr_arr,
                node->get_clock()->now().seconds(),
                "ESDF Map Text",
                "ESDF Map",
                esdf_box_max + utils::Vec3f(0, 0, 1.0),
                utils::Color::Blue(),
                0.6,
                0
            );
            visualizeBoundingBox(
                mkr_arr,
                node->get_clock()->now().seconds(),
                esdf_box_min,
                esdf_box_max,
                "ESDF Updating Range",
                utils::Color::Blue()
            );
        }

        mkr_arr_pub->publish(mkr_arr);
    }
};

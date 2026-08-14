#include "ros2/ros2_node.h"
#include <fstream>
#include "utils/logger.hpp"
#include "utils/plotter.hpp"
#include "utils/type_utils.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <memory>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <yaml-cpp/yaml.h>

namespace planner {
GlobalPlanner2d::GlobalPlanner2d(rclcpp::Node::SharedPtr nh_,std::string params_path):
    config(params_path),
    nh(nh_),
    mapInitialized(false),
    visualizer(nh_),
    //fsm_(config.fsm_config),
    plotter_(),
    ma_map_(std::make_shared<ma_map::MaMap>(config.map_params_path)),
    omni_lmpc_(config.lmpc_param) {
    start_time_ = std::chrono::steady_clock::now();

    mapSub = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
        config.map_topic_name,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { GlobalPlanner2d::map_callback(msg); }
    );
    OdomSub = nh->create_subscription<nav_msgs::msg::Odometry>(
        config.odom_topic_name,
        rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { GlobalPlanner2d::odom_callback(msg); }
    );
    targetSub = nh->create_subscription<geometry_msgs::msg::PoseStamped>(
        config.target_topic_name,
        rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { target_callback(msg); }
    );
    // 接收 rviz2 "Publish Point" 工具点选的点（geometry_msgs/PointStamped，默认 /clicked_point），
    // 每 4 个点连成一个封闭四边形区域并发布到 /ma_nav/clicked_regions 可视化
    clickedPointSub = nh->create_subscription<geometry_msgs::msg::PointStamped>(
        config.clickedPoint_topic_name,
        rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) { clicked_point_callback(msg); }
    );
    cmd_vel_pub_ = nh->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_chassis", 10);
    clicked_region_pub_ = nh->create_publisher<visualization_msgs::msg::MarkerArray>("/ma_nav/clicked_regions", 10);

    // 每 0.1 秒执行一次
    planner_timer_ = nh->create_wall_timer(
        std::chrono::milliseconds(33), // 时间间隔参数
        [&]() { planner_callback(); } // 回调函数
    );
    pub_viz_timer_ = nh->create_wall_timer(
        std::chrono::milliseconds(10), // 时间间隔参数
        [&]() { pub_callback(); } // 回调函数
    );
    controller_timer_ = nh->create_wall_timer(
        std::chrono::milliseconds(33), // 时间间隔参数
        [&]() { controller_callback(); } // 回调函数
    );

    // 保存/加载全局地图的 ROS2 服务
    save_map_srv_ = nh->create_service<std_srvs::srv::Trigger>(
        "/save_global_map",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
            save_global_map("/home/xyt/map/src/gcopter/map");
            res->success = true;
            res->message = "Global map saved to /home/xyt/map/src/gcopter/map";
        }
    );
    ma_map_->set_mapping_model(config.mapping_model);
    if (!config.mapping_model) {
        load_global_map("config.global_map_path");
    }
}
void GlobalPlanner2d::controller_callback() {
    if (!current_XYTheta.has_value()) {
        // spdlog::warn("[controller_callback] no current_XYtheta");
        return;
    }
    if (!trajectory_.isInitialized()) {
        // spdlog::warn("轨迹未初始化，等待规划...");
        return;
    }
    // 距离目标检查（优先）
    // 位置阈值 0.08m:比 MPC 停车半径(0.05)略宽,避免 MPC 在 0.06m 处爬行导致
    // 到位阶段永不触发;进入后若航向未对齐,原地旋转对齐后再停
    if ((current_XYTheta->head<2>() - goal_pose->p.head<2>()).norm() < 0.08) {
        // 目标带 yaw 且航向未对齐 -> 纯旋转收尾(MPC 的 Q_theta=0.1 权重低,残差大)
        const double yaw_err = std::remainder(goal_pose->yaw - (*current_XYTheta)(2), 2.0 * M_PI);
        if (std::abs(yaw_err) > 0.03) {
            // 到位旋转:大误差段 P 控制 + 最小转速保证(避免纯 P 尾段指数爬行、
            // “最后几度磨半天”);小误差段纯 P 平滑收敛(强制最小转速会过冲振荡)
            constexpr double kYaw = 4.0;   // 角速度增益
            constexpr double kWMax = 3.0;  // 最大角速度 rad/s(与 lmpc.u_max_w=4.0 同量级)
            constexpr double kWMin = 0.6;  // 最小角速度 rad/s(误差 >0.08 rad 时生效)
            constexpr double kBang = 0.08; // 大误差/小误差切换阈值 rad
            double w = kYaw * yaw_err;
            if (std::abs(yaw_err) > kBang)
            {
                const double w_abs = std::max(kWMin, std::min(kWMax, std::abs(w)));
                w = std::copysign(w_abs, yaw_err);
            }
            else
            {
                w = std::max(-kWMax, std::min(kWMax, w));
            }
            geometry_msgs::msg::Twist rot;
            rot.angular.z = w;
            cmd_vel_pub_->publish(rot);
            return;
        }
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        logger::ros2->debug("到达目标点，停止控制");
        return;
    }

    // 更新状态并求解（参考游标由 MPC 内部按机器人实际进度跟踪，
    //    不再依赖墙钟时间，避免落后参考时反复切角、偏差累积）
    omni_lmpc_.update_current_pose(*current_XYTheta);
    auto predicted = omni_lmpc_.slover(trajectory_);
    if (predicted.empty()) {
        logger::ros2->warn("MPC 求解失败");
        return;
    }

    // 发布控制指令
    Eigen::Vector3d u_cmd = omni_lmpc_.u_k;
    geometry_msgs::msg::Twist twist;
    twist.linear.x = u_cmd.x();
    twist.linear.y = u_cmd.y();
    twist.angular.z = u_cmd.z();
    cmd_vel_pub_->publish(twist);
}

void GlobalPlanner2d::planner_callback() {
    GlobalPlanner2d::plan_omni();
}
void GlobalPlanner2d::plan_omni() {
    if (this->goal_pose == std::nullopt) {
        logger::ros2->debug("no goal");
        return;
    }
    if (this->current_pose == std::nullopt) {
        logger::ros2->debug("no current_pose");
        return;
    }

    auto result = fsm_replanner.plan(goal_pose.value(), current_pose.value(),  ma_map_->get_grid_map());
    if (result) {
        auto [path, opt] = result.value();
        visualizer.PubGlobalPath(path.raw_path);
        //std::vector<Eigen::Vector2d> opt_path = opt.sampleTrajectory(0.1);
        visualizer.PubOptPath(path.optimized_path);

        //trajectory_ = opt.getTrajectory();
        // 新轨迹下发:把 MPC 跟踪游标定位到新轨迹上离机器人最近的点(全轨迹搜索),
        // 而不是 reset_track() 清零——清零会让参考回退到起点附近,MPC 急刹减速
        // (update_track_time 搜索窗口只有 [t_track_-0.3, t_track_+3.0],游标为 0 时只搜前 3s)
        // if (current_XYTheta.has_value()) {
        //     omni_lmpc_.initialize_track(*current_XYTheta, trajectory_);
        // } else {
        //     omni_lmpc_.reset_track();
        // }
        // visualizer.PubWayPoints(trajectory_);
        // std::vector<Eigen::Vector2d> dense_path = opt.sampleTrajectory(0.02);
        // visualizer.PubTrajectory(trajectory_, dense_path);
    }
}
// void GlobalPlanner2d::plan_omni() {
//     if (this->goal_pose == std::nullopt) {
//         logger::ros2->debug("no goal");
//         return;
//     }
//     if (this->current_pose == std::nullopt) {
//         logger::ros2->debug("no current_pose");
//         return;
//     }

//     auto result = fsm_.plan(goal_pose.value(), current_pose.value(), ma_map_->get_grid_map());
//     if (result) {
//         auto [astar_path, opt] = result.value();
//         visualizer.PubGlobalPath(astar_path);
//         std::vector<Eigen::Vector2d> opt_path = opt.sampleTrajectory(0.1);
//         visualizer.PubOptPath(opt_path);

//         trajectory_ = opt.getTrajectory();
//         // 新轨迹下发:把 MPC 跟踪游标定位到新轨迹上离机器人最近的点(全轨迹搜索),
//         // 而不是 reset_track() 清零——清零会让参考回退到起点附近,MPC 急刹减速
//         // (update_track_time 搜索窗口只有 [t_track_-0.3, t_track_+3.0],游标为 0 时只搜前 3s)
//         if (current_XYTheta.has_value()) {
//             omni_lmpc_.initialize_track(*current_XYTheta, trajectory_);
//         } else {
//             omni_lmpc_.reset_track();
//         }
//         visualizer.PubWayPoints(trajectory_);
//         std::vector<Eigen::Vector2d> dense_path = opt.sampleTrajectory(0.02);
//         visualizer.PubTrajectory(trajectory_, dense_path);
//     }
// }
void GlobalPlanner2d::odom_callback(const nav_msgs::msg::Odometry::SharedPtr& msg) {
    if (!current_pose.has_value()) {
        current_pose = utils::RobotState();
    }
    if (!current_XYTheta.has_value()) {
        current_XYTheta = Eigen::Vector3d::Zero();
    }
    const auto& quat = msg->pose.pose.orientation;
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    current_pose->p.x() = msg->pose.pose.position.x;
    current_pose->p.y() = msg->pose.pose.position.y;
    current_pose->p.z() = msg->pose.pose.position.z;
    const auto& twist = msg->twist.twist;
    current_pose->v.x() = twist.linear.x;
    current_pose->v.y() = twist.linear.y;
    current_pose->v.z() = twist.linear.z;
    current_pose->yaw = yaw;

    current_XYTheta->x() = msg->pose.pose.position.x;
    current_XYTheta->y() = msg->pose.pose.position.y;
    current_XYTheta->z() = yaw;
    rog_map::Pose pose;
    pose.first = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    pose.second = Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );
    ma_map_->update_odom(pose);
};
void GlobalPlanner2d::map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    rog_map::PointCloud pc;
    pcl::fromROSMsg(*msg, pc);
    ma_map_->update_cloud(pc);
    ma_map_->update_map();
    grid_map_ = ma_map_->get_grid_map();
    visualizer.visualize_occupied_map(*grid_map_);
    mapInitialized = true;
    //visualizer.visualizeMap(pc);
}
void GlobalPlanner2d::pub_callback() {
    static auto last_pub = std::chrono::steady_clock::now();
    // Use viz_time_rate from map config as publish frequency (Hz)
    if (ma_map_) {
        double viz_rate = ma_map_->get_config().viz_time_rate;
        if (viz_rate > 0) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - last_pub).count();
            if (elapsed < 1.0 / viz_rate) {
                return;
            }
            last_pub = now;
        }
    }
    visualizer.map_viz_callback(*ma_map_);
    // GridMap 占用栅格与 2D ESDF 的 OccupancyGrid 可视化（与 ROGMap 同频，受 viz_time_rate 限频）
    if (grid_map_) {
        visualizer.visualize_occupied_grid(*grid_map_);
        visualizer.visualize_esdf_grid(*grid_map_);
    }
}

void GlobalPlanner2d::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr& msg) {
    logger::ros2->info(
        "Received target pose with position ({:2f},{:2f},{:2f})",
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z
    );
    if (mapInitialized) {
        const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, 0.0);
        utils::RobotState temp_goal_pose;
        temp_goal_pose.p = goal;
        temp_goal_pose.yaw = atan2(msg->pose.orientation.z, msg->pose.orientation.w) * 2.0;
        goal_pose = temp_goal_pose;

        logger::ros2->info("set goal success");
    } else {
        logger::ros2->warn("map no init");
    }
    return;
}

void GlobalPlanner2d::clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr& msg) {
    clicked_points_.push_back(msg->point);
    logger::ros2->info(
        "Received rviz2 clicked point ({:.2f},{:.2f},{:.2f}), accumulated {}/4",
        msg->point.x,
        msg->point.y,
        msg->point.z,
        clicked_points_.size()
    );
    if (clicked_points_.size() < 4) {
        return;
    }

    // 每 4 个点连成一个封闭四边形：p0->p1->p2->p3->p0
    visualization_msgs::msg::Marker region;
    region.header.stamp = nh->now();
    region.header.frame_id = "world";
    region.ns = "clicked_region";
    region.id = static_cast<int>(clicked_regions_.size());
    region.type = visualization_msgs::msg::Marker::LINE_STRIP;
    region.action = visualization_msgs::msg::Marker::ADD;
    region.pose.orientation.w = 1.0;
    region.scale.x = 0.05; // 线宽
    region.color.r = 0.0;
    region.color.g = 1.0;
    region.color.b = 0.0;
    region.color.a = 1.0;
    for (const auto& p : clicked_points_) {
        region.points.push_back(p);
    }
    region.points.push_back(clicked_points_.front()); // 首点重复以闭合

    clicked_regions_.push_back(region);
    clicked_points_.clear();

    // 重新发布全部历史区域，已画的区域保持显示
    visualization_msgs::msg::MarkerArray regions;
    regions.markers = clicked_regions_;
    clicked_region_pub_->publish(regions);
    logger::ros2->info("Published clicked region #{}", region.id);
}

void GlobalPlanner2d::load_global_map(const std::string& pgm_path) {
    std::ifstream f(pgm_path, std::ios::binary);
    if (!f.is_open()) {
        logger::ros2->error("Failed to open global map: {}", pgm_path);
        return;
    }
    std::string magic;
    int w, h, maxval;
    f >> magic >> w >> h >> maxval;
    f.get();
    if (magic != "P5" || maxval != 255) {
        logger::ros2->error("Only P5 binary PGM (maxval=255) supported");
        return;
    }
    // 读入全部像素，之后按世界坐标做任意像素查询
    std::vector<unsigned char> pixels(static_cast<size_t>(w) * h);
    f.read(reinterpret_cast<char*>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
    f.close();

    // 读取同目录 global_map.yaml 中的 resolution/origin（save_global_map 生成的）。
    // 缺少该信息无法把 PGM 像素映射回世界坐标，直接拒绝加载，避免错位/越界。
    std::string yaml_path = pgm_path;
    if (yaml_path.size() > 4 && yaml_path.substr(yaml_path.size() - 4) == ".pgm") {
        yaml_path = yaml_path.substr(0, yaml_path.size() - 4) + ".yaml";
    }
    double pgm_res = 0.0;
    Eigen::Vector2d pgm_origin = Eigen::Vector2d::Zero();
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        if (!root["resolution"] || !root["origin"]) {
            logger::ros2->error(
                "Global map yaml {} missing resolution/origin, load aborted (size mismatch would misalign map)",
                yaml_path
            );
            return;
        }
        pgm_res = root["resolution"].as<double>();
        auto origin = root["origin"].as<std::vector<double>>();
        if (origin.size() < 2 || pgm_res <= 0.0) {
            logger::ros2->error("Global map yaml {} has invalid resolution/origin, load aborted", yaml_path);
            return;
        }
        pgm_origin << origin[0], origin[1];
    } catch (const std::exception& e) {
        logger::ros2->error(
            "Failed to parse global map yaml {} ({}), load aborted (size mismatch would misalign map)",
            yaml_path,
            e.what()
        );
        return;
    }

    // 按世界坐标重投影到当前栅格图：遍历当前每个 cell，把其中心的世界坐标
    // 换算成 PGM 像素坐标，仅当像素落在 PGM 范围内时取值（否则保持 0/自由）。
    // 这样无论当前地图尺寸/分辨率与保存时是否一致，障碍物都落在正确位置。
    auto grid = ma_map_->get_grid_map();
    const Eigen::Vector2i voxel_num = grid->getVoxelNum();
    const Eigen::Vector2d grid_origin = grid->getOrigin();
    const double grid_res = grid->getResolution();
    const int pgm_rows = h, pgm_cols = w;

    grid_map::RowMatrixXi global = grid_map::RowMatrixXi::Zero(voxel_num.x(), voxel_num.y());
    int loaded = 0;
    for (int i = 0; i < voxel_num.x(); ++i) {
        for (int j = 0; j < voxel_num.y(); ++j) {
            // 当前 cell 中心的世界坐标（与 GridMap::indexToPos 一致）
            const double wx = (i + 0.5) * grid_res + grid_origin.x();
            const double wy = (j + 0.5) * grid_res + grid_origin.y();
            // PGM 像素坐标约定（与 save_global_map 一致）：
            //   占用矩阵 map_occ(row, col) 的 row=世界 x、col=世界 y；
            //   PGM 列方向 = 世界 Y（左→右 y 增大），PGM 行方向 = 世界 X（顶→底 x 减小）
            const int px = static_cast<int>(std::floor((wy - pgm_origin.y()) / pgm_res)); // PGM 列 = 世界 y
            const int py_grid =
                static_cast<int>(std::floor((wx - pgm_origin.x()) / pgm_res)); // 世界 x 格号（0 = x 最小）
            const int pgm_row = pgm_rows - 1 - py_grid; // 顶行 = x 最大
            if (px < 0 || px >= pgm_cols || pgm_row < 0 || pgm_row >= pgm_rows) {
                continue; // 当前 cell 在老地图范围之外，保持自由
            }
            if (pixels[static_cast<size_t>(pgm_row) * pgm_cols + px] < 128) {
                global(i, j) = 1;
                ++loaded;
            }
        }
    }
    ma_map_->set_global_map(global);
    logger::ros2->info(
        "Loaded global map {}x{} (res={:.3f}, origin=({:.2f},{:.2f})) -> current grid {}x{} (res={:.3f}), {} occupied cells projected",
        pgm_cols,
        pgm_rows,
        pgm_res,
        pgm_origin.x(),
        pgm_origin.y(),
        voxel_num.x(),
        voxel_num.y(),
        grid_res,
        loaded
    );
}

void GlobalPlanner2d::save_global_map(const std::string& map_dir) {
    auto map_occ = ma_map_->get_occupancy_2d(); // 保存全球永久层
    int h = map_occ.rows(), w = map_occ.cols();
    double res = ma_map_->get_grid_map()->getResolution();
    Eigen::Vector2d origin = ma_map_->get_grid_map()->getOrigin();

    std::ofstream f(map_dir + "/global_map.pgm", std::ios::binary);
    f << "P5\n" << w << " " << h << "\n255\n";
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            // GridMap 行 0=底部=世界 y_min，PGM 行 0=顶部=世界 y_max，翻转 Y
            f.put(map_occ(h - 1 - y, x) ? (char)0 : (char)255);
    f.close();

    std::ofstream y(map_dir + "/global_map.yaml");
    y << "image: global_map.pgm\n";
    y << "resolution: " << res << "\n";
    y << "origin: [" << origin.x() << ", " << origin.y() << ", 0.0]\n";
    y << "negate: 0\n";
    y << "occupied_thresh: 0.65\n";
    y << "free_thresh: 0.196\n";
    y << "mode: trinary\n";
    y.close();

    logger::ros2->info("Saved global map {}x{} -> {}", w, h, map_dir);
}
} // namespace planner
#include "ros2/ros2_node.h"
#include <string>
#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto nh = std::make_shared<rclcpp::Node>("ma_nav_node");

    nh->declare_parameter<std::string>("planner_params_path");
    std::string params_path;
    nh->get_parameter("planner_params_path", params_path);

    planner::GlobalPlanner2d global_planner_2d(nh, params_path);

    // 创建多线程执行器，线程数可自定义，0 表示使用 CPU 核心数
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), global_planner_2d.get_num_threads() /* 线程数 */);
    executor.add_node(nh);

    rclcpp::WallRate rate(1000);
    while (rclcpp::ok()) {
        // 注意：不要在这里同时调用 spin_some，executor 已经接管 spin
        executor.spin_some(); 
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
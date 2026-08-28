#include "ros2/ros2_node.h"
#include <string>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto nh = std::make_shared<rclcpp::Node>("ma_nav_node");

    nh->declare_parameter<std::string>("planner_params_path");
    std::string params_path;
    nh->get_parameter("planner_params_path", params_path);

    planner::GlobalPlanner2d global_planner_2d(nh, params_path);
    
    rclcpp::WallRate rate(1000);
    while (rclcpp::ok()) {
        rclcpp::spin_some(nh);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
#include "ros2/ros2_node.h"
#include <string>

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto nh = std::make_shared<rclcpp::Node>("ma_nav_node");

  //planner::GlobalPlanner global_planner(nh);
  std::string params_path="/home/xyt/map/src/MA_Nav/config/planner.yaml";
  planner::GlobalPlanner2d global_planner_2d(nh,params_path);
  rclcpp::WallRate rate(1000);
  while (rclcpp::ok()) {
    rclcpp::spin_some(nh);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
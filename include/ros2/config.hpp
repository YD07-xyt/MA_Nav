#pragma once
#include "planner/controller/omni_lmpc.hpp"
#include "planner/fsm/fsm.hpp"
#include "utils/logger.hpp"
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <string>
#include <vector>
namespace planner {

struct Config {
  std::string map_topic_name ;
  std::string target_topic_name;
  std::string clickedPoint_topic_name;
  std::string odom_topic_name;
  bool mapping_model; // true: 建图模式，false: 规划模式
  std::string map_params_path;
  std::string global_map_path;
  FSM::PlannerConfig planner_config;
  controller::LMpc::LMpcParam lmpc_param;
  Config(){
    //TODO: 完成剩余的参数加载
    YAML::Node config = YAML::LoadFile("config.yaml");
    map_topic_name = config["ros2"]["map_topic_name"].as<std::string>();
    target_topic_name = config["ros2"]["target_topic_name"].as<std::string>();
    clickedPoint_topic_name = config["ros2"]["clickedPoint_topic_name"].as<std::string>();
    odom_topic_name = config["ros2"]["odom_topic_name"].as<std::string>();
    mapping_model=config["map"]["mapping_model"].as<bool>();
    map_params_path = config["ros2"]["map_params_path"].as<std::string>();
    global_map_path = config["ros2"]["global_map_path"].as<std::string>();
    
  }

};
} // namespace planner
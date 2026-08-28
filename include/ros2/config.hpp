#pragma once
#include "planner/controller/mpc.h"
#include "planner/controller/omni_lmpc.hpp"
#include "planner/replan/fsm_replanner.h"
#include "utils/logger.hpp"
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <string>
#include <vector>
namespace planner {

//从 YAML 节点加载 Eigen::Vector3d
inline Eigen::Vector3d load_vector3d(const YAML::Node& node) {
    if (!node.IsDefined() || !node.IsSequence() || node.size() != 3) {
        return Eigen::Vector3d::Zero();
    }
    return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

//从 YAML 节点加载 Eigen::Matrix3d（期望 3x3 数组或长度为9的列表）
inline Eigen::Matrix3d load_matrix3d(const YAML::Node& node) {
    Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
    if (!node.IsDefined()) return mat;
    if (node.IsSequence() && node.size() == 9) {
        // 按行优先填充
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                mat(i, j) = node[i * 3 + j].as<double>();
    } else if (node.IsSequence() && node.size() == 3) {
        // 假设是 3x3 矩阵的嵌套数组
        for (int i = 0; i < 3; ++i) {
            auto row = node[i];
            if (row.IsSequence() && row.size() == 3) {
                for (int j = 0; j < 3; ++j)
                    mat(i, j) = row[j].as<double>();
            }
        }
    }
    return mat;
}
//加载 MaSplineOptimizerConfig
inline void load_stage_config(const YAML::Node& node, ma_spline_opt::StageOptimizerConfig& stage) {
    if (!node) return;

    if (node["rho_energy"]) stage.rho_energy = node["rho_energy"].as<double>();
    if (node["weight_time"]) stage.weight_time = node["weight_time"].as<double>();
    if (node["weight_mean_time"]) stage.weight_mean_time = node["weight_mean_time"].as<double>();
    if (node["mean_lower"]) stage.mean_lower = node["mean_lower"].as<double>();
    if (node["mean_upper"]) stage.mean_upper = node["mean_upper"].as<double>();
    if (node["min_time"]) stage.min_time = node["min_time"].as<double>();
    if (node["weight_min_time"]) stage.weight_min_time = node["weight_min_time"].as<double>();
    if (node["weight_obstacle"]) stage.weight_obstacle = node["weight_obstacle"].as<double>();
    if (node["weight_vel"]) stage.weight_vel = node["weight_vel"].as<double>();
    if (node["weight_acc"]) stage.weight_acc = node["weight_acc"].as<double>();
    if (node["max_iterations"]) stage.max_iterations = node["max_iterations"].as<int>();
    if (node["g_epsilon"]) stage.g_epsilon = node["g_epsilon"].as<double>();
    if (node["lbfgs_mem_size"]) stage.lbfgs_mem_size = node["lbfgs_mem_size"].as<int>();
    if (node["lbfgs_delta"]) stage.lbfgs_delta = node["lbfgs_delta"].as<double>();
}

inline void load_ma_spline_opt_config(const YAML::Node& node, ma_spline_opt::MaSplineOptimizerConfig& param) {
    if (!node) return;

    if (node["mode"]) {
        std::string mode = node["mode"].as<std::string>();
        if (mode == "OMNI_XY") param.mode = ma_spline_opt::MaSplineOptimizerConfig::Mode::OMNI_XY;
        else if (mode == "OMNI_XY_YAW") param.mode = ma_spline_opt::MaSplineOptimizerConfig::Mode::OMNI_XY_YAW;
        else if (mode == "OMNI_XY_YAW_JOINT")
            param.mode = ma_spline_opt::MaSplineOptimizerConfig::Mode::OMNI_XY_YAW_JOINT;
    }

    if (node["safe_distance"]) param.safe_distance = node["safe_distance"].as<double>();
    if (node["v_max"]) param.v_max = node["v_max"].as<double>();
    if (node["a_max"]) param.a_max = node["a_max"].as<double>();
    if (node["integral_num_steps"]) param.integral_num_steps = node["integral_num_steps"].as<int>();

    if (node["stage1"]) load_stage_config(node["stage1"], param.stage1);

    if (node["stage2"]) load_stage_config(node["stage2"], param.stage2);
}
//加载 MincoOptimizerConfig
inline void load_minco_opt_config(const YAML::Node& node, minco_opt::MincoOptimizerConfig& param) {
    if (!node) return;
    if (node["weight_smooth"]) param.weight_smooth = node["weight_smooth"].as<double>();
    if (node["weight_obstacle"]) param.weight_obstacle = node["weight_obstacle"].as<double>();
    if (node["weight_feasibility"]) param.weight_feasibility = node["weight_feasibility"].as<double>();
    if (node["weight_time"]) param.weight_time = node["weight_time"].as<double>();
    if (node["weight_mean_time"]) param.weight_mean_time = node["weight_mean_time"].as<double>();
    if (node["max_vel"]) param.max_vel = node["max_vel"].as<double>();
    if (node["max_acc"]) param.max_acc = node["max_acc"].as<double>();
    if (node["safe_distance"]) param.safe_distance = node["safe_distance"].as<double>();
    if (node["mean_time_lower_bound"]) param.mean_time_lower_bound = node["mean_time_lower_bound"].as<double>();
    if (node["mean_time_upper_bound"]) param.mean_time_upper_bound = node["mean_time_upper_bound"].as<double>();
    if (node["integral_resolution"]) param.integral_resolution = node["integral_resolution"].as<int>();
    if (node["max_iterations"]) param.max_iterations = node["max_iterations"].as<int>();
    if (node["g_epsilon"]) param.g_epsilon = node["g_epsilon"].as<double>();
    if (node["min_time"]) param.min_time = node["min_time"].as<double>();
}

// 加载 ReplanParam
inline void load_replan_param(const YAML::Node& node, replan::FsmReplan::ReplanParam& param) {
    if (!node) return;
    if (node["goal_deviation"]) param.goal_deviation = load_vector3d(node["goal_deviation"]);
    // if (node["replan_interval"]) param.replan_interval = node["replan_interval"].as<double>();
    if (node["replan_lateral_dev"]) param.replan_lateral_dev = node["replan_lateral_dev"].as<double>();
    if (node["minco_traj_validity_duration"])
        param.minco_traj_validity_duration = node["minco_traj_validity_duration"].as<double>();
    if (node["goal_reached_radius"]) param.goal_reached_radius = node["goal_reached_radius"].as<double>();
    //if (node["hard_clearance"]) param.hard_clearance = node["hard_clearance"].as<double>();
    if (node["minco_traj_continuity_threshold"])
        param.minco_traj_continuity_threshold = node["minco_traj_continuity_threshold"].as<double>();
    if (node["projection_search_resolution"])
        param.projection_search_resolution = node["projection_search_resolution"].as<double>();
}

// 加载 PathPostProcessingParams
inline void load_path_post_processing_params(
    const YAML::Node& node,
    path_planning::PathPostProcessing::PathPostProcessingParams& param
) {
    if (!node) return;
    if (node["max_traj_num"]) param.max_traj_num = node["max_traj_num"].as<int>();
    if (node["dense_sample_resolution"]) param.dense_sample_resolution = node["dense_sample_resolution"].as<double>();
    if (node["rotation_penalty_weight"]) param.rotation_penalty_weight = node["rotation_penalty_weight"].as<double>();
    if (node["safe_threshold"]) param.safe_threshold = node["safe_threshold"].as<double>();
    if (node["max_vel"]) param.max_vel = node["max_vel"].as<double>();
    if (node["max_acc"]) param.max_acc = node["max_acc"].as<double>();
    if (node["time_resolution"]) param.time_resolution = node["time_resolution"].as<double>();
    if (node["min_traj_num"]) param.min_traj_num = node["min_traj_num"].as<int>();
    if (node["traj_cut_length"]) param.traj_cut_length = node["traj_cut_length"].as<double>();
    if (node["distance_weight"]) param.distance_weight = node["distance_weight"].as<double>();
    if (node["yaw_weight"]) param.yaw_weight = node["yaw_weight"].as<double>();
}

//加载 PlannerConfig
inline void load_planner_config(const YAML::Node& node, replan::FsmReplan::PlannerConfig& config) {
    if (!node) return;
    if (node["replan_params"]) load_replan_param(node["replan_params"], config.replan_params);
    // 新增：加载 minco 优化器参数
    if (node["minco_opt_params"]) load_minco_opt_config(node["minco_opt_params"], config.minco_opt_params);
    if (node["ma_spline_opt_params"])
        load_ma_spline_opt_config(node["ma_spline_opt_params"], config.ma_spline_opt_params);
    if (node["path_planning_params"])
        load_path_post_processing_params(node["path_planning_params"], config.path_planning_params);
}

//加载 LMpcParam
inline void load_lmpc_param(const YAML::Node& node, controller::LMpc::LMpcParam& param) {
    if (!node) return;
    if (node["N"]) param.N = node["N"].as<int>();
    if (node["dt"]) param.dt = node["dt"].as<double>();
    if (node["u_min"]) param.u_min = load_vector3d(node["u_min"]);
    if (node["u_max"]) param.u_max = load_vector3d(node["u_max"]);
    if (node["x_min"]) param.x_min = load_vector3d(node["x_min"]);
    if (node["x_max"]) param.x_max = load_vector3d(node["x_max"]);
    if (node["Q"]) param.Q = load_matrix3d(node["Q"]);
    if (node["R"]) param.R = load_matrix3d(node["R"]);
}

// 加载 6 维向量
inline Eigen::Matrix<double, 6, 1> load_vector6d(const YAML::Node& node) {
    Eigen::Matrix<double, 6, 1> vec = Eigen::Matrix<double, 6, 1>::Zero();
    if (!node.IsDefined() || !node.IsSequence() || node.size() != 6) {
        return vec;
    }
    for (int i = 0; i < 6; ++i) {
        vec(i) = node[i].as<double>();
    }
    return vec;
}

// 辅助函数：加载 6x6 矩阵
// 支持两种格式：
//   1. 长度为 36 的列表，按行优先填充
//   2. 长度为 6 的列表，作为对角矩阵
inline Eigen::Matrix<double, 6, 6> load_matrix6d(const YAML::Node& node) {
    Eigen::Matrix<double, 6, 6> mat = Eigen::Matrix<double, 6, 6>::Zero();
    if (!node.IsDefined()) return mat;

    if (node.IsSequence() && node.size() == 36) {
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                mat(i, j) = node[i * 6 + j].as<double>();
            }
        }
    } else if (node.IsSequence() && node.size() == 6) {
        for (int i = 0; i < 6; ++i) {
            mat(i, i) = node[i].as<double>();
        }
    }

    return mat;
}

//加载新 MPC 参数
inline void load_mpc_param(const YAML::Node& node, control::Mpc::Param& param) {
    if (!node) return;

    if (node["N"]) param.N = node["N"].as<int>();
    if (node["dt"]) param.dt = node["dt"].as<double>();

    if (node["Q"]) param.Q = load_matrix6d(node["Q"]);
    if (node["QN"]) param.QN = load_matrix6d(node["QN"]);

    if (node["R"]) param.R = load_matrix3d(node["R"]);
    if (node["Rd"]) param.Rd = load_matrix3d(node["Rd"]);

    if (node["x_min"]) param.x_min = load_vector6d(node["x_min"]);
    if (node["x_max"]) param.x_max = load_vector6d(node["x_max"]);

    if (node["u_min"]) param.u_min = load_vector3d(node["u_min"]);
    if (node["u_max"]) param.u_max = load_vector3d(node["u_max"]);
}

struct Config {
    std::string map_topic_name;
    std::string target_topic_name;
    std::string clickedPoint_topic_name;
    std::string odom_topic_name;
    std::string cmd_vel_name;
    std::string save_global_map_path;
    std::string save_map_srv_topic;
    bool is_minco;
    bool mapping_model; // true: 建图模式，false: 规划模式
    std::string map_params_path;
    std::string global_map_path;
    replan::FsmReplan::PlannerConfig planner_config;
    controller::LMpc::LMpcParam lmpc_param;
    control::Mpc::Param mpc_params;
    explicit Config(std::string params_path) {
        YAML::Node config = YAML::LoadFile(params_path);

        // 加载 ROS2 话题和地图参数（已有）
        if (config["ros2"]) {
            map_topic_name = config["ros2"]["map_topic_name"].as<std::string>();
            target_topic_name = config["ros2"]["target_topic_name"].as<std::string>();
            clickedPoint_topic_name = config["ros2"]["clickedPoint_topic_name"].as<std::string>();
            odom_topic_name = config["ros2"]["odom_topic_name"].as<std::string>();
            cmd_vel_name=config["ros2"]["cmd_vel_name"].as<std::string>();
            map_params_path = config["ros2"]["map_params_path"].as<std::string>();
            global_map_path = config["ros2"]["global_map_path"].as<std::string>();
            save_global_map_path=config["ros2"]["save_global_map_path"].as<std::string>();
            save_map_srv_topic=config["ros2"]["save_map_srv_topic"].as<std::string>();
            is_minco = config["ros2"]["is_minco"].as<bool>();
        }
        if (config["map"]) {
            mapping_model = config["map"]["mapping_model"].as<bool>();
        }

        // 加载规划器配置
        if (config["planner_config"]) {
            load_planner_config(config["planner_config"], planner_config);
        }

        // 加载 LMPC 参数
        if (config["lmpc_param"]) {
            load_lmpc_param(config["lmpc_param"], lmpc_param);
        }
        // 加载新世界系线性 MPC 参数
        if (config["mpc_param"]) {
            load_mpc_param(config["mpc_param"], mpc_params);
        }
    }
};
} // namespace planner
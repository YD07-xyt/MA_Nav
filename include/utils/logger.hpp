#pragma once
#include <memory>
#include "utils/fmt_eigen.hpp"
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace logger {
inline std::shared_ptr<spdlog::logger> Parameter = spdlog::stdout_color_mt("Parameter");
inline std::shared_ptr<spdlog::logger> SlidingMap = spdlog::stdout_color_mt("SlidingMap");
inline std::shared_ptr<spdlog::logger> ProbMap = spdlog::stdout_color_mt("ProbMap");
inline std::shared_ptr<spdlog::logger> CounterMap = spdlog::stdout_color_mt("CounterMap");
inline std::shared_ptr<spdlog::logger> InfMap = spdlog::stdout_color_mt("InfMap");
inline std::shared_ptr<spdlog::logger> ESDFMap = spdlog::stdout_color_mt("ESDFMap");
inline std::shared_ptr<spdlog::logger> ROGMap = spdlog::stdout_color_mt("ROGMap");
inline std::shared_ptr<spdlog::logger> MaMap = spdlog::stdout_color_mt("MaMap");

// 创建带整行颜色的 logger
inline std::shared_ptr<spdlog::logger> create_colored_logger(const std::string& name) {
    auto logger = spdlog::stdout_color_mt(name);
    logger->set_pattern("%^[%Y-%m-%d %H:%M:%S.%e] [%l] %v%$");
    return logger;
}

// 使用该函数创建所有 logger
inline std::shared_ptr<spdlog::logger> bt_replan = create_colored_logger("bt_replan");
inline std::shared_ptr<spdlog::logger> fsm_replan = create_colored_logger("fsm_replan");
inline std::shared_ptr<spdlog::logger> planning = create_colored_logger("path_planning");
inline std::shared_ptr<spdlog::logger> traj_opt = create_colored_logger("traj_optimize");
inline std::shared_ptr<spdlog::logger> ros2 = create_colored_logger("plan_ros2_node");
inline std::shared_ptr<spdlog::logger> controller = create_colored_logger("controller");
inline std::shared_ptr<spdlog::logger> timer = create_colored_logger("timer");
// extern std::shared_ptr<spdlog::logger> file_logger =
// spdlog::rotating_logger_mt(
//     "file_logger", "logs/app.log", 1024 * 1024 * 5, 3);

} // namespace logger
#pragma once
#include <memory>
#include"utils/fmt_eigen.hpp"
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


inline std::shared_ptr<spdlog::logger> bt_replan = spdlog::stdout_color_mt("bt_replan");
inline std::shared_ptr<spdlog::logger> fsm_replan = spdlog::stdout_color_mt("fsm_replan");
inline std::shared_ptr<spdlog::logger> planning = spdlog::stdout_color_mt("path_planning");
inline std::shared_ptr<spdlog::logger> traj_opt = spdlog::stdout_color_mt("traj_optimize");
inline std::shared_ptr<spdlog::logger> ros2 = spdlog::stdout_color_mt("plan_ros2_node");
inline std::shared_ptr<spdlog::logger> controller = spdlog::stdout_color_mt("controller");

inline std::shared_ptr<spdlog::logger> timer = spdlog::stdout_color_mt("timer");

// extern std::shared_ptr<spdlog::logger> file_logger =
// spdlog::rotating_logger_mt(
//     "file_logger", "logs/app.log", 1024 * 1024 * 5, 3);
} // namespace logger
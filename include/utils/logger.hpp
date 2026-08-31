#pragma once
#include <memory>
#include "utils/fmt_eigen.hpp"
#include <source_location>
#include <spdlog/common.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace logger {
// info 结构体（接受 logger）
template <typename... Args>
struct info {
    constexpr info(const std::shared_ptr<spdlog::logger>& log,
                   fmt::format_string<Args...> fmt,
                   Args&&... args,
                   std::source_location loc = std::source_location::current()) 
    {
        log->log(spdlog::source_loc{loc.file_name(),
                                    static_cast<int>(loc.line()),
                                    loc.function_name()},
                 spdlog::level::info,
                 fmt,
                 std::forward<Args>(args)...);
    }
};
template <typename... Args>
info(const std::shared_ptr<spdlog::logger>&,
     fmt::format_string<Args...> fmt,
     Args&&... args) -> info<Args...>;

template <typename... Args>
struct warn {
    constexpr warn(const std::shared_ptr<spdlog::logger>& log,
                   fmt::format_string<Args...> fmt,
                   Args&&... args,
                   std::source_location loc = std::source_location::current()) 
    {
        log->log(spdlog::source_loc{loc.file_name(),
                                    static_cast<int>(loc.line()),
                                    loc.function_name()},
                 spdlog::level::warn,
                 fmt,
                 std::forward<Args>(args)...);
    }
};
template <typename... Args>
warn(const std::shared_ptr<spdlog::logger>&,
     fmt::format_string<Args...> fmt,
     Args&&... args) -> warn<Args...>;


template <typename... Args>
struct error {
    constexpr error(const std::shared_ptr<spdlog::logger>& log,
                   fmt::format_string<Args...> fmt,
                   Args&&... args,
                   std::source_location loc = std::source_location::current()) 
    {
        log->log(spdlog::source_loc{loc.file_name(),
                                    static_cast<int>(loc.line()),
                                    loc.function_name()},
                 spdlog::level::err,
                 fmt,
                 std::forward<Args>(args)...);
    }
};
template <typename... Args>
error(const std::shared_ptr<spdlog::logger>&,
     fmt::format_string<Args...> fmt,
     Args&&... args) -> error<Args...>;

template <typename... Args>
struct debug {
    constexpr debug(const std::shared_ptr<spdlog::logger>& log,
                   fmt::format_string<Args...> fmt,
                   Args&&... args,
                   std::source_location loc = std::source_location::current()) 
    {
        log->log(spdlog::source_loc{loc.file_name(),
                                    static_cast<int>(loc.line()),
                                    loc.function_name()},
                 spdlog::level::debug,
                 fmt,
                 std::forward<Args>(args)...);
    }
};
template <typename... Args>
debug(const std::shared_ptr<spdlog::logger>&,
     fmt::format_string<Args...> fmt,
     Args&&... args) -> debug<Args...>;
    
#define LOG_INFO(logger, ...) SPDLOG_LOGGER_INFO(logger, __VA_ARGS__)
#define LOG_WARN(logger, ...) SPDLOG_LOGGER_WARN(logger, __VA_ARGS__)
#define LOG_ERROR(logger, ...) SPDLOG_LOGGER_ERROR(logger, __VA_ARGS__)
#define LOG_DEBUG(logger, ...) SPDLOG_LOGGER_DEBUG(logger, __VA_ARGS__)




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
    //logger->set_pattern("%^ [%m-%d %H:%M:%S.%e] [%n] [%l] %v %$");
    logger->set_pattern("%^[%m-%d %H:%M:%S.%e] [%n] [%l] [%s:%#] %v%$");
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
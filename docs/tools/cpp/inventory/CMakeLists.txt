# MA_Nav 文档流水线 · 证据提取器（inventory）独立构建工程
#
# 重要约束：
#   1. 本工程**独立**于 ma_nav 的 colcon 工作区，不被根 CMakeLists.txt 引用，
#      也不修改 src/ 下任何 CMakeLists.txt。
#   2. 只依赖 nlohmann/json（系统头 /usr/include/nlohmann/json.hpp）与 POSIX。
#   3. 运行时通过 clangd 的 LSP 接口 textDocument/documentSymbol 提取符号表，
#      不产出任何目标文件，不链接被文档化的仓库。
#
# 构建（需用户许可）：
#   cmake -S docs/tools/cpp/inventory -B docs/tools/cpp/inventory/build
#   cmake --build docs/tools/cpp/inventory/build -j

cmake_minimum_required(VERSION 3.16)
project(ma_nav_inventory LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_executable(ma_nav_inventory src/main.cpp)

find_path(NLOHMANN_JSON_INCLUDE nlohmann/json.hpp PATHS /usr/include /usr/local/include)
if(NOT NLOHMANN_JSON_INCLUDE)
    message(FATAL_ERROR "未找到 nlohmann/json.hpp")
endif()
target_include_directories(ma_nav_inventory PRIVATE ${NLOHMANN_JSON_INCLUDE})

target_compile_options(ma_nav_inventory PRIVATE -Wall -Wextra)

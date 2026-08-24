cmake_minimum_required(VERSION 3.8)
project(ma_nav)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

#rog_map need
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

add_definitions(-DORIGIN_AT_CORNER)

# 查找依赖
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(yaml-cpp REQUIRED)
find_package(ompl REQUIRED)
find_package(spdlog REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PCL REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# CasADi 通常安装在 /usr/local
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

file(GLOB_RECURSE map_sources CONFIGURE_DEPENDS "src/map/*.cpp")
file(GLOB_RECURSE planner_sources CONFIGURE_DEPENDS "src/planner/*.cpp"  
  "src/planner/path_planning/*.cpp" "src/planner/replan/*.cpp" "src/planner/contoller/*.cpp")

message(STATUS "planner_sources: ${planner_sources}")

add_library(3d_occ_map SHARED
${map_sources}
)

target_link_libraries(3d_occ_map
spdlog::spdlog
${PCL_LIBRARIES}
yaml-cpp::yaml-cpp
)

ament_auto_add_library(planner
${planner_sources}
src/ros2_node.cpp
)

target_link_libraries(planner
  ${OMPL_LIBRARIES}
  spdlog::spdlog
  casadi
  osqp::osqp
  Boost::boost
  OsqpEigen::OsqpEigen
  yaml-cpp::yaml-cpp
)

# 添加可执行文件
ament_auto_add_executable(ma_nav src/ma_nav.cpp)
set_target_properties(ma_nav PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# 链接依赖库
target_link_libraries(ma_nav
  ${OMPL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  spdlog::spdlog
  casadi
  OsqpEigen::OsqpEigen
  Boost::boost
  libqpOASES.a
  yaml-cpp::yaml-cpp
  planner
  3d_occ_map
)


# 导出包
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
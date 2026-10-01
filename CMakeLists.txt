cmake_minimum_required(VERSION 3.10)
project(ma_nav)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3  -Wall -g -DNDEBUG -march=native")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3  -Wall -g -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3  -g -DNDEBUG")
#########################################################
###################     LTO     #########################
#########################################################
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)

if(ipo_supported)
    # 显式使用并行 LTO，避免 GCC 12+ 默认串行 LTRANS
    add_compile_options($<$<CONFIG:Release>:-flto=auto>)
    add_link_options($<$<CONFIG:Release>:-flto=auto>)
else()
    message(WARNING "IPO/LTO 不被支持: ${ipo_error}")
endif()
#########################################################


#rog_map need
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

add_definitions(-DORIGIN_AT_CORNER)

# 查找依赖
find_package(ament_cmake_auto REQUIRED)
# ---- ROS 2 显式 find_package，创建 std_msgs::std_msgs 等现代 target ----
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(behaviortree_cpp REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ompl REQUIRED)
find_package(spdlog REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PCL REQUIRED)

# find_package(nlohmann_json REQUIRED)
# find_package(behaviortree_cpp REQUIRED)

# find_package(Boost REQUIRED COMPONENTS system filesystem thread)

#include_directories(/usr/local/include/osqp)

add_subdirectory(src/utils)
add_subdirectory(src/map)
add_subdirectory(src/planner/path_planning)
add_subdirectory(src/planner/traj_optimize)
add_subdirectory(src/planner/controller)
add_subdirectory(src/planner/replan)
add_subdirectory(src/ros2)


# 添加可执行文件
ament_auto_add_executable(ma_nav src/ma_nav.cpp)
set_target_properties(ma_nav PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE
)

target_link_libraries(ma_nav PUBLIC
  ${OMPL_LIBRARIES}
  spdlog::spdlog
  osqp::osqp
  OsqpEigen::OsqpEigen
  yaml-cpp::yaml-cpp
  utils
  3d_occ_map
  path_planning
  controller
  traj_optimize
  replan
  ros2
)

target_link_libraries(ma_nav PUBLIC
    rclcpp::rclcpp
    ${std_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
    ${std_srvs_LIBRARIES}
    ${pcl_conversions_LIBRARIES}
    tf2::tf2
    tf2_ros::tf2_ros
)


# 导出包
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
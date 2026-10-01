cmake_minimum_required(VERSION 3.16)

project(traj_optimize)

file(GLOB_RECURSE sources CONFIGURE_DEPENDS "src/*.cpp")

add_library(${PROJECT_NAME})

target_sources(${PROJECT_NAME} PRIVATE ${sources})

target_include_directories(${PROJECT_NAME} PUBLIC include)

target_link_libraries(${PROJECT_NAME} PRIVATE
    spdlog::spdlog
    Eigen3::Eigen
    utils
    path_planning
)
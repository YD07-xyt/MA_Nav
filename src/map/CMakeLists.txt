cmake_minimum_required(VERSION 3.16)

project(3d_occ_map)

file(GLOB_RECURSE sources CONFIGURE_DEPENDS "src/*.cpp")

add_library(${PROJECT_NAME})

target_sources(${PROJECT_NAME} PRIVATE ${sources})

target_include_directories(${PROJECT_NAME} PUBLIC include)

target_link_libraries(${PROJECT_NAME} PRIVATE
    spdlog::spdlog
    utils
    ${PCL_LIBRARIES}
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
)
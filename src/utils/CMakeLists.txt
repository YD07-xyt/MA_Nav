cmake_minimum_required(VERSION 3.16)

project(utils)

file(GLOB_RECURSE sources CONFIGURE_DEPENDS "src/*.cpp")

add_library(${PROJECT_NAME})

target_sources(${PROJECT_NAME} PRIVATE ${sources})

target_include_directories(${PROJECT_NAME} PUBLIC include)

target_link_libraries(${PROJECT_NAME} PRIVATE
    spdlog::spdlog
    yaml-cpp::yaml-cpp
)
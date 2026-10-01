cmake_minimum_required(VERSION 3.16)

project(controller)

include_directories(/usr/local/include/osqp)
include_directories(/usr/local/share/osqp)

file(GLOB_RECURSE sources CONFIGURE_DEPENDS "src/*.cpp")

include_directories(/usr/local/include/osqp)

add_library(${PROJECT_NAME})

target_sources(${PROJECT_NAME} PRIVATE ${sources})

target_include_directories(${PROJECT_NAME} PUBLIC include)
# target_include_directories(${PROJECT_NAME} PUBLIC /usr/local/include/osqp /usr/local/share/osqp     
#     /usr/local/share/osqp/codegen_files/inc/private
#     /usr/local/share/osqp/codegen_files/inc/public)
target_link_libraries(${PROJECT_NAME} 
    spdlog::spdlog
    utils
    libqpOASES.a
    Eigen3::Eigen
    osqp::osqp
    OsqpEigen::OsqpEigen
    traj_optimize
    path_planning
)
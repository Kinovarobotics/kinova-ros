cmake_minimum_required(VERSION 2.8.3)
project(kinova_robot)

include_directories(
    include/
)
link_directories(
    lib/
)

add_library(${PROJECT_NAME} SHARED
    src/kinova_robot.cpp
)

target_link_libraries(${PROJECT_NAME}
    kinova_comm
    kinova_cmd
)

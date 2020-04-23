cmake_minimum_required(VERSION 2.8.3)
project(vSegmapper)

add_definitions(-std=c++11 -DBENCHMARK_ENABLE)

find_package(catkin_simple 0.1.0 REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
    src/segmapper.cpp
	src/v_slam_worker.cpp
)

cs_add_executable(${PROJECT_NAME}_node src/segmapper_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

cs_install()
cs_export()


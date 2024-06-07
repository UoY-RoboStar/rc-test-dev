cmake_minimum_required(VERSION 3.0.2)
project(map_MM_suite_t19)
add_compile_options(-std=c++17 -DDEBUG)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sample
    map_MM_suite
)

catkin_package(CATKIN_DEPENDS
    roscpp
    sample
    map_MM_suite
)

include_directories(
    ../../../MM_suite/include
    ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node
	src/main.cpp
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    map_MM_suite_node
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    boost_program_options
    map_MM_suite_node
)

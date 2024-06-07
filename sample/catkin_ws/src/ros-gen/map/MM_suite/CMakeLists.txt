cmake_minimum_required(VERSION 3.0.2)
project(map_MM_suite)
add_compile_options(-std=c++17 -DDEBUG)

find_package(catkin REQUIRED COMPONENTS
	roscpp
sample
)

catkin_package(CATKIN_DEPENDS
    roscpp
sample
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_node
	src/message.cpp
	src/tracechecker.cpp
	src/xunit.cpp
	src/etbuilder.cpp
	src/executor.cpp
	src/etbuilder.cpp
	src/executor.cpp
	src/tester.cpp
	src/tracechecker.cpp
	usr/events.cpp
)

add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

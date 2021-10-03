cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 20)

project(robot_controller
		VERSION 2019.1.0
		DESCRIPTION "Controller"
		LANGUAGES CXX)

find_package(Cellworld_tools REQUIRED)
find_package(Easy-tcp REQUIRED)
find_package(Robot_lib)
find_package(Threads)


###
### MAIN LIBRARY SETUP
###

set( robot_controller
		src/console/robot_controller.cpp
		src/robot_controller_service.cpp)


SET(CMAKE_CXX_FLAGS  "-DC_GNU64=400 -DOS_LINUX -no-pie -Wall -Wextra")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -pthread")

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_library(robot_controller ${robot_controller})

target_link_libraries(robot_controller
		PUBLIC
		${CELLWORLD_TOOLS_LIBRARIES}
		easy-tcp ${OpenCV_LIBS})

target_include_directories(robot_controller
		SYSTEM INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>)

target_include_directories(robot_controller
		PRIVATE
		include )

add_executable(controller src/console/robot_controller.cpp)
target_link_libraries(controller robot_controller robot_lib)

###
### TESTS
###
find_package(CatchTests CONFIG QUIET)

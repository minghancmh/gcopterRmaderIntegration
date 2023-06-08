cmake_minimum_required(VERSION 2.8.3)

project(gcopter)

add_definitions(-std=c++11)


set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_finders")
find_package (CGAL REQUIRED)

option(USE_GUROBI "Use GUROBI as the solver" OFF) #if OFF, then it will use NLOPT as the solver


include(${CGAL_USE_FILE})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  decomp_util
  decomp_ros_utils
  separator
)

find_package(decomp_util REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    include
)

include_directories(${DECOMP_UTIL_INCLUDE_DIRS})

catkin_package()

add_library(cgal_utils STATIC src/cgal_utils)
target_include_directories (cgal_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(cgal_utils PUBLIC ${DECOMP_UTIL_LIBRARIES} ${catkin_LIBRARIES})

add_library(utils STATIC src/utils.cpp)
target_include_directories (utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(utils PUBLIC ${catkin_LIBRARIES})

add_library(octopus_search STATIC src/octopus_search.cpp src/bspline_utils.cpp src/utils.cpp) 
target_include_directories (octopus_search PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(octopus_search PUBLIC ${catkin_LIBRARIES}) 

find_package(NLOPT REQUIRED)

add_library(solver STATIC src/solver_nlopt.cpp src/bspline_utils.cpp src/nlopt_utils.cpp )
target_compile_definitions(solver PUBLIC USE_GUROBI_FLAG=0) 
target_include_directories (solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${DECOMP_UTIL_INCLUDE_DIRS})
target_link_libraries(solver PUBLIC ${NLOPT_LIBRARIES} ${DECOMP_UTIL_LIBRARIES} utils octopus_search ${catkin_LIBRARIES} cgal_utils)

add_executable(single_obstacle src/single_obstacle.cpp)
add_dependencies(single_obstacle ${catkin_EXPORTED_TARGETS})
target_link_libraries(single_obstacle ${catkin_LIBRARIES} utils cgal_utils   ${OMPL_LIBRARIES}
${catkin_LIBRARIES})

add_executable(test_nlopt src/test_nlopt.cpp)
add_dependencies(test_nlopt ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_nlopt ${catkin_LIBRARIES} ${OMPL_LIBRARIES} utils solver ${OMPL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(global_planning src/global_planning.cpp)

target_link_libraries(global_planning
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# unset(USE_GUROBI CACHE)

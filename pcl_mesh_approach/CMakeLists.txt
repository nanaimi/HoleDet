cmake_minimum_required(VERSION 3.21 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_OSX_ARCHITECTURES "arm64")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBoost_USE_MULTITHREADED=OFF -pthread -lboost_thread")
set(CMAKE_PREFIX_PATH "/opt/homebrew/Cellar/qt@5/5.15.2_2/lib/cmake")

project(pcl_mesh_approach)

find_package(PCL REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CGAL_INCLUDE_DIRS})
include_directories(${PROJECT_NAME} ${Boost_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${CGAL_LIBRARY_DIRS})
link_directories(${PROJECT_NAME} ${Boost_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_definitions(${CGAL_DEFINITIONS})

add_library(Helpers lib/helpers.cpp)

add_executable(pcl_mesh_approach src/main.cpp)

target_link_libraries(pcl_mesh_approach ${PCL_LIBRARIES})
target_link_libraries(pcl_mesh_approach ${Boost_LIBRARIES})
target_link_libraries(pcl_mesh_approach ${CGAL_LIBRARIES})

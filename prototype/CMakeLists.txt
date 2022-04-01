cmake_minimum_required(VERSION 3.21)
project(prototype)

set(CMAKE_CXX_STANDARD 14)
find_package(PCL 1.3 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(prototype main.cpp)

target_link_libraries (prototype ${PCL_LIBRARIES})
target_link_libraries (prototype Eigen3::Eigen)
target_link_libraries( prototype ${OpenCV_LIBS} )



cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
find_package(PCL REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(Utils src/holedet_utils.cpp)
target_include_directories(Utils PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(Utils ${CMAKE_CURRENT_SOURCE_DIR}/include/holedet_utils.h)
target_link_libraries(Utils ${PCL_LIBRARIES})


cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

include(GenerateExportHeader)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(Utils SHARED src/holedet_utils.cpp include/holedet_utils.h)
target_link_libraries(Utils ${PCL_LIBRARIES})
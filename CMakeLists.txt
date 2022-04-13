cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(HoleDet)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libs)
find_package(PCL REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(HoleDet src/main.cpp src/HoleDetector.cpp)

target_include_directories(HoleDet PUBLIC ${PCL_INCLUDE_DIRS})
target_include_directories(HoleDet PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/libs)
target_include_directories(HoleDet PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(HoleDet ${PCL_LIBRARIES})
target_link_libraries(HoleDet Utils)

target_link_directories(HoleDet PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/libs)
target_link_directories(HoleDet PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

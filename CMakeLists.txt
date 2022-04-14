cmake_minimum_required(VERSION 3.21 FATAL_ERROR)

project(HoleDet)

set(CMAKE_CXX_STANDARD 14)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libs)

add_executable(HoleDet src/main.cpp src/HoleDetector.cpp)

target_include_directories(HoleDet PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/libs/include)
target_include_directories(HoleDet PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(HoleDet Utils)

target_link_directories(HoleDet PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/libs)
target_link_directories(HoleDet PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

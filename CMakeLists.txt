cmake_minimum_required(VERSION 3.21 FATAL_ERROR)

project(HoleDet)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libs)

find_package(PkgConfig REQUIRED)
pkg_check_modules(yaml_cpp REQUIRED yaml-cpp)

#add_definitions(${YAML_CPP_LIBRARIES})

add_executable(HoleDet src/main.cpp src/HoleDetector.cpp)

target_include_directories(HoleDet PUBLIC ${YAML_CPP_INCLUDE_DIRS})
target_include_directories(HoleDet PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/libs/include)
target_include_directories(HoleDet PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} yaml-cpp)
target_link_libraries(HoleDet Utils)

target_link_directories(HoleDet PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/libs)
target_link_directories(HoleDet PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)


cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(HoleDet)
add_subdirectory(libs)
find_package( Boost REQUIRED)

include_directories(${PROJECT_NAME} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(HoleDet src/main.cpp src/HoleDetector.cpp)

target_include_directories(HoleDet PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries (HoleDet PUBLIC Utils)
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})
target_link_libraries(HoleDet PUBLIC ${PCL_LIBRARIES})

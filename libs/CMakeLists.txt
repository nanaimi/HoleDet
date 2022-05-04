cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

find_package(PCL REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV REQUIRED)


add_definitions(${PCL_DEFINITIONS})

add_library(Utils SHARED src/holedet_utils.cpp include/holedet_utils.h src/Normal2dEstimation.cpp include/Normal2dEstimation.h src/PCA2D.cpp include/PCA2D.h)

target_include_directories(Utils PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(Utils PUBLIC ${PCL_INCLUDE_DIRS})
target_include_directories(Utils PRIVATE include/)

target_link_directories(Utils PUBLIC ${PCL_LIBRARY_DIRS})

target_link_libraries(Utils ${PCL_LIBRARIES})
target_link_libraries(Utils Eigen3::Eigen)
target_link_libraries(Utils ${OpenCV_LIBS})
cmake_minimum_required(VERSION 2.8)
project(mypcp1)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++11 -O3 -fopenmp -pthread")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Eigen
include_directories("/usr/include/eigen3")
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})

# pcl
find_package(PCL REQUIRED COMPONENT common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

### mylib
include_directories(${PROJECT_SOURCE_DIR}/include)
add_library(mypcp1
        src/point.cpp
        src/pointcloud.cpp
        src/pcasolver.cpp
        src/voxelgridfilter.cpp)

# executable
add_executable(pca_pointcloud src/pca_pointcloud.cpp)
target_link_libraries(pca_pointcloud ${PCL_LIBRARIES} mypcp1)

add_executable(vgf_pointcloud src/vgf_pointcloud.cpp)
target_link_libraries(vgf_pointcloud ${PCL_LIBRARIES} mypcp1)

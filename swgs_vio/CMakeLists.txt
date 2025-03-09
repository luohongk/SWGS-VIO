cmake_minimum_required(VERSION 2.8.3)
project(swgs_vio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    camera_models
    cv_bridge
    roslib
    )

find_package(Ceres REQUIRED PATHS "/home/lhk/Thirdparty/Ceres")
find_package(Eigen3)

set(ONNXRUNTIME_ROOTDIR "/home/lhk/Thirdparty/onnxruntime")
include_directories("${ONNXRUNTIME_ROOTDIR}/include")
link_directories("${ONNXRUNTIME_ROOTDIR}/lib")

find_package(JPEG)
find_package(PNG)
find_package(OpenCV REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}  
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}include/
    )

# Add libraries
add_library(swgs_vio src/swgs_vio.cpp)    
    
link_libraries(
    swgs_vio
    ${OpenCV_LIBRARIES}
    )
catkin_package()

add_executable(swgs_vio_node main.cpp)
target_link_libraries(swgs_vio_node) 

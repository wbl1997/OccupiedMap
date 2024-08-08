cmake_minimum_required(VERSION 2.8.3)
project(pcd2map)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# 找到 catkin 和需要的组件
# set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    pcl_ros
    visualization_msgs
    )

# find_package(OpenCV REQUIRED)
find_package(OpenCV 4.2.0)
find_package(PCL 1.8 REQUIRED)

find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package( )

# 声明一个 C++ 可执行文件
add_executable(pcd2map src/pcd2map_node.cpp
                       src/point2pgm.cpp)

# 链接库文件
target_link_libraries(pcd2map ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

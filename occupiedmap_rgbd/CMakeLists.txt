cmake_minimum_required(VERSION 3.0.2)
project(occupiedmap_rgbd)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 找到 catkin 和需要的组件
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

# 找到 PCL 包
find_package(PCL 1.7 REQUIRED)  # 明确指定所需的 PCL 版本

# 包括头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 添加 PCL 的定义和库目录
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})  # 确保PCL库目录被添加

# 设置可执行文件的生成路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME})

add_executable(pointcloud_to_pointcloud2 src/pointcloud_to_pointcloud2.cpp)
add_executable(occupymap_rgbd src/occupymap_rgbd.cpp)

add_dependencies(pointcloud_to_pointcloud2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(occupymap_rgbd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pointcloud_to_pointcloud2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(occupymap_rgbd ${catkin_LIBRARIES} ${PCL_LIBRARIES})

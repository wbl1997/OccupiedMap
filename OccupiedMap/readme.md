# OccupiedMap 项目说明

## 简介
这个项目包括两个主要的部分：`readpose` 和 `slam2d`。`readpose` 负责将 TUM 真值数据转换为 ROS 的 `/odometry` 消息格式；而 `slam2d` 则负责将来自多个传感器的点云数据转换到世界坐标系下。

## 依赖
本项目依赖于以下 ROS 包和 PCL 库：
- pcl_conversions
- pcl_ros
- roscpp
- sensor_msgs
- PCL 1.7
- slamtoolbox
- pointcloud_to_laserscan

请确保您的系统已安装以上依赖。

## 编译
本项目使用 `catkin` 进行构建。在克隆项目后，请按照以下步骤编译：

1. 将项目文件夹放入您的 catkin 工作空间中的 `src` 目录。
2. 打开终端并导航到工作空间的根目录。
3. 运行以下命令来编译项目：
   ```bash
   catkin_make

## 运行
在编译完成后，进入`OccupiedMap/launch`文件夹中，修改文件`radar.launch`中的参数`tum_file_path`以及需要播放的bag的地址
然后您可以使用以下命令来运行各部分：
   ```bash
   source /devel/setup.bash
   roslaunch OccupiedMap radar.launch


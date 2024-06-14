#!/bin/bash

# 使用示例: ./run_with_auto_complete.sh 0528 data1

# 检查传入参数数量
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <date> <data>"
  exit 1
fi

DATE=$1
DATA=$2

# 自动补全bag路径和TUM路径
BAG_DIR="/home/levy/data/${DATE}"
SEARCH_PATTERN="${BAG_DIR}/${DATA}_2024-${DATE:0:2}-${DATE:2:2}-*.bag"
BAG_PATH=$(ls $SEARCH_PATTERN 2>/dev/null | head -n 1)
TUM_PATH="/home/levy/pcl_ws/src/pcl_01/data/${DATE}/${DATA}_gt.txt"


# 检查文件是否存在
if [ -z "$BAG_PATH" ]; then
  echo "Bag file not found for ${DATE} and ${DATA}"
  exit 1
fi

if [ ! -f "$TUM_PATH" ]; then
  echo "TUM file not found at $TUM_PATH"
  exit 1
fi

# 打印路径供调试
echo "BAG_PATH: $BAG_PATH"
echo "TUM_PATH: $TUM_PATH"

# 修改1.launch文件
LAUNCH_FILE="/home/levy/pcl_ws/src/pcl_01/launch/1.launch"

sed -i "s|<param name=\"tum_file_path\" value=\".*\"/>|<param name=\"tum_file_path\" value=\"$TUM_PATH\"/>|" $LAUNCH_FILE
sed -i "s|<node name=\"rosbag_play\" pkg=\"rosbag\" type=\"play\" args=\".*\" output=\"screen\"/>|<node name=\"rosbag_play\" pkg=\"rosbag\" type=\"play\" args=\"$BAG_PATH\" output=\"screen\"/>|" $LAUNCH_FILE

# 启动launch文件
roslaunch pcl_01 1.launch


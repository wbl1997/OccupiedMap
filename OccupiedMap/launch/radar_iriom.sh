gnome-terminal -x bash -c "roscore" & sleep 2

gnome-terminal -x bash -c "
source /home/wbl/code/4yuekaifa/occu_map_ws/devel/setup.bash
roslaunch /home/wbl/code/4yuekaifa/occu_map_ws/src/OccupiedMap/launch/radar_iriom.launch
" & sleep 2

gnome-terminal -x bash -c "
source /home/wbl/code/4yuekaifa/lidar_slam/fasterlio_lc_ws/devel/setup.bash
roslaunch faster_lio mapping_mid360.launch
" & sleep 2

# gnome-terminal -x bash -c "
# rosbag play /home/wbl/code/radar_lidar_imu_calib/data/0722/data3_*.bag -r 1 -s 0
# "

# gnome-terminal -x bash -c "
# rosbag play /home/wbl/code/radar_lidar_imu_calib/data/0722/data3_*.bag -r 1 -s 0
# "

gnome-terminal -x bash -c "
rosbag play /home/wbl/code/radar_lidar_imu_calib/data/0715/data1*.bag -r 1 -s 0
"
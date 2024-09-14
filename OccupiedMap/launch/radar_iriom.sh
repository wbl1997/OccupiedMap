gnome-terminal -x bash -c "roscore" & sleep 2

gnome-terminal -x bash -c "
source /home/levy/occu_map_ws/devel/setup.bash
roslaunch /home/levy/occu_map_ws/src/OccupiedMap/launch/radar_iriom.launch
" & sleep 2

gnome-terminal -x bash -c "
source /home/levy/fastlio_ws/devel/setup.bash
roslaunch fast_lio mapping_mid360.launch
" & sleep 2

gnome-terminal -x bash -c "
rosbag play /home/levy/data/0715/data2_*.bag -r 1 -s 0
"
#!/bin/bash
 
gnome-terminal -t "calibration_fin" -x bash -c "docker exec -it SPTG-LCC bash -c 'cd /calib_data/direct_lidar_camera; \
    source devel/setup.bash; rosrun direct_visual_lidar_calibration calibrate --data_path /calib_data/SPTG-LCC/data; exec bash';"

#!/bin/bash

# 1. 启动 roscore 并保持终端打开
gnome-terminal -t "start_ros" -x bash -c  "docker exec -it SPTG-LCC bash -c 'source /opt/ros/noetic/setup.bash;roscore; exec bash'"
# 睡眠1s
sleep 1s

# 2. 运行不同的命令
# 第一个 Python 脚本
gnome-terminal -t "mono_depth" -x bash -c  "docker exec -it SPTG-LCC bash -c 'source /opt/ros/noetic/setup.bash; source /opt/anaconda3/etc/profile.d/conda.sh;source /opt/anaconda3/bin/activate lightglue;cd /calib_data/mono_depth/Marigold;python run_sptg.py; exec bash';"

sleep 2s

# 第二个 Python 脚本
gnome-terminal -t "LightGlue" -x bash -c  "docker exec -it SPTG-LCC bash -c 'cd /calib_data/matcher/LightGlue;source /opt/ros/noetic/setup.bash; source /opt/anaconda3/etc/profile.d/conda.sh;source /opt/anaconda3/bin/activate lightglue; python infer_Lightglue.py; exec bash';"

sleep 2s

# 第三个 Python 脚本
gnome-terminal -t "Efficinet_LOFTR" -x bash -c  "docker exec -it SPTG-LCC bash -c 'cd /calib_data/matcher/Efficinet_LOFTR/EfficientLoFTR; source /opt/ros/noetic/setup.bash; source /opt/anaconda3/etc/profile.d/conda.sh;source /opt/anaconda3/bin/activate lightglue; python infer_EffLoFTR.py; exec bash';"

sleep 2s

# 启动 ROS 启动文件
gnome-terminal -t "calibration_pre" -x bash -c  "docker exec -it SPTG-LCC bash -c 'cd /calib_data/direct_lidar_camera; \
    source devel/setup.bash; roslaunch direct_visual_lidar_calibration livox_mid360.launch; exec bash';"

sleep 1s

# 运行 ROS 节点
gnome-terminal -t "calibration_fin" -x bash -c "docker exec -it SPTG-LCC bash -c 'cd /calib_data/direct_lidar_camera; \
    source devel/setup.bash; rosrun direct_visual_lidar_calibration initial_guess_auto --data_path /calib_data/SPTG-LCC/data; exec bash';"


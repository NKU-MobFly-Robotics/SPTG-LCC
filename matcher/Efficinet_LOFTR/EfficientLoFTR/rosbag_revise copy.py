#!/home/wyw/anaconda3/envs/lightglue/bin/python3
import rosbag
from sensor_msgs.msg import CameraInfo

# 1-15循环
for i in range(1, 16):
    input_bag_file = '/home/wyw/ROS1_PROJECT/BD/2023/Lidar_camera_calib/bag/rosbag_2023_03_28-16_25_54.bag'
    output_bag_file_t = f'/home/wyw/ROS1_PROJECT/BD/2023/Lidar_camera_calib/bag/output_{i}.bag'

    new_fx = 437.4979248046875
    new_fy = 437.4979248046875
    new_cx = 512.0
    new_cy = 272.0
    new_d = [0, 0, 0, 0, 0.0]

    with rosbag.Bag(input_bag_file, 'r') as bag, rosbag.Bag(output_bag_file_t, 'w') as new_bag:
        for topic, msg, t in bag.read_messages():
            # 将所有原始消息写入新bag文件
            new_bag.write(topic, msg, t)
            
            # 对于`/camera_info`话题，创建并写入新的话题消息
            if topic == '/camera_info':
                new_msg = CameraInfo()
                # new_msg.header = msg.header
                # new_msg.height = msg.height
                # new_msg.width = msg.width
                new_msg.distortion_model = msg.distortion_model

                # 修改内参矩阵K
                new_msg.K = [new_fx, 0.0, new_cx, 0.0, new_fy, new_cy, 0.0, 0.0, 1.0]

                # 修改畸变系数D
                new_msg.D = new_d

                # new_msg.R = msg.R
                # new_msg.P = msg.P
                # new_msg.binning_x = msg.binning_x
                # new_msg.binning_y = msg.binning_y
                # new_msg.roi = msg.roi

                # 写入新的CameraInfo消息到新的话题
                new_topic = f'/new_camera_info_{i}'
                new_bag.write(new_topic, new_msg, t)

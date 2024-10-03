#!/home/wyw/anaconda3/envs/lightglue/bin/python3
import rosbag
from sensor_msgs.msg import CameraInfo
 #写一个1-15的循环
for i in range(1, 50):
#   if i%4==0:  
    input_bag_file = '/media/wyw/My Passport1/kitt1/kitti_2011_10_03_drive_0034_filtered/part_'+str(i)+'.bag'
    output_bag_file_t = '/media/wyw/My Passport1/kitt1/outputkitti_2011_10_03_drive_0034_filtered/'+str(i)+'.bag'

    new_fx = 7.188560e+02
    new_fy = 7.188560e+02 
    new_cx = 6.071928e+02
    new_cy = 1.852157e+02
    # new_fx = 7.070912e+02 
    # new_fy = 7.070912e+02 
    # new_cx = 6.018873e+02
    # new_cy = 1.831104e+02   
    new_d = [0, 0, 0, 0, 0.0] 

    camera_info_messages = []
    other_messages = []
    flag_bag = 19
    timestamp_set = set()

    with rosbag.Bag(input_bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == '/kitti/camera_color_left/camera_info':
                new_msg = CameraInfo()
                new_msg.header = msg.header
                new_msg.height = msg.height
                new_msg.width = msg.width
                new_msg.distortion_model = msg.distortion_model

                # 修改内参矩阵K
                new_msg.K = [new_fx, 0.0, new_cx, 0.0, new_fy, new_cy, 0.0, 0.0, 1.0]
                
                # 修改畸变系数D
                new_msg.D = new_d

                new_msg.R = msg.R
                new_msg.P = msg.P
                new_msg.binning_x = msg.binning_x
                new_msg.binning_y = msg.binning_y
                new_msg.roi = msg.roi

                camera_info_messages.append((topic, new_msg, t))
                timestamp_set.add(t)
            else:
                other_messages.append((topic, msg, t))

        
           
        

        with rosbag.Bag(output_bag_file_t, 'w') as new_bag:
                    # 写入相机内参消息
                    for topic, msg, t in camera_info_messages:
                        new_bag.write(topic, msg, t)

                    # 写入时间戳匹配的其他消息
                    for topic, msg, t in other_messages:
                            new_bag.write(topic, msg, t)

                    flag_bag += 1
                    camera_info_messages.clear()
                    other_messages.clear()
                    timestamp_set.clear()



 
import rosbag
import os
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, PointCloud2
import cv2
from cv_bridge import CvBridge
import rosbag
import os
import rospy
import numpy as np
def split_bag(bagfile, output_dir, n_parts):
    bridge = CvBridge()
    try:
        bag = rosbag.Bag(bagfile)
    except Exception as e:
        print(f"Error opening bag file: {e}")
        return
    
    try:
        duration = bag.get_end_time() - bag.get_start_time()
        part_duration = duration / n_parts
    except Exception as e:
        print(f"Error calculating duration: {e}")
        bag.close()
        return

    for i in range(n_parts):
        start_time = bag.get_start_time() + i * part_duration
        end_time = start_time + part_duration
        output_file = os.path.join(output_dir, f'128_part_{i + 1}.bag')
        
        if i % 1 == 0:
            with rosbag.Bag(output_file, 'w') as outbag:
                for topic, msg, t in bag.read_messages(start_time=rospy.Time.from_sec(start_time), end_time=rospy.Time.from_sec(end_time)):
                    if topic == '/stereo/vehicle_frame_left/image_raw/compressed':
                        # 解压缩图像
                        np_arr = np.fromstring(msg.data, np.uint8)
                        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                        img_msg = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                        outbag.write('/stereo/vehicle_frame_left/image_raw', img_msg, t)
                    elif topic == '/stereo/vehicle_frame_left/camera_info':
                        outbag.write('/stereo/vehicle_frame_left_image_raw/camera_info', msg, t)
                    elif topic == '/os_cloud_node/points':
                        outbag.write('/osve_cloud_node/points', msg, t)
    
    bag.close()

if __name__ == '__main__':
    # 直接在脚本中定义参数
    bagfile = '/media/wyw/My Passport/ros_dataset/128_ouster/vehicle_campus01.bag'
    output_dir = '/media/wyw/My Passport/ros_dataset/128_ouster/vehicle_campus01_data'
    n_parts = 200

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    split_bag(bagfile, output_dir, n_parts)

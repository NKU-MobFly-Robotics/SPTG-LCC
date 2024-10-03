#!/home/wyw/anaconda3/envs/lightglue/bin/python3
import rospy
import rosbag
from sensor_msgs.msg import CompressedImage, CameraInfo, PointCloud2, Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
import os
import time
import cv2
from cv_bridge import CvBridge
import threading

# 用于同步数据的锁
data_lock = threading.Lock()
# 存储最新的一帧数据
latest_data = None

# 回调函数，当三个话题同步时调用
def callback(compressed_image_msg, camera_info_msg, point_cloud_msg):
    global latest_data
    with data_lock:
        latest_data = (compressed_image_msg, camera_info_msg, point_cloud_msg)

def save_bag():
    global latest_data
    while not rospy.is_shutdown():
        rospy.sleep(1)  # 每秒钟执行一次

        with data_lock:
            if latest_data is None:
                continue
            compressed_image_msg, camera_info_msg, point_cloud_msg = latest_data
            latest_data = None  # 清空最新数据以等待下一帧

        # 创建一个新的 bag 文件
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        bag_filename = os.path.join("/media/wyw/My Passport/kitt1/output", f"data_{timestamp}.bag")
        
        # 使用 CvBridge 将 CompressedImage 转换为 Image
        bridge = CvBridge()
        image_cv = bridge.compressed_imgmsg_to_cv2(compressed_image_msg, "bgr8")
        image_msg = bridge.cv2_to_imgmsg(image_cv, "bgr8")
        
        with rosbag.Bag(bag_filename, 'w') as bag:
            bag.write('/stereo/vehicle_frame_left/image_raw', image_msg)
            bag.write('/stereo/vehicle_frame_left/camera_info', camera_info_msg)
            bag.write('/os_cloud_node/points', point_cloud_msg)
        
        rospy.loginfo(f"Saved bag file: {bag_filename}")

def main():
    rospy.init_node('data_synchronizer', anonymous=True)

    # 创建订阅者
    image_sub = Subscriber('/stereo/vehicle_frame_left/image_raw/compressed', CompressedImage)
    camera_info_sub = Subscriber('/stereo/vehicle_frame_left/camera_info', CameraInfo)
    point_cloud_sub = Subscriber('/os_cloud_node/points', PointCloud2)

    # 创建同步器，使用 ApproximateTimeSynchronizer 进行时间近似同步
    ats = ApproximateTimeSynchronizer([image_sub, camera_info_sub, point_cloud_sub], queue_size=1, slop=0.05)
    ats.registerCallback(callback)

    # 创建并启动保存bag文件的线程
    save_thread = threading.Thread(target=save_bag)
    save_thread.start()

    rospy.spin()
    save_thread.join()  # 等待保存线程结束

if __name__ == '__main__':
    main()

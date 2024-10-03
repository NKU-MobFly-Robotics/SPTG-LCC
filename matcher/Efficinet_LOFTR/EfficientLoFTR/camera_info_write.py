#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo

def publish_camera_info():
    # 初始化ROS节点
    rospy.init_node('camera_info_publisher', anonymous=True)
    
    # 定义发布者，将CameraInfo消息发布到/camera_info话题
    pub = rospy.Publisher('/camera_info', CameraInfo, queue_size=10)
    
    # 设置发布频率（每20秒发布一次）
    rate = rospy.Rate(0.05)  # 0.05 Hz (每20秒一次)
    
    # 创建CameraInfo消息
    camera_info_msg = CameraInfo()

    # 设置相机内参（示例值）
    camera_info_msg.width = 640
    camera_info_msg.height = 480
    
    # 3x3 内参矩阵 K
    camera_info_msg.K = [588.9817208252855, 0.0, 324.0004924589568, 
                         0.0, 590.2390640395754, 224.0668563262575, 
                         0.0, 0.0, 1.0]  # fx, 0, cx, 0, fy, cy, 0, 0, 1

    # 5个畸变系数 D
    camera_info_msg.D = [0.12738026964577068, -0.2475383368037499, 
                         0.000549771541173375, 0.002756883813194011, 
                         0.00000]  # k1, k2, p1, p2, k3

    # 旋转矩阵 R (3x3)
    camera_info_msg.R = [1.0, 0.0, 0.0,  # 假设没有旋转
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]

    # 投影矩阵 P (3x4)
    camera_info_msg.P = [588.9817208252855, 0.0, 324.0004924589568, 0.0,  # fx, 0, cx, Tx
                         0.0, 590.2390640395754, 224.0668563262575, 0.0,  # 0, fy, cy, Ty
                         0.0, 0.0, 1.0, 0.0]  # 0, 0, 1, 0

    # 相机的畸变模型
    camera_info_msg.distortion_model = "plumb_bob"  # 通常的模型为"plumb_bob"
    
    # 循环持续发布消息
    while not rospy.is_shutdown():
        # 发布消息
        pub.publish(camera_info_msg)
        rospy.loginfo("Published camera info with model")
        
        # 按照设定的频率休眠
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera_info()
    except rospy.ROSInterruptException:
        pass

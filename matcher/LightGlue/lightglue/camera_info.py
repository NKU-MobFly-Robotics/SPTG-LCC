import rospy
from sensor_msgs.msg import CameraInfo

def publish_camera_info():
    pub = rospy.Publisher('camera_info', CameraInfo, queue_size=10)
    rospy.init_node('camera_info_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
 
    info = CameraInfo()
    info.header.frame_id = "camera_frame"
    info.width = 1860  # 修改为你的相机分辨率
    info.height = 1080  # 修改为你的相机分辨率
    info.distortion_model = "plumb_bob"
    info.D = [-0.36565734884394796,0.19454341750019136, -0.0003778025271473721, 9.299129873097343e-05,-0.0637399451159775]  # 修改为你的畸变参数
    info.K = [1215.16206202384, 0.0,  931.46704145638, 0.0,   1215.5773507687247,  541.5467734550551, 0.0, 0.0, 1.0]  # 修改为你的焦距和光心
    info.P = [1215.16206202384, 0.0, 931.46704145638, 0.0, 0.0,  1215.5773507687247, 541.5467734550551, 0.0, 0.0, 0.0, 1.0, 0.0]  # 修改为你的焦距和光心

    while not rospy.is_shutdown():
        info.header.stamp = rospy.Time.now()
        pub.publish(info)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera_info()
    except rospy.ROSInterruptException:
        pass
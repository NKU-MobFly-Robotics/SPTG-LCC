import rosbag
import os
import rospy

def split_bag(bagfile, output_dir, n_parts):
    bag = rosbag.Bag(bagfile)
    duration = bag.get_end_time() - bag.get_start_time()
    part_duration = duration / n_parts
    
    for i in range(n_parts):
        start_time = bag.get_start_time() + i * part_duration
        end_time = start_time + part_duration
        output_file = os.path.join(output_dir, f'part_2_00_{i + 1}.bag')
        if i>52 and i<60:
         with rosbag.Bag(output_file, 'w') as outbag:
            for topic, msg, t in bag.read_messages(start_time=rospy.Time.from_sec(start_time), end_time=rospy.Time.from_sec(end_time)):
                outbag.write(topic, msg, t)
    
    bag.close()

if __name__ == '__main__':
    # 直接在脚本中定义参数
    bagfile = '/media/wyw/Extreme SSD/ros_dataset/128_ouster/vehicle_multilayer00.bag'
    output_dir = '/media/wyw/Extreme SSD/ros_dataset/128_ouster/vehicle_multilayer00/'
   
    n_parts =120

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    split_bag(bagfile, output_dir, n_parts)

import os
import shutil
import random

# 设置原始文件夹路径和目标文件夹路径
source_folder = '/media/wyw/My Passport/ros_dataset/128_ouster/vehicle_campus01_data'
destination_folder = '/media/wyw/My Passport/ros_dataset/128_ouster/vehicle_campus01_data_split'

# 创建目标文件夹，如果不存在的话
if not os.path.exists(destination_folder):
    os.makedirs(destination_folder)

# 获取所有bag文件
bags = [f for f in os.listdir(source_folder) if f.endswith('.bag')]

# 打乱文件列表的顺序
random.shuffle(bags)

# 将文件列表分组
groups = [bags[i:i + 10] for i in range(0, 200, 10)]

# 将每组文件保存到新的子文件夹中
for idx, group in enumerate(groups):
    group_folder = os.path.join(destination_folder, f'Group_{idx + 1}')
    if not os.path.exists(group_folder):
        os.makedirs(group_folder)
    
    for file_name in group:
        source_path = os.path.join(source_folder, file_name)
        destination_path = os.path.join(group_folder, file_name)
        shutil.copy2(source_path, destination_path)

    print(f"Group {idx + 1} files have been moved to {group_folder}")

print("All files have been successfully moved.")

#!/home/wyw/anaconda3/envs/lightglue/bin/python3
from lightglue import LightGlue, SuperPoint, DISK,viz2d,ALIKED
from lightglue.utils import load_image, rbd
import os
import glob
import time
from tqdm import tqdm
import numpy as np
import cv2
import torch
from email.mime import image
import sys
import cv2
import math
import json
import argparse 
import numpy
import rospy
import rospy
from std_msgs.msg import Int32
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
# 定义ros服务器
np.set_printoptions(precision=8 , suppress=True) ## 8 decimal places are reserved, and Scientific notation is not used
from pathlib import Path
import gc
data_root = '/calib_data/open-source/direct_lidar_camera/src/direct_visual_lidar_calibration/scripts/LightGlue/DATA/' #### data root
print('\033[93m' + '****************************************************************************************************' + '\033[0m')
print('\033[93m' + '* WARNING: You are going to use LightGlue that is not allowed to be used for commercial purposes!! *' + '\033[0m')
print('\033[93m' + '****************************************************************************************************' + '\033[0m')
data_path="/calib_data/open-source/SPTG-LCC/data"
GPU_id = 0
def handle_int_service(req):
    # 接收到请求后的处理逻辑
    service_proxy = rospy.ServiceProxy('/depth_predict', Empty)
        
        # 创建一个服务请求对象（在这种情况下，不需要参数）
    request2 = EmptyRequest()
        
        # 调用服务
    response2 = service_proxy(request2)
    # rospy.loginfo("Received request with data: %d", req.data)
    torch.cuda.empty_cache()
    gc.collect()
    lightglue_predict()
    
    torch.cuda.empty_cache()
    gc.collect()
    # 在这里进行任何你想要的处理逻辑
    # 这里我们简单地将请求数据加1，并作为响应发布出去
    response =EmptyResponse()
 
    # rospy.loginfo("Sending response with result: %d", response.result)
    return response

def angle_to_rot(angle, image_shape):
    width, height = image_shape[:2]

    if angle == 90:
      code = cv2.ROTATE_90_CLOCKWISE
      func = lambda x: numpy.stack([x[:, 1], width - x[:, 0]], axis=1)
    elif angle == 180:
      code = cv2.ROTATE_180
      func = lambda x: numpy.stack([height - x[:, 0], width - x[:, 1]], axis=1)
    elif angle == 270:
      code = cv2.ROTATE_90_COUNTERCLOCKWISE
      func = lambda x: numpy.stack([height - x[:, 1], x[:, 0]], axis=1)
    else:
      print('error: unsupported rotation angle %d' % angle)
      exit(1)

    return code, func
def numpy_image_to_torch(image: np.ndarray) -> torch.Tensor:
    """Normalize the image tensor and reorder the dimensions."""
    if image.ndim == 3:
        image = image.transpose((2, 0, 1))  # HxWxC to CxHxW
    elif image.ndim == 2:
        image = image[None]  # add channel axis
    else:
        raise ValueError(f"Not an image: {image.shape}")
    return torch.tensor(image / 255.0, dtype=torch.float)
def read_image(Path1, grayscale: bool = False) -> np.ndarray:
    """Read an image from path as RGB or grayscale"""
    if not Path(Path1).exists():
        raise FileNotFoundError(f"No image at path {Path1}.")
    mode = cv2.IMREAD_GRAYSCALE if grayscale else cv2.IMREAD_COLOR
    image = cv2.imread(str(Path1), mode)
    if image is None:
        raise IOError(f"Could not read image at {Path1}.")
    if not grayscale:
        image = image[..., ::-1]
    return image
def safe_cm_prune(data):
    if not data:  # Check if the data is empty
        return []  # Return an empty list or another default value
    return viz2d.cm_prune(data)



def lightglue_predict():
    extractor = SuperPoint(max_num_keypoints=None).eval().cuda(GPU_id)
      
        # .cuda(GPU_id)  # load the extractor
    matcher = LightGlue(features='superpoint').eval().cuda(GPU_id)
    with open(data_path + '/calib.json', 'r') as f:
        calib_config = json.load(f)
    for bag_name in calib_config['meta']['bag_names']:
        
        # camera_image =load_image('%s/%s.png' % (data_path, bag_name))
        # lidar_image = load_image('%s/%s_lidar_intensities.png' % (data_path, bag_name))
      #写一个执行两次的FOR循环
      for i in range(4):
        
        print('processing %s' % bag_name)
          
        if i == 0:
              image_name = '_lidar_intensities.png'
              json_name = '_ph'
        elif i == 1: 
              image_name = '_lidar_intensities_eq.png'
              json_name = '_eq'
        elif i == 2: 
              image_name = '_lidar_depth.png'
              json_name = '_depth'
        else:
              image_name = '_lidar_depth_eq.png'
              json_name = '_depth_eq'
     
        if i<2:
             lidar_image = read_image(data_path + "/" + bag_name + image_name) #_rgb
             camera_image =read_image(data_path+"/"+bag_name+'.png') #_rgb

        else:
            # img=cv2.imread(data_path + "/" + bag_name + "_rgb_pred.png",0) 
            # img = cv2.equalizeHist(img); 
            # cv2.imwrite(data_path+"/"+bag_name+'_rgb_pred_copy.png',img)
            lidar_image = read_image(data_path + "/" + bag_name + image_name)
            camera_image =read_image(data_path+"/"+bag_name+'_rgb_pred.png') #_rgb
            
        # code, camera_R_inv = angle_to_rot(180, lidar_image.shape)
        # lidar_image = cv2.rotate(lidar_image, code)
        lidar_image= numpy_image_to_torch(lidar_image)
        camera_image= numpy_image_to_torch(camera_image)
        print(lidar_image.shape)
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # 'mps', 'cpu'

        
        # .cuda(GPU_id)
        feats0 = extractor.extract(camera_image.cuda(GPU_id)   ,resize=None)  # auto-resize the image, disable with resize=None
        feats1 = extractor.extract(lidar_image.cuda(GPU_id)  ,resize=None)

            # match the features
        matches01 = matcher({'image0': feats0, 'image1': feats1})

        feats0, feats1, matches01 = [rbd(x) for x in [feats0, feats1, matches01]]  # remove batch dimension
        kpts0_, kpts1_  = feats0['keypoints'], feats1['keypoints'] 
        confidence = matches01['matching_scores0']
        matches_save = matches01['matches0']  # indices with shape (K,2)
 
        kpts0, kpts1  = feats0['keypoints'], feats1['keypoints'] 
       
        # kpts1_ = camera_R_inv(kpts1_.cpu().numpy())
        matches = matches01['matches']  # indices with shape (K,2)
        points0 = feats0['keypoints'][matches[..., 0]]  # coordinates in image #0, shape (K,2)
        points1 = feats1['keypoints'][matches[..., 1]]  # coordinates in image #1, shape (K,2)
          
        result = { 'kpts0': kpts0_.flatten().tolist(), 'kpts1': kpts1_.flatten().tolist(), 'matches': matches_save.flatten().tolist(), 'confidence': confidence.flatten().tolist() }
        with open('%s/%s_matches'% (data_path, bag_name)+json_name+'_lightGlue.json' , 'w') as f:
          json.dump(result, f)
        axes = viz2d.plot_images([camera_image, lidar_image])
        viz2d.plot_matches(points0, points1, color="lime", lw=0.2)
        viz2d.add_text(0, f'Stop after {matches01["stop"]} layers', fs=20)
        viz2d.save_plot("/calib_data/open-source/show/"+bag_name+image_name+"_lightglue.png")
        # kpc0 = safe_cm_prune(matches01.get("prune0"))
        # kpc1 = safe_cm_prune(matches01.get("prune1"))
        # kpc0, kpc1 = viz2d.cm_prune(matches01["prune0"]), viz2d.cm_prune(matches01["prune1"])
        # viz2d.plot_images([camera_image, lidar_image])
        # viz2d.save_plot("/home/wyw/ROS1_PROJECT/BD/2023/Lidar_camera_calib/show/"+bag_name+image_name+"_origin.png")
        # viz2d.plot_keypoints([kpts0, kpts1], colors=[kpc0, kpc1], ps=10)
        # viz2d.save_plot("/home/wyw/ROS1_PROJECT/BD/2023/Lidar_camera_calib/show/"+bag_name+image_name+"_point.png")
    del matcher, extractor
    torch.cuda.empty_cache()
    gc.collect()
if __name__ == '__main__':
    rospy.init_node('init_T_LiDAR_Camera_service_server_lightglue')
    # 定义一个名为 "int_service" 的服务，使用 IntService 消息类型，并指定处理函数为 handle_int_service
    s = rospy.Service('/server_lightglue', Empty, handle_int_service)
    rospy.loginfo("init_T_LiDAR_Camera_service_server Ready.")
    rospy.spin()
   





# method = "SuperPoint"
# # method = "DISK"

# frame = 1 # frame(1,5,10,20,30). "frame = n" means n frame by frame calculation

# ##### single scene folder inference(If you only want to operate on a single scene folder, name it yourself)
# # single , single_folder = False , ""
# single , single_folder = True , "1"

# GPU_id = 0  ## 0,1,2,3

# def get_all_folders(path):
#     folders = []
#     for item in os.listdir(path):
#         if os.path.isdir(os.path.join(path, item)):
#             folders.append(item)
#     return folders

# folders = get_all_folders(data_root)

# if method == "SuperPoint":
# # SuperPoint+LightGlue
#     extractor = SuperPoint(max_num_keypoints=2048).eval().cuda(GPU_id)  # load the extractor
#     matcher = LightGlue(features='superpoint').eval().cuda(GPU_id)
#     # extractor = SuperPoint(max_num_keypoints=None).eval().cuda(GPU_id)  # load the extractor
#     # matcher = LightGlue(features='superpoint', depth_confidence=-1, width_confidence=-1).eval().cuda(GPU_id)
#     a = "/SuperPoint_"
# else:
# # or DISK+LightGlue
# #     extractor = DISK(max_num_keypoints=None).eval().cuda(GPU_id)  # load the extractor
# #     matcher = LightGlue(features='disk', depth_confidence=-1, width_confidence=-1).eval().cuda(GPU_id)
#     extractor = DISK(max_num_keypoints=2048).eval().cuda(GPU_id)  # load the extractor
#     matcher = LightGlue(features='disk').eval().cuda(GPU_id)  # load the matcher
#     a = "/DISK_"

# for i in range(len(folders)):
#     if single and folders[i] != single_folder:
#         continue
#     point_folder = data_root + folders[i] + a + "point_" + str(frame) + "/"
#     # Check if the folder already exists, skip if it exists
#     if not os.path.exists(point_folder):
#         os.mkdir(point_folder)
#         print(f"folder '{point_folder}' created")
#     else:
#         print(f"folder '{point_folder}' already exists")
#         continue

#     rgb_path = data_root + folders[i] + '/images/'
#     rgb_files = glob.glob(os.path.join(rgb_path, '*.png'))
#     for j in tqdm(range(0,len(rgb_files)-frame,frame), desc = folders[i]):
#         print(f"processing {rgb_files[j].split('/')[-1]} and {rgb_files[j+frame].split('/')[-1]}")
#         point_txt = point_folder + rgb_files[j].split("/")[-1].split(".")[0] + "_" +\
#                     rgb_files[j+frame].split("/")[-1].split(".")[0] + ".txt"
#         # Check if the folder already exists, delete if it exists
#         if os.path.exists(point_txt):
#             os.remove(point_txt)
#         time.sleep(0.05)
#             # load each image as a torch.Tensor on GPU with shape (3,H,W), normalized in [0,1]
#         image0 = load_image(rgb_files[j])
#         image1 = load_image(rgb_files[j+frame])

#             # extract local features
#         feats0 = extractor.extract(image0.cuda(GPU_id),resize=None)  # auto-resize the image, disable with resize=None
#         feats1 = extractor.extract(image1.cuda(GPU_id),resize=None)

#             # match the features
#         matches01 = matcher({'image0': feats0, 'image1': feats1})

#         feats0, feats1, matches01 = [rbd(x) for x in [feats0, feats1, matches01]]  # remove batch dimension
#         confidence = matches01['matching_scores0']

        
        


#         print(f"confidence: {confidence.mean().item():.4f}")
#     # //////////////////////////////////////////////
#         kpts0, kpts1  = feats0['keypoints'], feats1['keypoints'] 
#         matches = matches01['matches']  # indices with shape (K,2)
#         points0 = feats0['keypoints'][matches[..., 0]]  # coordinates in image #0, shape (K,2)
#         points1 = feats1['keypoints'][matches[..., 1]]  # coordinates in image #1, shape (K,2)
       
#         result = { 'kpts0': kpts0.flatten().tolist(), 'kpts1': kpts1.flatten().tolist(), 'matches': matches.flatten().tolist(), 'confidence': confidence.flatten().tolist() }

#         axes = viz2d.plot_images([image0, image1])
#         viz2d.plot_matches(points0, points1, color="lime", lw=0.2)
#         viz2d.add_text(0, f'Stop after {matches01["stop"]} layers', fs=20)
#         viz2d.save_plot(data_root+"c.png")
#         kpc0, kpc1 = viz2d.cm_prune(matches01["prune0"]), viz2d.cm_prune(matches01["prune1"])
#         viz2d.plot_images([image0, image1])
#         viz2d.save_plot(data_root+"b.png")
#         viz2d.plot_keypoints([kpts0, kpts1], colors=[kpc0, kpc1], ps=10)
#         viz2d.save_plot(data_root+"a.png")
#         p0 = points0.tolist()[:-1]
#         p1 = points1.tolist()[:-1]
#         for k in range(len(p0)):
#             p0[k] = [str(round(z0)) for z0 in p0[k]]
#             p1[k] = [str(round(z1)) for z1 in p1[k]]
#         p = [" ".join(p0[i]+p1[i]) for i in range(len(p0))]
#         with open(point_txt, 'a') as f:
#             f.write("\n".join(p))



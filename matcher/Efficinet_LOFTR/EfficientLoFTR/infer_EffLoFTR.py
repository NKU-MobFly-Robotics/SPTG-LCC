import os
os.chdir("..")
from copy import deepcopy
import matplotlib.pyplot as plt
import torch
import cv2
import numpy as np
import matplotlib.cm as cm
from src.utils.plotting import make_matching_figure
import json
import rospy
import rospy
from std_msgs.msg import Int32
import gc
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
# 定义ros服务器
data_root = '/calib_data/open-source/direct_lidar_camera/src/direct_visual_lidar_calibration/scripts/LightGlue/DATA/' #### data root
print('\033[93m' + '****************************************************************************************************' + '\033[0m')
print('\033[93m' + '* WARNING: You are going to use LightGlue that is not allowed to be used for commercial purposes!! *' + '\033[0m')
print('\033[93m' + '****************************************************************************************************' + '\033[0m')
data_path="/calib_data/open-source/SPTG-LCC/data"
from src.loftr import LoFTR, full_default_cfg, opt_default_cfg, reparameter

# You can choose model type in ['full', 'opt']
model_type = 'full'  # 'full' for best quality, 'opt' for best efficiency

# You can choose numerical precision in ['fp32', 'mp', 'fp16']. 'fp16' for best efficiency
precision = 'fp32'  # Enjoy near-lossless precision with Mixed Precision (MP) / FP16 computation if you have a modern GPU (recommended NVIDIA architecture >= SM_70).

# You can also change the default values like thr. and npe (based on input image size)

if model_type == 'full':
    _default_cfg = deepcopy(full_default_cfg)
elif model_type == 'opt':
    _default_cfg = deepcopy(opt_default_cfg)

if precision == 'mp':
    _default_cfg['mp'] = True
elif precision == 'fp16':
    _default_cfg['half'] = True
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
device='cuda'
print(_default_cfg)

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
    response = EmptyResponse()

    # rospy.loginfo("Sending response with result: %d", response.result)
    return response

def lightglue_predict():
        torch.cuda.empty_cache()
        gc.collect()
        matcher = LoFTR(config=_default_cfg)

        matcher.load_state_dict(torch.load(
            "/calib_data/open-source/direct_lidar_camera/src/direct_visual_lidar_calibration/scripts/Efficinet_LOFTR/EfficientLoFTR/weights/eloftr_outdoor.ckpt")[
                                    'state_dict'])
        matcher = reparameter(matcher)  # no reparameterization will lead to low performance

        if precision == 'fp16':
            matcher = matcher.half()

        matcher = matcher.eval().to(device) 

        with open(data_path + '/calib.json', 'r') as f:
            calib_config = json.load(f)
        for bag_name in calib_config['meta']['bag_names']:
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

                if i < 2:
                    img0_pth =data_path + "/" + bag_name + image_name
                    img1_pth =data_path+"/"+bag_name+'.png'

                else:
                    img0_pth = data_path + "/" + bag_name + image_name
                    img1_pth = data_path+"/"+bag_name+'_rgb_pred.png'
                img0_raw = cv2.imread(img1_pth, cv2.IMREAD_GRAYSCALE)
                img1_raw = cv2.imread(img0_pth, cv2.IMREAD_GRAYSCALE)
                img00_raw = cv2.resize(img0_raw, (img0_raw.shape[1]//32*32, img0_raw.shape[0]//32*32))  # input size shuold be divisible by 32
                img11_raw = cv2.resize(img1_raw, (img1_raw.shape[1]//32*32, img1_raw.shape[0]//32*32))
                scale_x0 = img0_raw.shape[1] / img00_raw.shape[1]
                scale_y0 = img0_raw.shape[0] / img00_raw.shape[0]
                scale_x1 = img1_raw.shape[1] / img11_raw.shape[1]
                scale_y1 = img1_raw.shape[0] / img11_raw.shape[0]
                inverse_scale_x0 = 1 / scale_x0
                inverse_scale_y0 = 1 / scale_y0
                inverse_scale_x1 = 1 / scale_x1
                inverse_scale_y1 = 1 / scale_y1
                print(img0_raw.shape, img00_raw.shape, img1_raw.shape, img11_raw.shape)
                if precision == 'fp16':
                    img0 = torch.from_numpy(img00_raw)[None][None].half().to(device) / 255.
                    img1 = torch.from_numpy(img11_raw)[None][None].half().to(device)  / 255.
                else:
                    img0 = torch.from_numpy(img00_raw)[None][None].to(device)  / 255.
                    img1 = torch.from_numpy(img11_raw)[None][None].to(device)  / 255.
                batch = {'image0': img0, 'image1': img1}

        # Inference with EfficientLoFTR and get prediction
                with torch.no_grad():
                    if precision == 'mp':
                        with torch.autocast(enabled=True, device_type='cuda'):
                            matcher(batch)
                    else:
                        matcher(batch)
                    mkpts0 = batch['mkpts0_f'].cpu().numpy()
                    mkpts1 = batch['mkpts1_f'].cpu().numpy()
                    mconf = batch['mconf'].cpu().numpy()
                    matchs = np.arange(len(mkpts1))

                    # print(mkpts1)

                    mkpts0_original = mkpts0 * np.array([scale_x0, scale_y0])
                    mkpts1_original = mkpts1 * np.array([scale_x1, scale_y1])
                    # print(mkpts1_original)
                    # print(len(matchs),len(mkpts1),len(mconf))
                    result = {'kpts0': mkpts0_original.flatten().tolist(), 'kpts1': mkpts1_original.flatten().tolist(),
                              'matches': matchs.flatten().tolist(), 'confidence': mconf.flatten().tolist()}
                    with open('%s/%s_matches' % (data_path, bag_name) + json_name + '_EFLoFTR.json', 'w') as f:
                        json.dump(result, f)
        # Draw
                if model_type == 'opt':
                    print(mconf.max())
                    mconf = (mconf - min(20.0, mconf.min())) / (max(30.0, mconf.max()) - min(20.0, mconf.min()))

                color = cm.jet(mconf)
                text = [
                    'LoFTR',
                    'Matches: {}'.format(len(mkpts0)),
                ]
                fig = make_matching_figure(img00_raw, img11_raw, mkpts0, mkpts1, color, text=text)
                plt.savefig('/calib_data/open-source/direct_lidar_camera/src/direct_visual_lidar_calibration/scripts/Efficinet_LOFTR/EfficientLoFTR/show/'+bag_name+image_name+"_lightglue.png")
   
        torch.cuda.empty_cache()
        gc.collect()
if __name__ == '__main__':
                rospy.init_node('init_T_LiDAR_Camera_service_server')
                # 定义一个名为 "int_service" 的服务，使用 IntService 消息类型，并指定处理函数为 handle_int_service
                s = rospy.Service('/server_EffLoFTR', Empty, handle_int_service)
                rospy.loginfo("init_T_LiDAR_Camera_service_server Ready.")
                rospy.spin()


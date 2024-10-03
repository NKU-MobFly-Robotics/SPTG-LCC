# Copyright 2023 Bingxin Ke, ETH Zurich. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# --------------------------------------------------------------------------
# If you find this code useful, we kindly ask you to cite our paper in your work.
# Please find bibtex at: https://github.com/prs-eth/Marigold#-citation
# More information about the method can be found at https://marigoldmonodepth.github.io
# --------------------------------------------------------------------------


import argparse
import logging
import os
from glob import glob

import numpy as np
import torch
from PIL import Image
from tqdm.auto import tqdm

import json
from marigold import MarigoldPipeline
import rospy
import rospy
from std_msgs.msg import Int32
from std_srvs.srv import Empty, EmptyResponse
import gc
EXTENSION_LIST = [".jpg", ".jpeg", ".png"]
data_path="/home/wyw/ROS1_PROJECT/BD/2023/Lidar_camera_calib/data/"

def handle_int_service(req):
    # 接收到请求后的处理逻辑
    # rospy.loginfo("Received request with data: %d", req.data)
    # 在这里进行任何你想要的处理逻辑
    # 这里我们简单地将请求数据加1，并作为响应发布出去
    response = EmptyResponse()
    torch.cuda.empty_cache() 
    gc.collect()
    depth_predict()
    torch.cuda.empty_cache()
    gc.collect()
    # rospy.loginfo("Sending response with result: %d", response.result)
    return response

def depth_predict():
    checkpoint_path = "/home/wyw/ROS1_PROJECT/BD/2023/mono_depth/Marigold/marigold-lcm-v1-0"  # args.checkpoint
    input_rgb_dir = "/home/wyw/ROS1_PROJECT/BD/2023/mono_depth/Marigold/input"  # args.input_rgb_dir
    output_dir = "/home/wyw/ROS1_PROJECT/BD/2023/mono_depth/Marigold/output"  # args.output_dir
    with open(data_path + '/calib.json', 'r') as f:
        calib_config = json.load(f)

    rgb_filename_list=[]
    for bag_name in calib_config['meta']['bag_names']:
        img=data_path+bag_name+"_rgb_pred.png"
        if os.path.exists(img):
             continue
        else:
           rgb_filename_list.append(data_path+bag_name+"_rgb.png")

    if(rgb_filename_list!=[]) :
        denoise_steps = 4  # args.denoise_steps
        ensemble_size = 5  # args.ensemble_size
        if ensemble_size > 15:
            logging.warning("Running with large ensemble size will be slow.")
        half_precision = True
        processing_res = 768
        match_input_res = True
        resample_method = "bilinear"

        color_map = "Spectral"
        seed = None
        batch_size = 1
        apple_silicon = False
        if apple_silicon and 0 == batch_size:
            batch_size = 1  # set default batchsize

        # -------------------- Preparation --------------------
        # Print out config
        logging.info(
            f"Inference settings: checkpoint = `{checkpoint_path}`, "
            f"with denoise_steps = {denoise_steps}, ensemble_size = {ensemble_size}, "
            f"processing resolution = {processing_res}, seed = {seed}; "
            f"color_map = {color_map}."
        )

        # Output directories
        output_dir_color = os.path.join(output_dir, "depth_colored")
        output_dir_tif = data_path
        output_dir_npy = os.path.join(output_dir, "depth_npy")
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(output_dir_color, exist_ok=True)
        os.makedirs(output_dir_tif, exist_ok=True)
        os.makedirs(output_dir_npy, exist_ok=True)
        logging.info(f"output dir = {output_dir}")

        # -------------------- Device --------------------
        if apple_silicon:
            if torch.backends.mps.is_available() and torch.backends.mps.is_built():
                device = torch.device("mps:0")
            else:
                device = torch.device("cpu")
                logging.warning("MPS is not available. Running on CPU will be slow.")
        else:
            if torch.cuda.is_available():
                device = torch.device("cuda")
            else:
                device = torch.device("cpu")
                logging.warning("CUDA is not available. Running on CPU will be slow.")
        logging.info(f"device = {device}")
        device = torch.device("cuda")
        # -------------------- Data --------------------
        # rgb_filename_list = glob(os.path.join(input_rgb_dir, "*"))
        # rgb_filename_list = [
        #     f for f in rgb_filename_list if os.path.splitext(f)[1].lower() in EXTENSION_LIST
        # ]
        # rgb_filename_list = sorted(rgb_filename_list)
        n_images = len(rgb_filename_list)
        if n_images > 0:
            logging.info(f"Found {n_images} images")
        else:
            logging.error(f"No image found in '{input_rgb_dir}'")
            exit(1)

        # -------------------- Model --------------------
        if half_precision:
            dtype = torch.float16
            variant = "fp16"
            logging.info(
                f"Running with half precision ({dtype}), might lead to suboptimal result."
            )
        else:
            dtype = torch.float32
            variant = None

        pipe = MarigoldPipeline.from_pretrained(
            checkpoint_path, variant=variant, torch_dtype=dtype
        )

        try:
            pipe.enable_xformers_memory_efficient_attention()
        except ImportError:
            pass  # run without xformers

        pipe = pipe.to(device)
        # -------------------- Inference and saving --------------------
        with torch.no_grad():
            os.makedirs(output_dir, exist_ok=True)

            for rgb_path in tqdm(rgb_filename_list, desc="Estimating depth", leave=True):
                # Read input image
                input_image = Image.open(rgb_path)

                # Predict depth

                pipe_out = pipe(
                    input_image,
                    denoising_steps=denoise_steps,
                    ensemble_size=ensemble_size,
                    processing_res=processing_res,
                    match_input_res=match_input_res,
                    batch_size=batch_size,
                    color_map=color_map,
                    show_progress_bar=True,
                    resample_method=resample_method,
                    seed=seed,
                )

                depth_pred: np.ndarray = pipe_out.depth_np
                depth_colored: Image.Image = pipe_out.depth_colored

                # Save as npy
                rgb_name_base = os.path.splitext(os.path.basename(rgb_path))[0]
                pred_name_base = rgb_name_base + "_pred"
                npy_save_path = os.path.join(output_dir_npy, f"{pred_name_base}.npy")
                if os.path.exists(npy_save_path):
                    logging.warning(f"Existing file: '{npy_save_path}' will be overwritten")
                np.save(npy_save_path, depth_pred)

                # Save as 16-bit uint png
                depth_to_save = (depth_pred * 65535.0).astype(np.uint16)
               

                png_save_path = os.path.join(output_dir_tif, f"{pred_name_base}.png")
                if os.path.exists(png_save_path):
                    logging.warning(f"Existing file: '{png_save_path}' will be overwritten")
                Image.fromarray(depth_to_save).save(png_save_path, mode="I;16")

                # Colorize
                colored_save_path = os.path.join(
                    output_dir_color, f"{pred_name_base}_colored.png"
                )
                if os.path.exists(colored_save_path):
                    logging.warning(
                        f"Existing file: '{colored_save_path}' will be overwritten"
                    )
                depth_colored.save(colored_save_path)
    torch.cuda.empty_cache()
    gc.collect()
if "__main__" == __name__:
    logging.basicConfig(level=logging.INFO)
    #
    # # -------------------- Arguments --------------------

    rospy.init_node('depth_modle')
    # 定义一个名为 "int_service" 的服务，使用 IntService 消息类型，并指定处理函数为 handle_int_service
    s = rospy.Service('/depth_predict', Empty, handle_int_service)
    rospy.loginfo("depth_predict_service_server Ready.")
    rospy.spin()


# SPTG-LCC: Single-shot, Pixel-level, Target-free and General LiDAR-Camera Extrinsic Self-Calibration
The data fusion of LiDAR and camera holds vast application prospects, with calibration as a crucial prerequisite. In this paper, we propose SPTG-LCC, a novel, general, and target-free LiDAR-camera extrinsic calibration framework. On the one hand, SPTG-LCC  is open-source, which is very suitable for practitioners seeking a robust, general, and convenient target-free calibration tool. On the other hand, the four diverse datasets are open-source, which is very suitable for researchers to comprehensively evaluate feature-based target-free LiDAR-camera calibration methods.
![The pipeline of SPTG-LCC](./img/framework_LC.png)



# Video Link: [Video on Youtube](https://www.youtube.com/watch?v=qKA_KamtwTk) 
# To Do List
The following tasks will be completed quickly and step by step.
 
## Experimental Video
- [x] Video Link:  [Video on Youtube](https://www.youtube.com/watch?v=qKA_KamtwTk) 
## Datasets open-source
our  self-assembled  sensor suites as follows, where the camera is the Realsense D455  and ZED 2i. Four diverse datasets were collected using these four suites,  named **FB-LCC-NS360**, **FB-LCC-NS70**,  **FB-LCC-RS16**, and **FB-LCC-MEMS-M1**,  which are released for evaluating feature-based LiDAR-camera calibration methods. Moreover, sequence 00 on the public KITTI odometry benchmark  is evenly divided into 67 LiDAR-camera data pairs as a dataset, named **FB-LCC-RS-KITTI-VLP-64**.  
|![Dataset details](./img/sensor_suite.png)| ![Our self-assembled sensor suite](./img/Dataset_im.png)|
|--------------------------------------------|--------------------------------------------|
- [ ] FB-LCC-NS360 :  
- [ ] FB-LCC-NS70 :
- [x] FB-LCC-RS16 : [Baidu Cloud Disk](https://pan.baidu.com/s/1cqg4VAdqK6Zdf-HeuskbGw?pwd=l3jo )  
- [x] FB-LCC-MEMS-M1 :  [Baidu Cloud Disk](https://pan.baidu.com/s/1acPQ5RdGDKxyb62OjBmeRA?pwd=zpus )
- [x] FB-LCC-RS-KITTI-VLP-64 : [Baidu Cloud Disk](https://pan.baidu.com/s/17NBXiui3NjhHaQtuMu_A7w?pwd=qjmp )
## Code open-source
- [ ] **Docker images tool**
- [ ] main code
- [ ] test code
 
## Environment Setup
### 1. main code
```bash
https://github.com/NKU-MobFly-Robotics/SPTG-LCC.git
```
Assuming the folder where the code is downloaded locally is：
**/home/wyw/SPTG-LCC**
### 2. Docker images download

### 3. catkin_make
#### 1. Building an Docker Image
```bash
sudo docker run -it -v  /home/wyw/SPTG-LCC:/calib_data -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY --net=host -e GDK_SCALE   -e GDK_DPI_SCALE  --privileged --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all --name SPTG-LCC  sptg-lcc:latest   bash
```
#### 2. Start Docker Image
```bash
sudo docker start SPTG-LCC
```
#### 3. Enter the  Docker Image
```bash
sudo docker exec -it  SPTG-LCC bash
```
#### 4. catkin_make code
```bash
cd /calib_data/direct_lidar_camera
```
```bash
conda deactivate
```
```bash
catkin_make
```
### 4. weights download
#### 1. Download EfficientLoFTR weights 
Download the weight file and put it in the following folder
```bash
cd /home/wyw/SPTG-LCC/matcher/Efficinet_LOFTR/EfficientLoFTR && mkdir weights
```
#### 2. Download Lightglue weights
```bash
cd /home/wyw/SPTG-LCC/matcher/LightGlue 
```
#### 2. Download mono_depth weights
```bash
cd /home/wyw/SPTG-LCC/mono_depth/Marigold/ 
```
### 5. Start calibration

#### 1.Put your test bag package into the following folder
```bash
cd /home/wyw/SPTG-LCC/SPTG-LCC/bag
```
#### 2.Run the script
./LiDAR_Camera_calib.sh

#### 3.The results are saved in the following path
```bash
cd /home/wyw/SPTG-LCC/SPTG-LCC/results
```

# Acknowledgements  
- We sincerely appreciate the following open-source projects: [DVLC](https://github.com/koide3/direct_visual_lidar_calibration), [KITTI](https://www.cvlibs.net/datasets/kitti/), [Lightglue](https://github.com/cvg/LightGlue), [Efficient-LoFTR](https://github.com/zju3dv/EfficientLoFTR), [Marigold](https://github.com/prs-eth/Marigold), [Superpoint](https://github.com/rpautrat/SuperPoint). 
- In particular, our code framework is based on [DVLC(direct_visual_lidar_calibration)](https://github.com/koide3/direct_visual_lidar_calibration), thanks to this great open-source work.

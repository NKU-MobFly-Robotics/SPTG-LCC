#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <boost/program_options.hpp>
#include "direct_visual_lidar_calibration/init_LC.h" 
#include <nlohmann/json.hpp>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/Pose3.h>
#include <vlcal/common/visual_lidar_visualizer.hpp>
#include <fstream>
#include <ctime>
#include <sstream>
 
#include <dfo/nelder_mead.hpp>
#include <vlcal/common/console_colors.hpp>

#include <camera/create_camera.hpp>
#include <vlcal/common/estimate_fov.hpp>
#include <vlcal/common/estimate_pose.hpp>
#include <vlcal/common/visual_lidar_data.hpp>

#include <glk/primitives/primitives.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <ros/ros.h>
#include "std_msgs/Int32.h" 
#include "std_srvs/Empty.h" // 包含服务消息类型的头文件
#include <nav_msgs/Odometry.h>
Eigen::Matrix4d T_LC_reference_matrix;
 Eigen::Isometry3d  T_camera_lidar_now;
bool  T_camera_lidar_solve=false;
 std::vector<vlcal::VisualLiDARData::ConstPtr> dataset_visual;
 camera::GenericCameraBase::ConstPtr proj_visual;
namespace vlcal {

class InitialGuessAuto {
public:
  std::vector<int>  flag_depth_intens;
  int keypoint_num;
  int keypoint_num_depth;
  int bag_num;
  std::string mutching_model_select;
  bool pinhole_if;
  std::string data_path;
  camera::GenericCameraBase::ConstPtr proj;
  std::vector<VisualLiDARData::ConstPtr> dataset;
  std::vector<VisualLiDARData::ConstPtr> dataset2;
  void InitialGuessAuto_read(  std::string& data_path_input){
    data_path=data_path_input;
    std::ifstream ifs(data_path + "/calib.json");
    if (!ifs) {
      std::cerr << vlcal::console::bold_red << "error: failed to open " << data_path << "/calib.json" << vlcal::console::reset << std::endl;
      abort();
    }
  
    const int pick_window_size = 1;
    for (int i = -pick_window_size; i <= pick_window_size; i++) {
      for (int j = -pick_window_size; j <= pick_window_size; j++) {
        if (i == 0 && j == 0) {
          continue;
        }
        pick_offsets.emplace_back(i, j);
      }
    }
    std::sort(pick_offsets.begin(), pick_offsets.end(), [](const auto& lhs, const auto& rhs) { return lhs.squaredNorm() < rhs.squaredNorm(); });

    ifs >> config;
    
    const std::string camera_model = config["camera"]["camera_model"];
    const std::vector<double> intrinsics = config["camera"]["intrinsics"];
    const std::vector<double> distortion_coeffs = config["camera"]["distortion_coeffs"];
    proj = camera::create_camera(camera_model, intrinsics, distortion_coeffs);
    proj_visual=proj;
    std::vector<int> insert_indices;
    bag_num=0;
    const std::vector<std::string> bag_names = config["meta"]["bag_names"];
    if(mutching_model_select!="EFLoFTR_and_Glue"){
      for (const auto& bag_name : bag_names) 
      { dataset2.emplace_back(std::make_shared<VisualLiDARData>(data_path, bag_name));
        dataset_visual=dataset2;
        bag_num++;
        for(int i=0;i<3;i++)
      {
        
          dataset.emplace_back(std::make_shared<VisualLiDARData>(data_path, bag_name));
             if(i==0)
              {
               if(pinhole_if==true)
                {auto corrs = read_correspondences(data_path, bag_name, dataset.back()->points,mutching_model_select);
                correspondences.insert(correspondences.end(), corrs.begin(), corrs.end());
                int last_index = correspondences.size() - 1;
                flag_depth_intens.push_back(last_index);}
                else
                {
                  auto corrs = read_correspondences(data_path, bag_name, dataset.back()->points,mutching_model_select).at(0);
                  correspondences.push_back(corrs);
                  int last_index = correspondences.size() - 1;
                  flag_depth_intens.push_back(last_index);
                }
              }
          else if(i==1)
              { 
    
                auto corrs_eq = read_correspondences_eq(data_path, bag_name, dataset.back()->points,mutching_model_select);
                correspondences.insert(correspondences.end(), corrs_eq.begin(), corrs_eq.end());
                 
                auto corrs_eq_2 = read_correspondences_depth_eq(data_path, bag_name, dataset.back()->points,mutching_model_select);
                correspondences.insert(correspondences.end(), corrs_eq_2.begin(), corrs_eq_2.end());
                int last_index = correspondences.size() - 1;
                flag_depth_intens.push_back(last_index);
              }
          else
              { 
                
                 if(pinhole_if==true)
                {auto corrs_depth = read_correspondences_depth(data_path, bag_name, dataset.back()->points,mutching_model_select);
                correspondences.insert(correspondences.end(), corrs_depth.begin(), corrs_depth.end());
                int last_index = correspondences.size() - 1;
                flag_depth_intens.push_back(last_index);  
                keypoint_num_depth=corrs_depth.size();}
                else
                  {auto corrs_depth = read_correspondences_depth(data_path, bag_name, dataset.back()->points,mutching_model_select).at(0);
                correspondences.push_back(corrs_depth);
                int last_index = correspondences.size() - 1;
                flag_depth_intens.push_back(last_index);}

              }  
         }
      }
    }
    else
    {
      for(int j=0;j<2;j++)
      {
          if(j==0)
          {
            mutching_model_select="_EFLoFTR";
          }
          else
          {
            mutching_model_select="_lightGlue";
          }
          for (const auto& bag_name : bag_names) 
            { bag_num++;
                for(int i=0;i<3;i++)
              {
              
                    dataset.emplace_back(std::make_shared<VisualLiDARData>(data_path, bag_name));
                    if(i==0)
                    {
                        if(pinhole_if==true)
                          {auto corrs = read_correspondences(data_path, bag_name, dataset.back()->points,mutching_model_select);
                          correspondences.insert(correspondences.end(), corrs.begin(), corrs.end());
                          int last_index = correspondences.size() - 1;
                          flag_depth_intens.push_back(last_index);}
                          else
                          {
                            auto corrs = read_correspondences(data_path, bag_name, dataset.back()->points,mutching_model_select).at(0);
                            correspondences.push_back(corrs);
                          int last_index = correspondences.size() - 1;
                          flag_depth_intens.push_back(last_index);
                          }
                    }
                else if(i==1)
                    { 
                    
                      auto corrs_eq = read_correspondences_eq(data_path, bag_name, dataset.back()->points,mutching_model_select);
                      correspondences.insert(correspondences.end(), corrs_eq.begin(), corrs_eq.end());
                      
                        auto corrs_eq_2 = read_correspondences_depth_eq(data_path, bag_name, dataset.back()->points,mutching_model_select);
                        correspondences.insert(correspondences.end(), corrs_eq_2.begin(), corrs_eq_2.end());

                      
                        int last_index = correspondences.size() - 1;
                        flag_depth_intens.push_back(last_index);
      
                    }
                else
                    { 
                      
                      if(pinhole_if==true)
                      {auto corrs_depth = read_correspondences_depth(data_path, bag_name, dataset.back()->points,mutching_model_select);
                      correspondences.insert(correspondences.end(), corrs_depth.begin(), corrs_depth.end());
                      int last_index = correspondences.size() - 1;
                      flag_depth_intens.push_back(last_index);  
                      keypoint_num_depth=corrs_depth.size();}
                      else
                        {auto corrs_depth = read_correspondences_depth(data_path, bag_name, dataset.back()->points,mutching_model_select).at(0);
                      correspondences.push_back(corrs_depth);
                      int last_index = correspondences.size() - 1;
                      flag_depth_intens.push_back(last_index);}
                    
                    }
              }
            }
        }

    }
      }
  
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>
  read_correspondences(const std::string& data_path, const std::string& bag_name, const Frame::ConstPtr& points,std::string mutching_model_select) {
   cv::Mat point_indices_8uc4 = cv::imread(data_path + "/" + bag_name + "_lidar_indices.png", -1);
    cv::Mat point_indices = cv::Mat(point_indices_8uc4.rows, point_indices_8uc4.cols, CV_32SC1, reinterpret_cast<int*>(point_indices_8uc4.data));

    std::ifstream matches_ifs(data_path + "/" + bag_name + "_matches_ph"+mutching_model_select+".json");
    if(!matches_ifs) {
      std::cerr << vlcal::console::bold_red << "error: failed to open " << data_path + "/" + bag_name + "_matches.json" << vlcal::console::reset << std::endl;
      abort();
    }
     nlohmann::json matching_result;
    matches_ifs >> matching_result;

    std::vector<int> kpts0 = matching_result["kpts0"];
    std::vector<int> kpts1 = matching_result["kpts1"];
    std::vector<int> matches = matching_result["matches"];
    std::vector<double> confidence = matching_result["confidence"];

    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>> correspondences;
    for (int i = 0; i < matches.size(); i++) {
      if (matches[i] < 0) {
        continue;
      }
       const Eigen::Vector2i kp0(kpts0[2 * i], kpts0[2 * i + 1]);
      const Eigen::Vector2i kp1(kpts1[2 * matches[i]], kpts1[2 * matches[i] + 1]);
           std::int32_t point_index;
try {           
  //  std::cout << "Error:2222 " << std::endl;

      point_index = point_indices.at<std::int32_t>(kp1.y(), kp1.x());
            // std::cout << "Error:2222 " << std::endl;

}
catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    // Handle error or recover here
    continue;
}
      const int pick_window_size = 2;
      if(point_index < 0) {
        for(const auto& offset : pick_offsets) {
          point_index = point_indices.at<std::int32_t>(kp1.y() + offset.y(), kp1.x() + offset.x());

          if (point_index >= 0) {
            break;
          }
        }

        if (point_index < 0) {
          std::cerr << vlcal::console::bold_yellow << "warning: ignore keypoint in a blank region!!" << vlcal::console::reset << std::endl;
        }
        continue;
      }
      try {
    correspondences.emplace_back(kp0.cast<double>(), points->points[point_index]);
} catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    // Handle error or recover here
    continue;
}
    }

   
    return correspondences;
  }
std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>
  read_correspondences_depth_eq(const std::string& data_path, const std::string& bag_name, const Frame::ConstPtr& points,std::string mutching_model_select) {
   cv::Mat point_indices_8uc4 = cv::imread(data_path + "/" + bag_name + "_lidar_indices_eq.png", -1);
    cv::Mat point_indices = cv::Mat(point_indices_8uc4.rows, point_indices_8uc4.cols, CV_32SC1, reinterpret_cast<int*>(point_indices_8uc4.data));

    std::ifstream matches_ifs(data_path + "/" + bag_name + "_matches_depth_eq"+mutching_model_select+".json");
    if(!matches_ifs) {
      std::cerr << vlcal::console::bold_red << "error: failed to open " << data_path + "/" + bag_name + "_matches.json" << vlcal::console::reset << std::endl;
      abort();
    }
     nlohmann::json matching_result;
    matches_ifs >> matching_result;

    std::vector<int> kpts0 = matching_result["kpts0"];
    std::vector<int> kpts1 = matching_result["kpts1"];
    std::vector<int> matches = matching_result["matches"];
    std::vector<double> confidence = matching_result["confidence"];

    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>> correspondences;
    for (int i = 0; i < matches.size(); i++) {
      if (matches[i] < 0) {
        continue;
      }
       const Eigen::Vector2i kp0(kpts0[2 * i], kpts0[2 * i + 1]);
      const Eigen::Vector2i kp1(kpts1[2 * matches[i]], kpts1[2 * matches[i] + 1]);
           std::int32_t point_index;
try {           
  //  std::cout << "Error:2222 " << std::endl;

      point_index = point_indices.at<std::int32_t>(kp1.y(), kp1.x());
            // std::cout << "Error:2222 " << std::endl;

}
catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    // Handle error or recover here
    continue;
}
      const int pick_window_size = 2;
      if(point_index < 0) {
        for(const auto& offset : pick_offsets) {
          point_index = point_indices.at<std::int32_t>(kp1.y() + offset.y(), kp1.x() + offset.x());

          if (point_index >= 0) {
            break;
          }
        }

        if (point_index < 0) {
          std::cerr << vlcal::console::bold_yellow << "warning: ignore keypoint in a blank region!!" << vlcal::console::reset << std::endl;
        }
        continue;
      }
      try {
    correspondences.emplace_back(kp0.cast<double>(), points->points[point_index]);
} catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    // Handle error or recover here
    continue;
}
    }

   
    return correspondences;
  }



  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>
  read_correspondences_depth(const std::string& data_path, const std::string& bag_name, const Frame::ConstPtr& points,std::string mutching_model_select) {
    cv::Mat point_indices_8uc4 = cv::imread(data_path + "/" + bag_name + "_lidar_indices.png", -1);
    cv::Mat point_indices = cv::Mat(point_indices_8uc4.rows, point_indices_8uc4.cols, CV_32SC1, reinterpret_cast<int*>(point_indices_8uc4.data));

    std::ifstream matches_ifs(data_path + "/" + bag_name + "_matches_depth"+mutching_model_select+".json");
    if(!matches_ifs) {
      std::cerr << vlcal::console::bold_red << "error: failed to open " << data_path + "/" + bag_name + "_matches.json" << vlcal::console::reset << std::endl;
      abort();
    }
     nlohmann::json matching_result;
    matches_ifs >> matching_result;

    std::vector<int> kpts0 = matching_result["kpts0"];
    std::vector<int> kpts1 = matching_result["kpts1"];
    std::vector<int> matches = matching_result["matches"];
    std::vector<double> confidence = matching_result["confidence"];

    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>> correspondences;
    for (int i = 0; i < matches.size(); i++) {
      if (matches[i] < 0) {
        continue;
      }
       const Eigen::Vector2i kp0(kpts0[2 * i], kpts0[2 * i + 1]);
      const Eigen::Vector2i kp1(kpts1[2 * matches[i]], kpts1[2 * matches[i] + 1]);
           std::int32_t point_index;
try {           // std::cout << "Error:2222 " << std::endl;

      point_index = point_indices.at<std::int32_t>(kp1.y(), kp1.x());
          //  std::cout << "Error:2222 " << std::endl;

}
catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    // Handle error or recover here
    continue;
}
      const int pick_window_size = 2;
      if(point_index < 0) {
        for(const auto& offset : pick_offsets) {
          point_index = point_indices.at<std::int32_t>(kp1.y() + offset.y(), kp1.x() + offset.x());

          if (point_index >= 0) {
            break;
          }
        }

        if (point_index < 0) {
          std::cerr << vlcal::console::bold_yellow << "warning: ignore keypoint in a blank region!!" << vlcal::console::reset << std::endl;
        }
        continue;
      }
      try {
    correspondences.emplace_back(kp0.cast<double>(), points->points[point_index]);
} catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    // Handle error or recover here
    continue;
}
    }

    return correspondences;
  }
 std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>
  read_correspondences_eq(const std::string& data_path, const std::string& bag_name, const Frame::ConstPtr& points,std::string mutching_model_select) {
    cv::Mat point_indices_8uc4 = cv::imread(data_path + "/" + bag_name + "_lidar_indices_eq.png", -1);
    cv::Mat point_indices = cv::Mat(point_indices_8uc4.rows, point_indices_8uc4.cols, CV_32SC1, reinterpret_cast<int*>(point_indices_8uc4.data));

    std::ifstream matches_ifs(data_path + "/" + bag_name + "_matches_eq"+mutching_model_select+".json");
    if(!matches_ifs) {
      std::cerr << vlcal::console::bold_red << "error: failed to open " << data_path + "/" + bag_name + "_matches_eq.json" << vlcal::console::reset << std::endl;
      abort();
    }

    nlohmann::json matching_result;
    matches_ifs >> matching_result;

    std::vector<int> kpts0 = matching_result["kpts0"];
    std::vector<int> kpts1 = matching_result["kpts1"];
    std::vector<int> matches = matching_result["matches"];
    std::vector<double> confidence = matching_result["confidence"];

    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>> correspondences;
    for (int i = 0; i < matches.size(); i++) {
      if (matches[i] < 0) {
        continue;
      }

      const Eigen::Vector2i kp0(kpts0[2 * i], kpts0[2 * i + 1]);
      const Eigen::Vector2i kp1(kpts1[2 * matches[i]], kpts1[2 * matches[i] + 1]);
      std::int32_t point_index;
try {
    point_index = point_indices.at<std::int32_t>(kp1.y(), kp1.x());
 }
catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    // Handle error or recover here
    continue;
}
 
      const int pick_window_size = 2;
      if(point_index < 0) {
        for(const auto& offset : pick_offsets) {
          point_index = point_indices.at<std::int32_t>(kp1.y() + offset.y(), kp1.x() + offset.x());

          if (point_index >= 0) {
            break;
          }
        }

        if (point_index < 0) {
          std::cerr << vlcal::console::bold_yellow << "warning: ignore keypoint in a blank region!!" << vlcal::console::reset << std::endl;
        }
        continue;
      }
 
     try {
    correspondences.emplace_back(kp0.cast<double>(), points->points[point_index]);
} catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    // Handle error or recover here
    continue;
}
    }

    return correspondences;
  }
  Eigen::Isometry3d estimate_and_save(const boost::program_options::variables_map& vm,bool pinhole_if) {
    PoseEstimationParams params;
    params.ransac_iterations = vm["ransac_iterations"].as<int>();
    params.ransac_error_thresh = vm["ransac_error_thresh"].as<double>();
    params.robust_kernel_width = vm["robust_kernel_width"].as<double>();
    keypoint_num=correspondences.size();
    keypoint_num_depth=keypoint_num_depth/bag_num;
    keypoint_num=keypoint_num/bag_num;
    PoseEstimation pose_estimation(params);
    std::cout << "44444444444444444444: " << correspondences.size() << std::endl;
    std::vector<bool> inliers;
    Eigen::Isometry3d T_camera_lidar = pose_estimation.estimate(proj, correspondences, &inliers,flag_depth_intens, pinhole_if);
 //开一个可视化的线程
    T_camera_lidar_solve=true;
  T_camera_lidar_now=T_camera_lidar;
    
      


    const Eigen::Isometry3d T_lidar_camera = T_camera_lidar.inverse();

     Eigen::Isometry3d init_T_camera_lidar1;
     //给init_T_camera_lidar手动赋值
      init_T_camera_lidar1.translation() << 0.0, 0.0, 0.0;

      // init_T_camera_lidar.linear() = Eigen::Quaterniond(0.508862,
      // 0.5018527, -0.4926349, 0.4965018  ).normalized().toRotationMatrix();

      // Remove hidden points
    
    // points = view_culling.cull(points, init_T_camera_lidar,0.4);
 
  
      init_T_camera_lidar1.linear()<<-0.0648776,  0.99782 , 0.0121069,
 0.0144289 ,-0.0111932,   0.999833,
  0.997789,  0.0650415, -0.0136712;

    Eigen::Isometry3d stand_T_camera_lidar;
     stand_T_camera_lidar.translation() << 0.0, 0.0, 0.0;
      stand_T_camera_lidar.linear()<< 0, 0 , 1,
 -1,0,0,
  0,-1,0;
   
 Eigen::Isometry3d truth_T_Lidar_camera;
    truth_T_Lidar_camera.translation() << -0.13353, -0.055647, -0.110234;
    truth_T_Lidar_camera.linear()<<-0.0131624,  0.00449038,  0.999903,
   0.999893, 0.00628775,  0.0111904,
   -0.00643463,  0.99997, 0.00440714; 
 
  Eigen::Isometry3d truth_T_Lidar360_camera;
    truth_T_Lidar360_camera.translation() << 0.0894463, -0.0164956, -0.0654503;
    truth_T_Lidar360_camera.linear()<<0.0000675, -0.00885072, 0.999961,
   -0.999997, 0.00260872,  0.0000901,
   -0.00260945,  -0.999958, -0.00885051; 

// T_camera_lidar=T_camera_lidar*stand_T_camera_lidar*init_T_camera_lidar1; 

  truth_T_Lidar360_camera.linear() <<-0.0175979, -0.0007485,  0.9998448,
  -0.9998432,  0.0019689, -0.0175964,
  -0.0019554, -0.9999978, -0.0007830;
// 假设R1和R2是你要比较的两个旋转矩阵

Eigen::Matrix3d R1 = truth_T_Lidar360_camera.linear();
Eigen::Matrix3d R2 = T_camera_lidar.linear();
 truth_T_Lidar360_camera=T_LC_reference_matrix;
Eigen::Isometry3d Error_T_Lidar360_camera = truth_T_Lidar360_camera * T_lidar_camera.inverse();
Eigen::Matrix3d R = Error_T_Lidar360_camera.linear();
Eigen::AngleAxisd angleAxis(R);
Eigen::Vector3d axis = angleAxis.axis();
double angle = angleAxis.angle();
Eigen::Vector3d translation_diff = truth_T_Lidar360_camera.translation() - T_lidar_camera.translation();
double distance = translation_diff.norm();

// double distance = Error_T_Lidar360_camera.translation().norm();

std::cout << "Axis: " << axis.transpose() << std::endl;
std::cout << "Angle: " << angle * 180 / M_PI << std::endl;
std::cout << "Distance: " << distance << std::endl;
std::cout << "T_camera_lidar: " << T_camera_lidar.matrix() << std::endl;
std::cout << "T_lidar_camera: " << T_camera_lidar.inverse().matrix() << std::endl;
//计算Error_T_Lidar360_camera平移向量的模长
 


 
    const Eigen::Vector3d trans = T_lidar_camera.translation();
    const Eigen::Quaterniond quat = Eigen::Quaterniond(T_lidar_camera.linear()).normalized();
    const std::vector<double> values = {trans.x(), trans.y(), trans.z(), quat.x(), quat.y(), quat.z(), quat.w()};

    config["results"]["init_T_lidar_camera_auto"] = values;
    std::cout << data_path << std::endl;
    std::mutex mutex;

    // 使用互斥量保护文件写入操作
    {
      std::lock_guard<std::mutex> lock(mutex);
      std::ofstream ofs(data_path + "/calib2.json");
      
      if (!ofs) {
        std::cerr << vlcal::console::bold_red << "error: failed to open " << data_path + "/calib.json"
                  << " for writing" << vlcal::console::reset << std::endl;
        abort();
      }

      std::cout << data_path << "ssssssss" << std::endl;

      ofs << config.dump(2) << std::endl;
      ofs.flush();
    }
    return T_lidar_camera;
  }
private:
   
  nlohmann::json config;

  std::vector<Eigen::Vector2i> pick_offsets;

  

  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>> correspondences;
};
}
bool initial_alignment;
int preprecess_LC_flag=0;
 using namespace boost::program_options;
variables_map vm;
std::string data_path;
std::string match_model="/server_lightglue",mutching_model_select;
int server_num=0;
void preprecess_LC(const std_msgs::Int32::ConstPtr& msg)
{
     preprecess_LC_flag=msg->data;
   
    ROS_INFO("Received message: [%d]", msg->data); // 输出接收到的消息
}
int key_point_num=0;
std::string getCurrentTimeAsFileName() {
    // 获取当前时间
    std::time_t now = std::time(nullptr);
    std::tm* time_info = std::localtime(&now);

    // 创建时间字符串
    std::ostringstream oss;
    oss << (1900 + time_info->tm_year) << "_"
        << (1 + time_info->tm_mon) << "_"
        << time_info->tm_mday << "_"
        << time_info->tm_hour << "_"
        << time_info->tm_min << "_"
        << time_info->tm_sec;

    return oss.str();
}

void saveTransformationAndInverse(const Eigen::Isometry3d& T_lidar_camera, const std::string& file_name) {
    // 获取旋转矩阵和四元数
    Eigen::Matrix3d rotation_matrix = T_lidar_camera.rotation();
    Eigen::Quaterniond quaternion(T_lidar_camera.rotation());
   Eigen::Matrix4d matrix_lidar_camera = T_lidar_camera.matrix();
    // 计算逆矩阵
    Eigen::Isometry3d T_camera_lidar = T_lidar_camera.inverse();
    Eigen::Matrix3d rotation_matrix_inv = T_camera_lidar.rotation();
    Eigen::Quaterniond quaternion_inv(T_camera_lidar.rotation());
   Eigen::Matrix4d matrix_camera_lidar = T_camera_lidar.matrix();
    // 保存到文件
    std::ofstream file(file_name);
    if (file.is_open()) {
        file << "T_lidar_camera (camera is reference ^C_P=T_LC* ^L_P):\n" << matrix_lidar_camera << "\n\n";
        file << "T_lidar_camera Quaternion ([x, y, z, w]):\n" << quaternion.coeffs() << "\n\n";
        file << "T_camera_lidar:\n" << matrix_camera_lidar << "\n\n";
        file << "T_camera_lidarQuaternion:\n" << quaternion_inv.coeffs() << "\n";
        file.close();
        std::cout << "Data saved to " << file_name << std::endl;
    } else {
        std::cerr << "Unable to open file " << file_name << std::endl;
    }
}

bool LI_init_server(direct_visual_lidar_calibration::init_LC::Request& req,
              direct_visual_lidar_calibration::init_LC::Response& res)
{
    // Handle the service request here
    // For example, you can simply echo the request and respond with some predefined value
    ROS_INFO("Received request: %d", req.request_data);
    server_num++;
  //获取req.request_data的个位和十位
   
   int  tens = req.request_data / 10;
   int  ones = req.request_data % 10;
  if(tens==0)
    {
       match_model="/server_EffLoFTR";
       mutching_model_select="_EFLoFTR";
    }
  else if(tens==1)
  {
    match_model="/server_EffLoFTR";
       mutching_model_select="_EFLoFTR";
    // match_model="/server_lightglue";
    //   mutching_model_select="_lightGlue";
    // match_model="/server_EffLoFTR";
  }
  else  if(tens==2)
    {
      match_model="/server_EffLoFTR";
       mutching_model_select="_EFLoFTR";
      //  mutching_model_select="_lightGlue";
      // match_model="/server_lightglue";
    }  

  int pinhole_if  =ones;
     std_srvs::Empty srv;
    ros::NodeHandle nh;
 
  ros::ServiceClient client_lightglue = nh.serviceClient<std_srvs::Empty>(match_model);
    // 调用服务
     Eigen::Isometry3d T_lidar_camera;
  if(tens<3)
    {if (client_lightglue.call(srv))
    
   
    {
    vlcal::InitialGuessAuto init_guess;
     init_guess.mutching_model_select=mutching_model_select; //_EFLoFTR
    
   if(pinhole_if==1)
   {
         init_guess.pinhole_if=true;
   }
  else
    {
      init_guess.pinhole_if=false;
    }
    init_guess.InitialGuessAuto_read(data_path);
    std::cout<<data_path<<"sssssssssssssqww"<<std::endl;
    T_lidar_camera=init_guess.estimate_and_save(vm,init_guess.pinhole_if); 
    key_point_num=init_guess.keypoint_num;
    if(key_point_num<800||init_guess.keypoint_num_depth<200)
    {
       mutching_model_select="_EFLoFTR";
       match_model="/server_EffLoFTR";
    }

    
    
   init_guess.mutching_model_select=mutching_model_select;
  //发布里程计信息
   
   
     res.response_data.pose.pose.position.x = T_lidar_camera.translation().x();
     res.response_data.pose.pose.position.y = T_lidar_camera.translation().y();
     res.response_data.pose.pose.position.z = T_lidar_camera.translation().z();
    const Eigen::Quaterniond quat = Eigen::Quaterniond(T_lidar_camera.linear()).normalized();

     res.response_data.pose.pose.orientation.x = quat.x();
     res.response_data.pose.pose.orientation.y = quat.y();
     res.response_data.pose.pose.orientation.z = quat.z();
     res.response_data.pose.pose.orientation.w = quat.w();
    }
    }
  else
  { ros::ServiceClient client_EFLoFTR = nh.serviceClient<std_srvs::Empty>("/server_EffLoFTR");
   ros::ServiceClient client_lightglue2 = nh.serviceClient<std_srvs::Empty>("/server_lightglue");
     vlcal::InitialGuessAuto init_guess;
      init_guess.mutching_model_select="EFLoFTR_and_Glue"; //_EFLoFTR
    if (client_EFLoFTR.call(srv))
    {
      if (client_lightglue2.call(srv))
      {
        if(pinhole_if==1)
        {
              init_guess.pinhole_if=true;
        }
        else
          {
            init_guess.pinhole_if=false;
          }
          init_guess.InitialGuessAuto_read(data_path);
          std::cout<<data_path<<"sssssssssssssqww"<<std::endl;
          T_lidar_camera=init_guess.estimate_and_save(vm,init_guess.pinhole_if); 
          vlcal::InitialGuessAuto init_guess;
           
           res.response_data.pose.pose.position.x = T_lidar_camera.translation().x();
     res.response_data.pose.pose.position.y = T_lidar_camera.translation().y();
     res.response_data.pose.pose.position.z = T_lidar_camera.translation().z();
    const Eigen::Quaterniond quat = Eigen::Quaterniond(T_lidar_camera.linear()).normalized();

     res.response_data.pose.pose.orientation.x = quat.x();
     res.response_data.pose.pose.orientation.y = quat.y();
     res.response_data.pose.pose.orientation.z = quat.z();
     res.response_data.pose.pose.orientation.w = quat.w();
      }
   
  }
   
    // Fill the response data
  }  
saveTransformationAndInverse(T_lidar_camera,"/calib_data/SPTG-LCC/results/calibration_"+getCurrentTimeAsFileName()+".txt");
    return true;
}
int main(int argc, char** argv) {
  
  
  using namespace boost::program_options;
  options_description description("initial_guess_auto");
  //定义ros节点定义发布者
  ros::init(argc, argv, "PUB_INIT_T_LiDAR_Camera");

  ros::NodeHandle nh;
std::vector<double> T_LC_reference;

  nh.param< std::vector<double> >("initial_guess/T_LC_reference",T_LC_reference , std::vector<double>());
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            T_LC_reference_matrix(i, j) = T_LC_reference[i * 4 + j];
        }
    }  
  nh.getParam("/preprocess/initial_alignment", initial_alignment);
  //定义发布者
  ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/init_T_LiDAR_Camera", 1000);
  ros::Subscriber sub_preprecess_LC = nh.subscribe<std_msgs::Int32>("/preprecess_LC", 1, preprecess_LC);
  ros::ServiceClient client_lightglue = nh.serviceClient<std_srvs::Empty>("/server_lightglue");
  nav_msgs::Odometry odom;
   ros::ServiceServer service = nh.advertiseService("init_LC", LI_init_server);
  //初始化odom
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";
  odom.pose.pose.position.x = 0;
  odom.pose.pose.position.y = 0;
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation.x = 0;
  odom.pose.pose.orientation.y = 0;
   odom.pose.pose.orientation.z = 0;
   odom.pose.pose.orientation.w = 1;
  // clang-format off
  description.add_options()
    ("help", "produce help message")
    ("data_path", value<std::string>(), "directory that contains preprocessed data")
    ("ransac_iterations", value<int>()->default_value(8192), "iterations for RANSAC")
    ("ransac_error_thresh", value<double>()->default_value(10.0), "reprojection error threshold [pix]")
    ("robust_kernel_width", value<double>()->default_value(10.0), "Cauchy kernel width for fine estimation [pix]")
  ;
  // clang-format on

  positional_options_description p;
  p.add("data_path", 1);
  store(command_line_parser(argc, argv).options(description).positional(p).run(), vm);
  notify(vm);

  if (vm.count("help") || !vm.count("data_path")) {
    std::cout << description << std::endl;
    return 0;
  }

   data_path = vm["data_path"].as<std::string>();

  
 Eigen::Isometry3d T_lidar_camera;

  // T_lidar_camera=init_guess.estimate_and_save(vm);
  // while (ros::ok())
  // {
   
  //  if(preprecess_LC_flag==1)
  //  {
  
  // //调用ros服务
  //  std_srvs::Empty srv;

  //   // 调用服务
  //   if (client_lightglue.call(srv))
    
   
  //   {
  //     vlcal::InitialGuessAuto init_guess(data_path);
  //   T_lidar_camera=init_guess.estimate_and_save(vm);
  // //发布里程计信息
   
  //   odom.header.stamp = ros::Time::now();
  //   odom.header.frame_id = "odom";
  //   odom.child_frame_id = "base_link";
  //   odom.pose.pose.position.x = T_lidar_camera.translation().x();
  //   odom.pose.pose.position.y = T_lidar_camera.translation().y();
  //   odom.pose.pose.position.z = T_lidar_camera.translation().z();
  //   const Eigen::Quaterniond quat = Eigen::Quaterniond(T_lidar_camera.linear()).normalized();

  //   odom.pose.pose.orientation.x = quat.x();
  //   odom.pose.pose.orientation.y = quat.y();
  //   odom.pose.pose.orientation.z = quat.z();
  //   odom.pose.pose.orientation.w = quat.w();
  //   }
  //   else
  //   {
  //     ROS_ERROR("Failed to call service");
       
  //   }
   
  //  }
   
  //   pub.publish(odom);  
  //   ros::spinOnce();
  //   sleep(1);
  // }
  //  std::thread([&] {
  //  ros::spin();}).detach();
 ros::spin();
//    while(T_camera_lidar_solve==false){
//     std::cout<<"wait for initial guess"<<T_camera_lidar_solve<<std::endl;
//    };
//     { vlcal::InitialGuessAuto init_guess;
     
      
//  auto viewer = guik::LightViewer::instance(Eigen::Vector2i(-1, -1), vm.count("background"));
//     viewer->set_draw_xy_grid(false);
//     viewer->use_arcball_camera_control();

//     viewer->invoke([] {
//       ImGui::SetNextWindowPos({55, 300}, ImGuiCond_Once);
//       ImGui::Begin("texts");
//       ImGui::End();
//       ImGui::SetNextWindowPos({55, 60}, ImGuiCond_Once);
//       ImGui::Begin("visualizer");
//       ImGui::End();
//       ImGui::SetNextWindowPos({1260, 60}, ImGuiCond_Once);
//       ImGui::Begin("images");
//       ImGui::End();
//     });
//     //  vis.spin_once();
//       //  if (!vm.count("auto_quit"))
//       //   {
         
 

//       vlcal::VisualLiDARVisualizer vis(proj_visual,dataset_visual, false);
//   vis.set_T_camera_lidar(T_camera_lidar_now);
//       if (!vm.count("auto_quit"))
//         {
//       viewer->spin();return 0;
//         }

//         }
   
   
 return 0;
  
}

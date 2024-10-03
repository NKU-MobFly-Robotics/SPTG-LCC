#include <vlcal/preprocess/preprocess.hpp>
#include "ros/package.h"
#include <fstream>
#include <iostream>
#include <unordered_set>
#include <filesystem>

#include <sophus/se3.hpp>
#include <sophus/ceres_manifold.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include <nlohmann/json.hpp>

#include <vlcal/common/frame_cpu.hpp>
#include <vlcal/common/time_keeper.hpp>
#include <vlcal/common/console_colors.hpp>

#include <camera/create_camera.hpp>
#include <vlcal/common/estimate_fov.hpp>
#include <vlcal/preprocess/generate_lidar_image.hpp>
#include <vlcal/preprocess/static_point_cloud_integrator.hpp>
#include <vlcal/preprocess/dynamic_point_cloud_integrator.hpp>
#include <vlcal/calib/view_culling_inital.hpp>
 
#include <glk/io/ply_io.hpp>
#include <glk/texture_opencv.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <ros/ros.h>

namespace vlcal {

Preprocess::Preprocess() {}

Preprocess::~Preprocess() {}
bool Preprocess::Preprocess_init(int argc, char** argv, int status_flag, Eigen::Isometry3d init_L_C)
{
  using namespace boost::program_options;
  options_description description("preprocess");
   std::vector<double> matrix_T_L0C0,matrix_T_CL_init;
  
    nh_.param<std::string>("preprocess/data_path",data_path,  "directory that contains rosbags for calibration");
    nh_.param<std::string>("preprocess/dst_path",dst_path,  "directory that contains rosbags for calibration");
    nh_.param<bool>("preprocess/auto_topic",auto_topic,  "directory that contains rosbags for calibration");
    nh_.param<bool>("preprocess/visualize",visualize,  "directory that contains rosbags for calibration");
    nh_.param<bool>("preprocess/cull_hidden_points",cull_hidden_points,  "directory that contains rosbags for calibration");
    nh_.param<bool>("preprocess/initial_alignment",initial_alignment,  "directory that contains rosbags for calibration");
    nh_.param<bool>("preprocess/dynamic_point",dynamic_point,  "directory that contains rosbags for calibration");
    nh_.param<int>("preprocess/LiDAR_type",LiDAR_type,  0);
    nh_.param<int>("preprocess/Hfov",hfov,  0);

   
    nh_.param<std::string>("preprocess/image_topic",image_topic,  "directory that contains rosbags for calibration");
      nh_.param<std::string>("preprocess/points_topic",points_topic,  "directory that contains rosbags for calibration");
    nh_.param<std::string>("preprocess/camera_info_topic",camera_info_topic,  "directory that contains rosbags for calibration");
 
    nh_.param< std::vector<double> >("initial_guess/T_L0C0",matrix_T_L0C0 , std::vector<double>());
    nh_.param< std::vector<double> >("initial_guess/T_CL_init",matrix_T_CL_init, std::vector<double>());
   
    double min_dis;
    nh_.param< double >("preprocess/min_dis",min_dis,0.3);
    
    Eigen::Matrix4d transformation_matrix;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            transformation_matrix(i, j) = matrix_T_L0C0[i * 4 + j];
        }
    }  
   
   T_L0C0=transformation_matrix;
  for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            transformation_matrix(i, j) = matrix_T_CL_init[i * 4 + j];
        }
    } 
    T_CL_init= transformation_matrix;
    // nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);
    // nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    // nh.param<string>("preprocess/image_topic",lid_topic,"/livox/lidar");
    // nh.param<string>("preprocess/points_topic", imu_topic,"/livox/imu");
    // nh.param<bool>("common/time_sync_en", time_sync_en, false);
    // nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    // nh.param<vector<double>>("initial_guess/T_L0C0", extrinT, vector<double>());
    // nh.param<vector<double>>("initial_guess/R_L0C0", extrinR, vector<double>())
    // nh.param<vector<double>>("initial_guess/T_LC_init", extrinT, vector<double>());
    // nh.param<vector<double>>("initial_guess/R_LC_init", extrinR, vector<double>());
 std::cout<<"ddddddddd"<<min_dis<<std::endl;
   // clang-format off
  description.add_options()
    ("help", "produce help message")
    // ("data_path", value<std::string>(), "directory that contains rosbags for calibration")
    // ("dst_path", value<std::string>(), "directory to save preprocessed data")
    ("bag_id", value<int>(), "specify the bag to use (just for evaluation)")
    ("first_n_bags", value<int>(), "use only the first N bags (just for evaluation)")
    ("auto_topic,a", "automatically select topics")
    ("dynamic_lidar_integration,d", "create target point cloud from dynamic LiDAR data (for velodyne-like LiDARs)")
    ("intensity_channel,i", value<std::string>()->default_value("auto"), "auto or channel name")
    // ("camera_info_topic", value<std::string>())
    // ("image_topic", value<std::string>())
    // ("points_topic", value<std::string>())
    ("camera_model", value<std::string>()->default_value("auto"), "auto, atan, plumb_bob, fisheye(=equidistant), omnidir, or equirectangular")
    ("camera_intrinsics", value<std::string>(), "camera intrinsic parameters [fx,fy,cx,cy(,xi)] (don't put spaces between values!!)")
    ("camera_distortion_coeffs", value<std::string>(), "camera distortion parameters [k1,k2,p1,p2,k3] (don't put spaces between values!!)")
    ("k_neighbors", value<int>()->default_value(30), "num of neighbor points used for point covariance estimation of CT-ICP")
    ("voxel_resolution", value<double>()->default_value(0.0005), "voxel grid resolution")
    ("min_distance", value<double>()->default_value( min_dis), "minimum point distance. Points closer than this value will be discarded")
    ("verbose", "if true, print out optimization status")
    // ("visualize,v", "if true, show extracted images and points")
  ;
  // clang-format on

  positional_options_description p;
  p.add("data_path", 1);
  p.add("dst_path", 1);

  variables_map vm;
  store(command_line_parser(argc, argv).options(description).positional(p).run(), vm);
  notify(vm);

  // if (vm.count("help") || !vm.count("data_path") || !vm.count("dst_path")) {
  //   std::cout << description << std::endl;
  //   return true;
  // }


 
  std::cout << "data_path: " << data_path << std::endl;
  std::cout << "dst_path : " << dst_path << std::endl;
  std::filesystem::create_directories(dst_path);

  std::vector<std::string> bag_filenames;
  for (const auto& path : std::filesystem::directory_iterator(data_path)) {
    if (!valid_bag(path.path().string())) {
      continue;
    }

    bag_filenames.emplace_back(path.path().string());
  }

  std::cout << "input_bags:" << std::endl;
  for (const auto& bag_filename : bag_filenames) {
    std::cout << "- " << bag_filename << std::endl;
  }

  if (bag_filenames.empty()) {
    std::cerr << vlcal::console::bold_red << "error: no input bags!!" << vlcal::console::reset << std::endl;
    return 1;
  }

  std::sort(bag_filenames.begin(), bag_filenames.end());

  if (vm.count("bag_id")) {
    const int bag_id = vm["bag_id"].as<int>();
    std::cerr << vlcal::console::bold_yellow << "use only " << bag_filenames[bag_id] << vlcal::console::reset << std::endl;
    const std::string bag_filename = bag_filenames[bag_id];
    bag_filenames = {bag_filename};
  }

  if (vm.count("first_n_bags")) {
    const int first_n_bags = vm["first_n_bags"].as<int>();
    bag_filenames.erase(bag_filenames.begin() + first_n_bags, bag_filenames.end());

    std::cerr << vlcal::console::bold_yellow << "use only the following rosbags:" << vlcal::console::reset << std::endl;
    for (const auto& bag_filename : bag_filenames) {
      std::cerr << vlcal::console::bold_yellow << "- " << bag_filename << vlcal::console::reset << std::endl;
    }
  }

  // topics
  // why omp causes errors for structured bindings?
  // const auto topics = get_topics(vm, bag_filenames.front());
  // const auto camera_info_topic = std::get<0>(topics);
  // const auto image_topic = std::get<1>(topics);
  // const auto points_topic = std::get<2>(topics);
  std::cout << "selected topics:" << std::endl;
  std::cout << "- camera_info: " << camera_info_topic << std::endl;
  std::cout << "- image      : " << image_topic << std::endl;
  std::cout << "- points     : " << points_topic << std::endl;

  // intensity channel
  const std::string intensity_channel = get_intensity_channel(vm, bag_filenames.front(), points_topic);
  std::cout << "intensity_channel: " << intensity_channel << std::endl;

  // camera params
  auto [camera_model, image_size, intrinsics, distortion_coeffs] = get_camera_params(vm, bag_filenames.front(), camera_info_topic, image_topic);
   
   std::cout << "intensity_channel4: " << intensity_channel << std::endl;

  camera::GenericCameraBase::ConstPtr proj = camera::create_camera(camera_model, intrinsics, distortion_coeffs);
    ViewCullingParams view_culling_params;
     std::cout << "intensity_channel3: " << intensity_channel << std::endl;

  view_culling_params.enable_depth_buffer_culling = false;
  
  ViewCulling_inital view_culling(proj, {image_size.width,image_size.height}, view_culling_params);
     std::cout << "intensity_channel2: " << intensity_channel << std::endl;

   
    // auto new_data = std::make_shared<VisualLiDARData>(data->image, culled_points);
  std::cout << "camera_model: " << camera_model << std::endl;
  std::cout << "image_size  : " << image_size.width << " " << image_size.height << std::endl;
  std::cout << "intrinsics  : " << Eigen::Map<const Eigen::VectorXd>(intrinsics.data(), intrinsics.size()).transpose() << std::endl;
  std::cout << "dist_coeffs : " << Eigen::Map<const Eigen::VectorXd>(distortion_coeffs.data(), distortion_coeffs.size()).transpose() << std::endl;

  // process bags
  int num_threads_per_bag = omp_get_max_threads();
  std::cout << "processing images and points (num_threads_per_bag=" << num_threads_per_bag << ")" << std::endl;

  lidar_points.resize(bag_filenames.size());

  // omp_set_max_active_levels(2);
 
//  if(status_flag)
    // #pragma omp parallel for
    for (int i = 0; i < bag_filenames.size(); i++) {
      std::cout << "start processing " << bag_filenames[i] << std::endl;

      const auto& bag_filename = bag_filenames[i];
       

      auto [image, points,image_rgb] = get_image_and_points(vm, bag_filename, image_topic, points_topic, intensity_channel, num_threads_per_bag);
       
    //给init_T_camera_lidar手动赋值
    if(cull_hidden_points)
     { 

      // Remove hidden points
    
    points = view_culling.cull(points, T_CL_init,0.4);}
    
 if(initial_alignment) 
      {
    for (int i = 0; i < points->size(); i++) {
      
      points->points[i] = T_L0C0* T_CL_init* points->points[i];
    }
      }
      lidar_points[i] = points;
      const std::string bag_name = std::filesystem::path(bag_filename).filename();
      cv::imwrite(dst_path + "/" + bag_name + ".png", image);
      cv::imwrite(dst_path + "/" + bag_name + "_rgb.png", image_rgb);

      std::string dst_path_l2e = dst_path.substr(0, dst_path.size()-1);
      cv::imwrite(dst_path_l2e + "_l2e/image/" + bag_name + ".png", image);
      cv::imwrite(dst_path_l2e + "_l2e/image/" + bag_name + "_rgb.png", image_rgb);
    
     }
  return true;
}
void Preprocess::save_point_forNID(std::vector<std::string> bag_filenames,std::string dst_path)
{   
  #pragma omp parallel for
  for (int i = 0; i < bag_filenames.size(); i++) 
  {
      std::cout << "start processing " << bag_filenames[i] << std::endl;

      const auto& bag_filename = bag_filenames[i];
 
      const std::string bag_name = std::filesystem::path(bag_filename).filename();
     
      glk::PLYData ply;
      ply.vertices.resize(lidar_points[i]->size());
      ply.intensities.resize(lidar_points[i]->size());
      
      std::transform(lidar_points[i]->points, lidar_points[i]->points + lidar_points[i]->size(), ply.vertices.begin(), [](const Eigen::Vector4d& p) { return p.cast<float>().head<3>(); });
      std::copy(lidar_points[i]->intensities, lidar_points[i]->intensities + lidar_points[i]->size(), ply.intensities.begin());
      glk::save_ply_binary(dst_path + "/" + bag_name + ".ply", ply);
      //保存激光点云到txt文件 (x, y, z, intensity) 坐标值，以逗号分隔,txt文件不存在时会自动创建
    //去掉dst_path的最后一个字符
    // std::string dst_path_l2e = dst_path.substr(0, dst_path.size()-1);
    //   std::ofstream ofs(dst_path_l2e + "_l2e/lidar/" + bag_name + ".txt");
    //   for (int j = 0; j < lidar_points[i]->size(); j++) {
    //     const Eigen::Vector4d& p = lidar_points[i]->points[j];
       
    //      ofs << p.x() << ", " << p.y() << ", " << p.z() << ", " << lidar_points[i]->intensities[j]*255 << std::endl;
    //   }
      // ofs.close();
      std::cout << "processed " << bag_filename << std::endl;
    }
}

void Preprocess::view_process(std::vector<std::string> bag_filenames,std::string dst_path)
{

   
      auto viewer = guik::LightViewer::instance();
      viewer->clear();
      viewer->use_arcball_camera_control();

      viewer->invoke([] {
        ImGui::SetNextWindowPos({1260, 60}, ImGuiCond_Once);
        ImGui::Begin("images");
        ImGui::End();
        ImGui::SetNextWindowPos({55, 300}, ImGuiCond_Once);
        ImGui::Begin("texts");
        ImGui::End();
      });

      for (const auto& bag_filename : bag_filenames) {
        const std::string bag_name = std::filesystem::path(bag_filename).filename();
        const cv::Mat image = cv::imread(dst_path + "/" + bag_name + ".png");
        const auto points = glk::load_ply(dst_path + "/" + bag_name + ".ply");

        auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points->vertices);
        cloud_buffer->add_intensity(glk::COLORMAP::TURBO, points->intensities);

        viewer->append_text(bag_filename);
        viewer->update_image("image", glk::create_texture(image));
        viewer->update_drawable("points", cloud_buffer, guik::VertexColor());
        viewer->spin_until_click();
        std::cout<<"bag_filenames"<<bag_filename<<std::endl;
      }
     //等待10s
    //  sleep(10);
    //   viewer->close();


}
cv::Mat Preprocess::hole_process(cv::Mat cropped_intensities_eq,cv::Mat cropped_indices_eq,bool indices,bool depth, int radius_eq ) {
  if(!indices)
{  cv::Mat result_eq = cropped_intensities_eq.clone();
    // 遍历图像每个像素
    for (int y = 0; y < cropped_intensities_eq.rows; y++) {
        for (int x = 0; x < cropped_intensities_eq.cols; x++) {
            // 如果当前像素为黑色孔洞
            if (cropped_intensities_eq.at<uchar>(y, x) == 0) {
                // 计算周围像素的平均值
                // index_image.at<std::int32_t>(pt_2d.y(), pt_2d.x())
                
                int sum_indices = 0;
                int count_indices = 0;
                //  for (int j = -radius; j <= radius; j++) {
                //     for (int i = -radius; i <= radius; i++) {
                //         int yy = std::max(0, std::min(cropped_indices_8uc4.rows - 1, y + j));
                //         int xx = std::max(0, std::min(cropped_indices_8uc4.cols - 1, x + i));
                //         sum_indices += cropped_indices_8uc4.at<uchar>(yy, xx);
                //         count_indices++;
                //     }
                // } 
                  // index_image.at<std::int32_t>(y, x)=index_image.at<std::int32_t>(y, x)
                int sum  = 0;
                int count  = 0;
                for (int j = -radius_eq; j <= radius_eq; j++) {
                    for (int i = -radius_eq; i <= radius_eq; i++) {
                        int yy = std::max(0, std::min(cropped_intensities_eq.rows - 1, y + j));
                        int xx = std::max(0, std::min(cropped_intensities_eq.cols - 1, x + i));
                        sum += cropped_intensities_eq.at<uchar>(yy, xx);
                        count++;
                    }
                }
                // 用平均值代替孔洞像素值
                result_eq.at<uchar>(y, x) = sum/ count;
                // result_indices.at<uchar>(y, x) =int(sum_indices / count_indices) ;
            }
        }
    }
    cv::Mat equalized;
    equalizeHist(result_eq.clone(), equalized);
     cv::Mat denoised_image;
    cv::medianBlur(equalized, denoised_image, 3); // 3x3中值滤波
    return equalized;
}
 cv::Mat result_indices_eq = cropped_indices_eq.clone();
    // cv::Mat result_indices = cropped_indices_8uc4.clone();

    // 定义孔洞大小（周围像素的范围）
    
    // 遍历图像每个像素
  if(indices){
    for (int y = 0; y < cropped_intensities_eq.rows; y++) {
        for (int x = 0; x < cropped_intensities_eq.cols; x++) {
            // 如果当前像素为黑色孔洞
            if (cropped_intensities_eq.at<uchar>(y, x) == 0) {
                // 计算周围像素的平均值
                // index_image.at<std::int32_t>(pt_2d.y(), pt_2d.x())
             
                int sum  = 0;
                int count  = 0;
                for (int j = -radius_eq; j <= radius_eq; j++) {
                    for (int i = -radius_eq; i <= radius_eq; i++) {
                        int yy = std::max(0, std::min(cropped_indices_eq.rows - 1, y + j));
                        int xx = std::max(0, std::min(cropped_indices_eq.cols - 1, x + i));
                       if(depth)
                         sum += cropped_indices_eq.at<uchar>(yy, xx);
                      else
                      //找(y, x)附近 result_indices_eq.at<uchar>(y, x) ！=0与(y, x)最解决的点


                       { 
                        sum += cropped_indices_eq.at<std::int32_t>(yy, xx);}
                        count++;
                    }
                }
                // 用平均值代替孔洞像素值
                // result_indices.at<uchar>(y, x) = sum/ count;
                  if(depth)
                result_indices_eq.at<uchar>(y, x) =sum/ count;
                else
                  {   

                    
                    result_indices_eq.at<std::int32_t>(y, x) =sum/ count;}
            }
        }
    }
      return result_indices_eq;
  }
  
}
struct PixelInfo {
  cv::Point pt;
  std::int32_t value;
};

bool inBounds(const cv::Mat& img, int x, int y) {
  return x >= 0 && x < img.cols && y >= 0 && y < img.rows;
}

cv::Mat replaceNegativeOnesWithNearestNonNegative(cv::Mat img2, int searchRange) {
    // 初始化队列，记录所有-1的位置
  cv::Mat img=img2.clone();
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            if (img.at<std::int32_t>(r, c) == -1) {
                bool found = false;
                // 对于每个-1的像素，搜索其邻域
                for (int radius = 1; radius <= searchRange && !found; radius++) {
                    for (int dy = -radius; dy <= radius; dy++) {
                        for (int dx = -radius; dx <= radius; dx++) {
                            if (dx * dx + dy * dy > radius * radius) continue; // Ensure the point is within a circle
                            int newX = c + dx;
                            int newY = r + dy;

                            if (inBounds(img, newX, newY) && img2.at<std::int32_t>(newY, newX) != -1) {
                                img.at<std::int32_t>(r, c) = img2.at<std::int32_t>(newY, newX);
                                found = true;
                                break;
                            }
                        }
                        if (found) break;
                    }
                }
            }
        }
    }

    return img;
}

 cv::Mat fill_in_fast(cv::Mat depth_map, double max_depth=100.0, cv::Mat custom_kernel=cv::Mat(),
                     bool extrapolate=false, std::string blur_type="bilateral") {
    // Invert

    
    // cv::Mat valid_pixels = depth_map > 0.1;

    //  for(int y = 0; y < depth_map.rows; y++) {
    //     for(int x = 0; x < depth_map.cols; x++) {
    //         float& pixel = depth_map.at<float>(y, x);
    //         // 检查像素值是否大于0.1。
    //         if(pixel > 0.1f) {
    //             // 修改像素值。
    //             depth_map.at<float>(y, x) = max_depth - pixel;
    //         }
    //     }
    // }


    
    // Dilate
    cv::dilate(depth_map, depth_map, custom_kernel);
cv::Mat FULL_KERNEL_5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    // Hole closing
    cv::morphologyEx(depth_map, depth_map, cv::MORPH_OPEN, FULL_KERNEL_5);

    // Fill empty spaces with dilated values
    cv::Mat empty_pixels = depth_map < 0.1;
    cv::Mat dilated;
    // cv::Mat FULL_KERNEL_7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // cv::dilate(depth_map, dilated, FULL_KERNEL_7);
    // depth_map.copyTo(dilated, empty_pixels);
    cv::Mat FULL_KERNEL_31 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(31, 31));
  cv::Mat mask;
      cv::Mat result = depth_map.clone();  // 从原图开始复制，以保持非掩膜区域不变

cv::compare(depth_map, 0.1, mask, cv::CMP_LT); 
    cv::Mat valid_pixels = depth_map < 0.1;
   cv::Mat dilatedImage;
    int dilation_size = 5;  // 膨胀程度
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
      cv::Size(5, 5));
    cv::dilate(depth_map, dilatedImage, element);

    // 使用掩膜从膨胀后的图像中选择性地保留像素
     dilatedImage.copyTo(result, mask);  // 将膨胀后的区域复制到结果图中的对应掩膜位置


    // Extend highest pixel to top of image
    if (extrapolate) {
        // ... This part is more complex in C++, you might need to write a loop

        // Large Fill
        empty_pixels = depth_map < 0.1;
        cv::dilate(depth_map, dilated, FULL_KERNEL_31);
        depth_map.copyTo(dilated, empty_pixels);
    }

    // Median blur
    cv::medianBlur(result, result, 3);
     cv::Mat blurred;
  // cv::GaussianBlur(result, blurred, cv::Size(3, 3), 0);
    // Bilateral or Gaussian blur
//     if (blur_type == "bilateral") {
//         // Bilateral blur
        cv::Mat filtered;
     cv::bilateralFilter(result, filtered, 5, 0.5, 2.0);
depth_map = filtered;
//     } else if (blur_type == "gaussian") {
//         // Gaussian blur
//         valid_pixels = depth_map > 0.1;
//         cv::Mat blurred;
//         cv::GaussianBlur(depth_map, blurred, cv::Size(5, 5), 0);
//         depth_map.copyTo(blurred, valid_pixels);
//     }

    // Invert
    // valid_pixels = depth_map > 0.1;




    //     for(int y = 0; y < depth_map.rows; y++) {
    //     for(int x = 0; x < depth_map.cols; x++) {
    //         float& pixel = depth_map.at<float>(y, x);
    //         // 检查像素值是否大于0.1。
    //         if(pixel > 0.1f) {
    //             // 修改像素值。
    //             depth_map.at<float>(y, x) = max_depth - pixel;
    //         }
    //     }
    // }


    return filtered;
}
cv::Mat fill_in_fast_velodyne(cv::Mat depth_map, double max_depth=100.0, cv::Mat custom_kernel=cv::Mat(),
                     bool extrapolate=false, std::string blur_type="bilateral") {
    // Invert

    
    // cv::Mat valid_pixels = depth_map > 0.1;

    //  for(int y = 0; y < depth_map.rows; y++) {
    //     for(int x = 0; x < depth_map.cols; x++) {
    //         float& pixel = depth_map.at<float>(y, x);
    //         // 检查像素值是否大于0.1。
    //         if(pixel > 0.1f) {
    //             // 修改像素值。
    //             depth_map.at<float>(y, x) = max_depth - pixel;
    //         }
    //     }
    // }


    
    // Dilate
    // cv::dilate(depth_map, depth_map, custom_kernel);
 

    // Fill empty spaces with dilated values
    cv::Mat empty_pixels = depth_map < 0.1;
    cv::Mat dilated;
    // cv::Mat FULL_KERNEL_7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // cv::dilate(depth_map, dilated, FULL_KERNEL_7);
    // depth_map.copyTo(dilated, empty_pixels);
    cv::Mat FULL_KERNEL_31 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(31, 31));
  cv::Mat mask;
      cv::Mat result = depth_map.clone();  // 从原图开始复制，以保持非掩膜区域不变

cv::compare(depth_map, 0.1, mask, cv::CMP_LT); 
    cv::Mat valid_pixels = depth_map < 0.1;
   cv::Mat dilatedImage;
    int dilation_size = 5;  // 膨胀程度
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
      cv::Size(3, 3));
    cv::dilate(depth_map, dilatedImage, element);

    // 使用掩膜从膨胀后的图像中选择性地保留像素
     dilatedImage.copyTo(result, mask);  // 将膨胀后的区域复制到结果图中的对应掩膜位置

//第二次膨胀
    cv::Mat empty_pixels2 = result < 0.1;
    cv::Mat dilated2;
    // cv::Mat FULL_KERNEL_7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // cv::dilate(depth_map, dilated, FULL_KERNEL_7);
    // depth_map.copyTo(dilated, empty_pixels);
    cv::Mat FULL_KERNEL_312 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(31, 31));
  cv::Mat mask2;
      cv::Mat result2 = result.clone();  // 从原图开始复制，以保持非掩膜区域不变

cv::compare(result, 0.1, mask2, cv::CMP_LT); 
    cv::Mat valid_pixels2 = result < 0.1;
   cv::Mat dilatedImage2;
    int dilation_size2 = 5;  // 膨胀程度
    cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT,
      cv::Size(5, 5));
    cv::dilate(result, dilatedImage2, element2);

    // 使用掩膜从膨胀后的图像中选择性地保留像素
     dilatedImage2.copyTo(result2, mask2);  // 将膨胀后的区域复制到结果图中的对应掩膜位置





    // Extend highest pixel to top of image
    if (extrapolate) {
        // ... This part is more complex in C++, you might need to write a loop

        // Large Fill
        empty_pixels = depth_map < 0.1;
        cv::dilate(depth_map, dilated, FULL_KERNEL_31);
        depth_map.copyTo(dilated, empty_pixels);
    }

    // Median blur
      cv::Mat blurredImage;
      // cv::GaussianBlur(result2, blurredImage, cv::Size(2, 2), 0);
    // cv::medianBlur(result2, result2, 3);
   
  // cv::GaussianBlur(result, blurred, cv::Size(3, 3), 0);
    // Bilateral or Gaussian blur
//     if (blur_type == "bilateral") {
//         // Bilateral blur
//         cv::Mat filtered;
//      cv::bilateralFilter(result, filtered, 5, 0.5, 2.0);
// depth_map = filtered;
//     } else if (blur_type == "gaussian") {
//         // Gaussian blur
//         valid_pixels = depth_map > 0.1;
//         cv::Mat blurred;
//         cv::GaussianBlur(depth_map, blurred, cv::Size(5, 5), 0);
//         depth_map.copyTo(blurred, valid_pixels);
//     }

    // Invert
    // valid_pixels = depth_map > 0.1;




    //     for(int y = 0; y < depth_map.rows; y++) {
    //     for(int x = 0; x < depth_map.cols; x++) {
    //         float& pixel = depth_map.at<float>(y, x);
    //         // 检查像素值是否大于0.1。
    //         if(pixel > 0.1f) {
    //             // 修改像素值。
    //             depth_map.at<float>(y, x) = max_depth - pixel;
    //         }
    //     }
    // }


    return result2;
}

 cv::Mat fill_in_fast_intensity(cv::Mat depth_map, double max_depth=100.0, cv::Mat custom_kernel=cv::Mat(),
                     bool extrapolate=false, std::string blur_type="bilateral") {
 
    // Dilate
    cv::dilate(depth_map, depth_map, custom_kernel);
// cv::Mat FULL_KERNEL_5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
//     // Hole closing
//     cv::morphologyEx(depth_map, depth_map, cv::MORPH_OPEN, FULL_KERNEL_5);

    // Fill empty spaces with dilated values
    cv::Mat empty_pixels = depth_map < 0.1;
    cv::Mat dilated;
    // cv::Mat FULL_KERNEL_7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // cv::dilate(depth_map, dilated, FULL_KERNEL_7);
    // depth_map.copyTo(dilated, empty_pixels);
    cv::Mat FULL_KERNEL_31 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
  cv::Mat mask;
      cv::Mat result = depth_map.clone();  // 从原图开始复制，以保持非掩膜区域不变

cv::compare(depth_map, 0.1, mask, cv::CMP_LT); 
    cv::Mat valid_pixels = depth_map < 0.1;
   cv::Mat dilatedImage;
    int dilation_size = 5;  // 膨胀程度
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
      cv::Size(5, 5));
    cv::dilate(depth_map, dilatedImage, element);

    // 使用掩膜从膨胀后的图像中选择性地保留像素
     dilatedImage.copyTo(result, mask);  // 将膨胀后的区域复制到结果图中的对应掩膜位置


    // Extend highest pixel to top of image
    if (extrapolate) {
        // ... This part is more complex in C++, you might need to write a loop

        // Large Fill
        empty_pixels = depth_map < 0.1;
        cv::dilate(depth_map, dilated, FULL_KERNEL_31);
        depth_map.copyTo(dilated, empty_pixels);
    }
   cv::Mat equalized;
    equalizeHist(result.clone(), equalized);
    // Median blur
    cv::medianBlur(equalized, equalized, 3);
    //  cv::Mat blurred;
  // cv::GaussianBlur(result, blurred, cv::Size(3, 3), 0);
    // Bilateral or Gaussian blur
//     if (blur_type == "bilateral") {
//         // Bilateral blur
//         cv::Mat filtered;
//      cv::bilateralFilter(result, filtered, 5, 0.5, 2.0);
// depth_map = filtered;
//     } else if (blur_type == "gaussian") {
//         // Gaussian blur
//         valid_pixels = depth_map > 0.1;
        // cv::Mat blurred;
        // cv::GaussianBlur(result, blurred, cv::Size(5, 5), 0);
//         depth_map.copyTo(blurred, valid_pixels);
//     }

    // Invert
    // valid_pixels = depth_map > 0.1;




    //     for(int y = 0; y < depth_map.rows; y++) {
    //     for(int x = 0; x < depth_map.cols; x++) {
    //         float& pixel = depth_map.at<float>(y, x);
    //         // 检查像素值是否大于0.1。
    //         if(pixel > 0.1f) {
    //             // 修改像素值。
    //             depth_map.at<float>(y, x) = max_depth - pixel;
    //         }
    //     }
    // }


    return equalized;
}

 cv::Mat fill_in_fast_intensity_velodyne(cv::Mat depth_map, double max_depth=100.0, cv::Mat custom_kernel=cv::Mat(),
                     bool extrapolate=false, std::string blur_type="bilateral") {
     
        cv::Mat custom_kernel2 = (cv::Mat_<uchar>(3, 3)<<
       0, 1, 0,
        1, 1, 1,
        0, 1, 0);
    // Dilate
    // cv::dilate(depth_map, depth_map, custom_kernel2);
// cv::Mat FULL_KERNEL_5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
//     // Hole closing
//     cv::morphologyEx(depth_map, depth_map, cv::MORPH_OPEN, FULL_KERNEL_5);

    // Fill empty spaces with dilated values
    cv::Mat empty_pixels = depth_map < 0.1;
    cv::Mat dilated;
    // cv::Mat FULL_KERNEL_7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // cv::dilate(depth_map, dilated, FULL_KERNEL_7);
    // depth_map.copyTo(dilated, empty_pixels);
    cv::Mat FULL_KERNEL_31 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
  cv::Mat mask;
      cv::Mat result = depth_map.clone();  // 从原图开始复制，以保持非掩膜区域不变

cv::compare(depth_map, 0.1, mask, cv::CMP_LT); 
    cv::Mat valid_pixels = depth_map < 0.1;
   cv::Mat dilatedImage;
    int dilation_size = 3;  // 膨胀程度
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
      cv::Size(3, 3));
    cv::dilate(depth_map, dilatedImage, element);

    // 使用掩膜从膨胀后的图像中选择性地保留像素
     dilatedImage.copyTo(result, mask);  // 将膨胀后的区域复制到结果图中的对应掩膜位置


    // Extend highest pixel to top of image
 
        // ... This part is more complex in C++, you might need to write a loop
   cv::Mat dilatedImage2;
    cv::Mat result2 = result.clone();  //
        // Large Fill
        cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT,
      cv::Size(3, 3));
        empty_pixels = result < 0.1;
        cv::dilate(result, dilatedImage2, element2);
        dilatedImage2.copyTo(result2, empty_pixels);
   
   cv::Mat equalized;
    equalizeHist(result2.clone(), equalized);
    // Median blur
    // cv::medianBlur(equalized, equalized, 3);
     cv::Mat medianBlurblurred;
  cv::medianBlur(equalized, medianBlurblurred, 3);
    // Bilateral or Gaussian blur
//     if (blur_type == "bilateral") {
//         // Bilateral blur
//         cv::Mat filtered;
//      cv::bilateralFilter(result, filtered, 5, 0.5, 2.0);
// depth_map = filtered;
//     } else if (blur_type == "gaussian") {
//         // Gaussian blur
//         valid_pixels = depth_map > 0.1;
        // cv::Mat blurred;
        // cv::GaussianBlur(result, blurred, cv::Size(5, 5), 0);
//         depth_map.copyTo(blurred, valid_pixels);
//     }

    // Invert
    // valid_pixels = depth_map > 0.1;




    //     for(int y = 0; y < depth_map.rows; y++) {
    //     for(int x = 0; x < depth_map.cols; x++) {
    //         float& pixel = depth_map.at<float>(y, x);
    //         // 检查像素值是否大于0.1。
    //         if(pixel > 0.1f) {
    //             // 修改像素值。
    //             depth_map.at<float>(y, x) = max_depth - pixel;
    //         }
    //     }
    // }


    return medianBlurblurred;
}
bool Preprocess::run(int argc, char** argv, int status_flag, Eigen::Isometry3d init_L_C)
 {
  using namespace boost::program_options;
  options_description description("preprocess");

   // clang-format off
  description.add_options()
    ("help", "produce help message")
    ("data_path", value<std::string>(), "directory that contains rosbags for calibration")
    ("dst_path", value<std::string>(), "directory to save preprocessed data")
    ("bag_id", value<int>(), "specify the bag to use (just for evaluation)")
    ("first_n_bags", value<int>(), "use only the first N bags (just for evaluation)")
    ("auto_topic,a", "automatically select topics")
    ("dynamic_lidar_integration,d", "create target point cloud from dynamic LiDAR data (for velodyne-like LiDARs)")
    ("intensity_channel,i", value<std::string>()->default_value("auto"), "auto or channel name")
    // ("camera_info_topic", value<std::string>())
    // ("image_topic", value<std::string>())
    // ("points_topic", value<std::string>())
    ("camera_model", value<std::string>()->default_value("auto"), "auto, atan, plumb_bob, fisheye(=equidistant), omnidir, or equirectangular")
    ("camera_intrinsics", value<std::string>(), "camera intrinsic parameters [fx,fy,cx,cy(,xi)] (don't put spaces between values!!)")
    ("camera_distortion_coeffs", value<std::string>(), "camera distortion parameters [k1,k2,p1,p2,k3] (don't put spaces between values!!)")
    ("k_neighbors", value<int>()->default_value(30), "num of neighbor points used for point covariance estimation of CT-ICP")
    ("voxel_resolution", value<double>()->default_value(0.0005), "voxel grid resolution")
    ("min_distance", value<double>()->default_value(0.3), "minimum point distance. Points closer than this value will be discarded")
    ("verbose", "if true, print out optimization status")
    ("visualize,v", "if true, show extracted images and points")
  ;
  // clang-format on

  positional_options_description p;
  p.add("data_path", 1);
  p.add("dst_path", 1);

  variables_map vm;
  store(command_line_parser(argc, argv).options(description).positional(p).run(), vm);
  notify(vm);

  // if (vm.count("help") || !vm.count("data_path") || !vm.count("dst_path")) {
  //   std::cout << description << std::endl;
  //   return true;
  // }

  // const std::string data_path = vm["data_path"].as<std::string>();
  // const std::string dst_path = vm["dst_path"].as<std::string>();
  std::cout << "data_path: " << data_path << std::endl;
  std::cout << "dst_path : " << dst_path << std::endl;
  std::filesystem::create_directories(dst_path);

  std::vector<std::string> bag_filenames;
  for (const auto& path : std::filesystem::directory_iterator(data_path)) {
    if (!valid_bag(path.path().string())) {
      continue;
    }

    bag_filenames.emplace_back(path.path().string());
  }

  std::cout << "input_bags:" << std::endl;
  for (const auto& bag_filename : bag_filenames) {
    std::cout << "- " << bag_filename << std::endl;
  }

  if (bag_filenames.empty()) {
    std::cerr << vlcal::console::bold_red << "error: no input bags!!" << vlcal::console::reset << std::endl;
    return 1;
  }

  std::sort(bag_filenames.begin(), bag_filenames.end());

  if (vm.count("bag_id")) {
    const int bag_id = vm["bag_id"].as<int>();
    std::cerr << vlcal::console::bold_yellow << "use only " << bag_filenames[bag_id] << vlcal::console::reset << std::endl;
    const std::string bag_filename = bag_filenames[bag_id];
    bag_filenames = {bag_filename};
  }

  if (vm.count("first_n_bags")) {
    const int first_n_bags = vm["first_n_bags"].as<int>();
    bag_filenames.erase(bag_filenames.begin() + first_n_bags, bag_filenames.end());

    std::cerr << vlcal::console::bold_yellow << "use only the following rosbags:" << vlcal::console::reset << std::endl;
    for (const auto& bag_filename : bag_filenames) {
      std::cerr << vlcal::console::bold_yellow << "- " << bag_filename << vlcal::console::reset << std::endl;
    }
  }

  // topics
  // why omp causes errors for structured bindings?
  // const auto topics = get_topics(vm, bag_filenames.front());
  // const auto camera_info_topic = std::get<0>(topics);
  // const auto image_topic = std::get<1>(topics);
  // const auto points_topic = std::get<2>(topics);
  std::cout << "selected topics:" << std::endl;
  std::cout << "- camera_info: " << camera_info_topic << std::endl;
  std::cout << "- image      : " << image_topic << std::endl;
  std::cout << "- points     : " << points_topic << std::endl;

  // intensity channel
  const std::string intensity_channel = get_intensity_channel(vm, bag_filenames.front(), points_topic);
  std::cout << "intensity_channel: " << intensity_channel << std::endl;
  
  // camera params
  auto [camera_model, image_size, intrinsics, distortion_coeffs] = get_camera_params(vm, bag_filenames.front(), camera_info_topic, image_topic);
  
  
  camera::GenericCameraBase::ConstPtr proj = camera::create_camera(camera_model, intrinsics, distortion_coeffs);
    ViewCullingParams view_culling_params;

  view_culling_params.enable_depth_buffer_culling = false;
  ViewCulling_inital view_culling(proj, {image_size.width,image_size.height}, view_culling_params );
     
 for(int i=0;i<bag_filenames.size();i++) 
   {  
  if(status_flag>0||cull_hidden_points)
     
  {lidar_points[i] = view_culling.cull(lidar_points[i], init_L_C,0);}
   }
   
  // lidar_points[i] = view_culling.cull(lidar_points[i], init_L_C,0.4-status_flag*0.1);
    // auto new_data = std::make_shared<VisualLiDARData>(data->image, culled_points,double status_flag);
  std::cout << "camera_model: " << camera_model << std::endl;
  std::cout << "image_size  : " << image_size.width << " " << image_size.height << std::endl;
  std::cout << "intrinsics  : " << Eigen::Map<const Eigen::VectorXd>(intrinsics.data(), intrinsics.size()).transpose() << std::endl;
  std::cout << "dist_coeffs : " << Eigen::Map<const Eigen::VectorXd>(distortion_coeffs.data(), distortion_coeffs.size()).transpose() << std::endl;

  // process bags
  int num_threads_per_bag = omp_get_max_threads();
  std::cout << "processing images and points (num_threads_per_bag=" << num_threads_per_bag << ")" << std::endl;

  // std::vector<Frame::ConstPtr> lidar_points(bag_filenames.size());

  // omp_set_max_active_levels(2);
  // #pragma omp parallel for
 
 {
    
    std::cout << "lidar_points:" << lidar_points.size() << std::endl;
    std::cout << "lidar_points[0]:" << lidar_points.front()->size() << std::endl;

    // Generate LiDAR images
    const double lidar_fov = vlcal::estimate_lidar_fov(lidar_points.front());
    std::cout << "LiDAR FoV: " << lidar_fov * 180.0 / M_PI << "[deg]" << std::endl;
    Eigen::Vector2i lidar_image_size,lidar_image_size_equirectangular;
    std::string lidar_camera_model,lidar_camera_model_equirectangular;
    std::vector<double> lidar_camera_intrinsics;
    std::vector<double> lidar_camera_intrinsics_equirectangular;

    Eigen::Isometry3d T_lidar_camera = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_lidar_camera_equirectangular = Eigen::Isometry3d::Identity();

    if (lidar_fov < 150.0 * M_PI / 180.0) {
   lidar_image_size = {1024, 1024};
    T_lidar_camera.linear() = (Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ())).toRotationMatrix();
    const double fx = lidar_image_size.x() / (2.0 * std::tan(lidar_fov / 2.0));
    lidar_camera_model = "plumb_bob";
    lidar_camera_intrinsics = {fx, fx, lidar_image_size[0] / 2.0, lidar_image_size[1] / 2.0};
   
    lidar_image_size_equirectangular = {1920,960};
    T_lidar_camera_equirectangular.linear() =(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX())).toRotationMatrix();
     lidar_camera_model_equirectangular ="equirectangular";
    lidar_camera_intrinsics_equirectangular ={static_cast<double>(lidar_image_size_equirectangular[0]), static_cast<double>(lidar_image_size_equirectangular[1])};
      pinhole_if_pre=true;
      // lidar_image_size = {1920,960};
      // T_lidar_camera.linear() = (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX())).toRotationMatrix();
      // lidar_camera_model = "equirectangular";
      // lidar_camera_intrinsics = {static_cast<double>(lidar_image_size[0]), static_cast<double>(lidar_image_size[1])};

     } 
     else 
     {lidar_image_size = {1024, 1024};
    T_lidar_camera.linear() = (Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ())).toRotationMatrix();
    const double fx = lidar_image_size.x() / (2.0 * std::tan(lidar_fov / 2.0));
    lidar_camera_model = "plumb_bob";
    lidar_camera_intrinsics = {fx, fx, lidar_image_size[0] / 2.0, lidar_image_size[1] / 2.0};
   

     lidar_image_size_equirectangular = {1920,960};
    T_lidar_camera_equirectangular.linear() =(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX())).toRotationMatrix();
     lidar_camera_model_equirectangular ="equirectangular";
    lidar_camera_intrinsics_equirectangular ={static_cast<double>(lidar_image_size_equirectangular[0]), static_cast<double>(lidar_image_size_equirectangular[1])};
      lidar_camera_intrinsics_equirectangular = {static_cast<double>(lidar_image_size[0]), static_cast<double>(lidar_image_size[1])};
   pinhole_if_pre=false;
    }

    auto lidar_proj = camera::create_camera(lidar_camera_model, lidar_camera_intrinsics, {});
    auto lidar_proj_equirectangular = camera::create_camera(lidar_camera_model_equirectangular, lidar_camera_intrinsics_equirectangular, {});

  
  #pragma omp parallel for
    for (int i = 0; i < bag_filenames.size(); i++) {
      const std::string bag_name = std::filesystem::path(bag_filenames[i]).filename();
        std::cout << "save LiDAR images" << std::endl;
    //计时器
   
      auto [intensities, depth_image,indices] = vlcal::generate_lidar_image(lidar_proj, lidar_image_size, T_lidar_camera.inverse(), lidar_points[i]);
      auto [intensities_eq,depth_image_eq,indices_eq] = vlcal::generate_lidar_image(lidar_proj_equirectangular, lidar_image_size_equirectangular, T_lidar_camera_equirectangular.inverse(), lidar_points[i]);
            

      intensities_eq.clone().convertTo(intensities_eq, CV_8UC1, 255.0);
      depth_image_eq.clone().convertTo(depth_image_eq, CV_8UC1, 255.0);

      cv::Rect roi_eq = cv::boundingRect(intensities_eq);
     cv::Mat index_image_eq=indices_eq;  //(image_size[1], image_size[0], CV_32SC1, cv::Scalar::all(-1));
//  index_image.at<std::int32_t>(pt_2d.y(), pt_2d.x()) = i;
     cv::Mat cropped_intensities_eq = intensities_eq(roi_eq);
      cv::Mat copy_intensities_eq=cropped_intensities_eq.clone();
         cv::Mat copy_intensities_eq_save=cropped_intensities_eq.clone();
          cv::Mat copy_intensities_eq_save2=cropped_intensities_eq.clone();
      cv::Mat cropped_indices_eq = indices_eq(roi_eq);
       cv::Mat copy_cropped_indices_eq=cropped_indices_eq;
      cv::Mat cropped_depth_eq = depth_image_eq(roi_eq);
       
     std::cout<<"1hole_processddddddddddddddd"<<std::endl;

     cv::Mat intensities_eq_eh=hole_process(cropped_intensities_eq,cropped_indices_eq,false,false,2);
     std::cout<<"hole_processddddddddddddddd"<<std::endl;
     copy_cropped_indices_eq=replaceNegativeOnesWithNearestNonNegative(copy_cropped_indices_eq,3);
      // cv::Mat indices_eq_eh=hole_process(copy_intensities_eq,cropped_indices_eq,true,false,2);
     std::cout<<"3hole_processddddddddddddddd"<<std::endl;

       cv::Mat depth_image_eq_eh=hole_process(copy_intensities_eq,cropped_depth_eq,true,true,2);
              std::cout<<"4hole_processddddddddddddddd"<<std::endl;

     cv::Mat index_image=indices;  //(image_size[1], image_size[0], CV_32SC1, cv::Scalar::all(-1));
//  index_image.at<std::int32_t>(pt_2d.y(), pt_2d.x()) = i;


     intensities.clone().convertTo(intensities, CV_8UC1, 255.0);
      // depth_image=depth_image/100;
      depth_image.clone().convertTo(depth_image, CV_8UC1, 255.0);   
      cv::Mat depth_image_copy2=depth_image.clone();

          cv::Rect roi = cv::boundingRect(intensities);
 cv::Mat cropped_intensities = intensities(roi);
      cv::Mat copy_intensities=cropped_intensities.clone();
      cv::Mat copy_intensities_save=cropped_intensities.clone();
      cv::Mat copy_intensities_save2=cropped_intensities.clone();

      cv::Mat cropped_indices= indices(roi);
       cv::Mat copy_cropped_indices=cropped_indices.clone();
         cv::Mat copy_cropped_indices_save=cropped_indices.clone();
      cv::Mat cropped_depth = depth_image(roi);
 cv::Mat copy_cropped_depth=cropped_depth.clone() ;
 cv::Mat copy_cropped_depth_save=cropped_depth.clone() ;

     cv::Mat intensities_eh=hole_process(cropped_intensities,cropped_indices,false,false,5);
        cropped_indices=replaceNegativeOnesWithNearestNonNegative(cropped_indices ,3);

      // cv::Mat indices_eh=hole_process(copy_intensities,cropped_indices,true,false,2);
       cv::Mat depth_image_eh=hole_process(copy_intensities,cropped_depth,true,true,3);
     cv::Mat custom_kernel = (cv::Mat_<uchar>(3, 3)
      <<
       0, 1, 0,
        1, 1, 1,
        0, 1, 0);
       cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
            
      cv::Mat depth_fill_in_fast,depth_fill_in_fast_eq,cropped_intensities_fast,cropped_intensities_fast_eq;
     if(LiDAR_type==0)
     { depth_fill_in_fast=fill_in_fast_velodyne( cropped_depth, 255.0, custom_kernel,false, "gaussian");
       depth_fill_in_fast_eq=fill_in_fast_velodyne( cropped_depth_eq, 255.0, custom_kernel,false, "gaussian");
      cropped_intensities_fast=fill_in_fast_intensity_velodyne( copy_intensities_save, 255.0, custom_kernel,false, "gaussian");
      cropped_intensities_fast_eq=fill_in_fast_intensity_velodyne( copy_intensities_eq_save, 255.0, custom_kernel,false, "gaussian");
     }
     else { depth_fill_in_fast=fill_in_fast( cropped_depth, 255.0, custom_kernel,false, "gaussian");
       depth_fill_in_fast_eq=fill_in_fast( cropped_depth_eq, 255.0, custom_kernel,false, "gaussian");
      cropped_intensities_fast=fill_in_fast_intensity( copy_intensities_save, 255.0, custom_kernel,false, "gaussian");
      cropped_intensities_fast_eq=fill_in_fast_intensity( copy_intensities_eq_save, 255.0, custom_kernel,false, "gaussian");
     }
      cv::Mat indiceseq_8uc4_save(copy_cropped_indices_eq.rows, copy_cropped_indices_eq.cols, CV_8UC4, reinterpret_cast<cv::Vec4b*>(copy_cropped_indices_eq.data));
      cv::Mat indices_8uc4_save(cropped_indices.rows, cropped_indices.cols, CV_8UC4, reinterpret_cast<cv::Vec4b*>(cropped_indices.data));
    cv::Mat indices_8uc4_save2(copy_cropped_indices_save.rows, copy_cropped_indices_save.cols, CV_8UC4, reinterpret_cast<cv::Vec4b*>(copy_cropped_indices_save.data));
      cv::imwrite(dst_path + "/" + bag_name + "_lidar_depth.png", depth_fill_in_fast);
      cv::imwrite(dst_path + "/" + bag_name + "_lidar_intensities.png", cropped_intensities_fast);
      cv::imwrite(dst_path + "/" + bag_name + "_lidar_indices.png", indices_8uc4_save);
 
      cv::imwrite(dst_path + "/" + bag_name + "_lidar_depth_eq.png", depth_fill_in_fast_eq);
      cv::imwrite(dst_path + "/" + bag_name + "_lidar_intensities_eq.png", cropped_intensities_fast_eq);
      cv::imwrite(dst_path + "/" + bag_name + "_lidar_indices_eq.png", indiceseq_8uc4_save);


      cv::imwrite(dst_path + "/" + bag_name + "_lidar_intensities_save.png", copy_intensities_save2);
      cv::imwrite(dst_path + "/" + bag_name + "_lidar_indices_save.png", indiceseq_8uc4_save);
      cv::imwrite(dst_path + "/" + bag_name + "_lidar_depth_save.png", copy_cropped_depth_save);
      
    //   cv::imwrite(dst_path + "/" + bag_name + "_lidar_intensities.png", cropped_intensities);
      // cv::imwrite(dst_path + "/" + bag_name + "_lidar_indices.png", cropped_indices);
      // cv::imwrite(dst_path + "/" + bag_name + "_lidar_intensities_eq.png", cropped_intensities_eq);
      // cv::imwrite(dst_path + "/" + bag_name + "_lidar_indices_eq.png", cropped_indices_eq);
    }
   
    //
    std::vector<std::string> bag_names(bag_filenames);
    std::transform(bag_filenames.begin(), bag_filenames.end(), bag_names.begin(), [](const auto& path) { return std::filesystem::path(path).filename(); });

    std::cout << "save meta data" << std::endl;

    nlohmann::json config;
    config["meta"]["data_path"] = data_path;
    config["meta"]["camera_info_topic"] = camera_info_topic;
    config["meta"]["image_topic"] = image_topic;
    config["meta"]["points_topic"] = points_topic;
    config["meta"]["intensity_channel"] = intensity_channel;
    config["meta"]["bag_names"] = bag_names;
    config["camera"]["camera_model"] = camera_model;
    config["camera"]["intrinsics"] = intrinsics;
    config["camera"]["distortion_coeffs"] = distortion_coeffs;

    std::ofstream ofs(dst_path + "/calib.json");
    ofs << config.dump(2) << std::endl;
    // if(status_flag==4)
      auto start = std::chrono::system_clock::now();
    {save_point_forNID(bag_filenames,dst_path);};
    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "elapsed time: " << duration.count() << " [ms]" << std::endl; 
  if(status_flag==4)
    {view_process(bag_filenames,dst_path);}
    
  return true;
}
 }
std::tuple<std::string, std::string, std::string> Preprocess::get_topics(const boost::program_options::variables_map& vm, const std::string& bag_filename) {
  std::string camera_info_topic;
  std::string image_topic;
  std::string points_topic;

  if (auto_topic) {
    const auto topics_and_types = get_topics_and_types(bag_filename);
    std::cout << "topics in " << bag_filename << ":" << std::endl;
    for (const auto& [topic, type] : topics_and_types) {
      std::cout << "- " << topic << " : " << type << std::endl;

      if (type.find("CameraInfo") != std::string::npos) {
        if (!camera_info_topic.empty()) {
          std::cerr << vlcal::console::bold_yellow << "warning: bag constains multiple camera_info topics!!" << vlcal::console::reset << std::endl;
        }
        camera_info_topic = topic;
      } else if (type.find("Image") != std::string::npos) {
        if (!image_topic.empty()) {
          std::cerr << vlcal::console::bold_yellow << "warning: bag constains multiple image topics!!" << vlcal::console::reset << std::endl;
        }
        image_topic = topic;
      } else if (type.find("PointCloud2") != std::string::npos) {
        if (!points_topic.empty()) {
          std::cerr << vlcal::console::bold_yellow << "warning: bag constains multiple points topics!!" << vlcal::console::reset << std::endl;
        }
        points_topic = topic;
      }
    }
  }

  if (camera_info_topic.empty() && vm.count("camera_info_topic")) {
    camera_info_topic = vm["camera_info_topic"].as<std::string>();
  }
  if (image_topic.empty() && vm.count("image_topic")) {
    image_topic = vm["image_topic"].as<std::string>();
  }
  if (points_topic.empty() && vm.count("points_topic")) {
    points_topic = vm["points_topic"].as<std::string>();
  }

  if (camera_info_topic.empty()) {
    std::cerr << console::bold_yellow << "warning: failed to get camera_info topic!!" << console::reset << std::endl;
  }
  if (image_topic.empty()) {
    std::cerr << console::bold_yellow << "warning: failed to get image topic!!" << console::reset << std::endl;
  }
  if (points_topic.empty()) {
    std::cerr << console::bold_yellow << "warning: failed to get points topic!!" << console::reset << std::endl;
  }

  return {camera_info_topic, image_topic, points_topic};
}

std::string Preprocess::get_intensity_channel(const boost::program_options::variables_map& vm, const std::string& bag_filename, const std::string& points_topic) {
  std::string intensity_channel = vm["intensity_channel"].as<std::string>();
  if (intensity_channel != "auto") {
    return intensity_channel;
  }

  std::unordered_map<std::string, int> channel_priorities;
  channel_priorities["auto"] = -1;
  channel_priorities["intensity"] = 1;
  channel_priorities["reflectivity"] = 2;

  const auto point_fields = get_point_fields(bag_filename, points_topic);
  for (const auto& field : point_fields) {
    if (!channel_priorities.count(field)) {
      continue;
    }

    if (channel_priorities[intensity_channel] < channel_priorities[field]) {
      intensity_channel = field;
    }
  }

  if (intensity_channel == "auto") {
    std::cerr << vlcal::console::bold_red << "error: failed to determine point intensity channel automatically" << vlcal::console::reset << std::endl;
    std::cerr << vlcal::console::bold_red << "     : you must specify the intensity channel to be used manually" << vlcal::console::reset << std::endl;
  }

  return intensity_channel;
}
std::tuple<std::string, cv::Size, std::vector<double>, std::vector<double>> Preprocess::get_manual_camera_params(
  const boost::program_options::variables_map& vm,
  const std::string& bag_filename,
  const std::string& camera_info_topic,
  const std::string& image_topic) {
  //
  cv::Size image_size = get_image_size(bag_filename, image_topic);
  if (image_size.width == 0 && image_size.height == 0) {
    std::cerr << vlcal::console::bold_yellow << "warning: image size is not set (image_topic=" << image_topic << ")" << vlcal::console::reset << std::endl;
  }

  std::string camera_model =  "plumb_bob" ;
 

  std::vector<double> intrinsics = {1215.16206202384, 1215.5773507687247,931.46704145638,541.5467734550551};
std::vector<double> distortion_coeffs = {-0.36565734884394796, 0.19454341750019136, -0.0003778025271473721, 9.299129873097343e-05, -0.0637399451159775};

    return {camera_model, image_size, intrinsics, distortion_coeffs};
   

  
 }
std::tuple<std::string, cv::Size, std::vector<double>, std::vector<double>> Preprocess::get_camera_params(
  const boost::program_options::variables_map& vm,
  const std::string& bag_filename,
  const std::string& camera_info_topic,
  const std::string& image_topic) {
  //
  cv::Size image_size = get_image_size(bag_filename, image_topic);
  if (image_size.width == 0 && image_size.height == 0) {
    std::cerr << vlcal::console::bold_yellow << "warning: image size is not set (image_topic=" << image_topic << ")" << vlcal::console::reset << std::endl;
  }

  std::string camera_model = vm["camera_model"].as<std::string>();
  if (camera_model != "auto") {
    const std::unordered_set<std::string> valid_camera_models = {"plumb_bob", "fisheye", "equidistant", "omnidir", "equirectangular"};
    if (!valid_camera_models.count(camera_model)) {
      std::cerr << vlcal::console::bold_red << "error: invalid camera model " << camera_model << vlcal::console::reset << std::endl;

      std::stringstream sst;
      for (const auto& model : valid_camera_models) {
        sst << " " << model;
      }
      std::cerr << vlcal::console::bold_red << "     : supported camera models are" << sst.str() << vlcal::console::reset << std::endl;

      abort();
    }

    std::vector<double> intrinsics;
    std::vector<double> distortion_coeffs;
    if (camera_model == "equirectangular") {
      intrinsics = {static_cast<double>(image_size.width), static_cast<double>(image_size.height)};
    } else {
      if (!vm.count("camera_intrinsics")) {
        std::cerr << vlcal::console::bold_red << "error: camera_intrinsics has not been set!!" << vlcal::console::reset << std::endl;
      }
      if (!vm.count("camera_distortion_coeffs")) {
        std::cerr << vlcal::console::bold_red << "error: camera_distortion_coeffs has not been set!!" << vlcal::console::reset << std::endl;
      }

      std::vector<std::string> intrinsic_tokens;
      std::vector<std::string> distortion_tokens;
      boost::split(intrinsic_tokens, vm["camera_intrinsics"].as<std::string>(), boost::is_any_of(","));
      boost::split(distortion_tokens, vm["camera_distortion_coeffs"].as<std::string>(), boost::is_any_of(","));

      intrinsics.resize(intrinsic_tokens.size());
      distortion_coeffs.resize(distortion_tokens.size());
      std::transform(intrinsic_tokens.begin(), intrinsic_tokens.end(), intrinsics.begin(), [](const auto& token) { return std::stod(token); });
      std::transform(distortion_tokens.begin(), distortion_tokens.end(), distortion_coeffs.begin(), [](const auto& token) { return std::stod(token); });
    }

    return {camera_model, image_size, intrinsics, distortion_coeffs};
  }

  std::cout << "try to get the camera model automatically" << std::endl;
  auto [distortion_model, intrinsics, distortion_coeffs] = get_camera_info(bag_filename, camera_info_topic);
   std::cout << "try to get the camera model automatically2" << std::endl;

  return {distortion_model, image_size, intrinsics, distortion_coeffs};
}
int adaptiveBins(std::vector<double>& intensities) {
    // 计算强度值范围
    double minIntensity = *std::min_element(intensities.begin(), intensities.end());
    double maxIntensity = *std::max_element(intensities.begin(), intensities.end());
    double intensityRange = maxIntensity - minIntensity;

    // 根据强度值范围和数据量选择 bins 值
    int numBins = static_cast<int>(std::sqrt(intensities.size() * intensityRange / (maxIntensity - minIntensity)));

    return numBins;
}
void globalHistogramEqualization(std::vector<double>& intensities) {
    // 计算直方图
    std::vector<int> histogram(256, 0);
    for (double intensity : intensities) {
        int binIndex = static_cast<int>(intensity);
        histogram[binIndex]++;
    }

    // 计算累积分布函数（CDF）
    std::vector<double> cdf(256, 0);
    cdf[0] = histogram[0];
    for (int i = 1; i < 256; ++i) {
        cdf[i] = cdf[i - 1] + histogram[i];
    }

    // 归一化 CDF
    int totalPixels = intensities.size();
for (int i = 0; i < 256; ++i) {
    cdf[i] /= totalPixels;
}

    // 应用均衡化
    for (int i = 0; i < intensities.size(); ++i) {
        intensities[i] = cdf[static_cast<int>(intensities[i])] ;
    }
}
 

std::tuple<cv::Mat, Frame::ConstPtr,cv::Mat> Preprocess::get_image_and_points(
  const boost::program_options::variables_map& vm,
  const std::string& bag_filename,
  const std::string& image_topic,
  const std::string& points_topic,
  const std::string& intensity_channel,
  const int num_threads) {
  //
  auto [image,image_rgb] = get_image(bag_filename, image_topic);
  if (!image.data) {
    std::cerr << vlcal::console::bold_red << "error: failed to obtain an image (image_topic=" << image_topic << ")" << vlcal::console::reset << std::endl;
    abort();
  }

  //cv::imread读取RGB图像转化为灰度图像
//  cv::Mat image_rgb2 =cv::imread("/calib_data/data/result.png");
//   cv::cvtColor(image_rgb2, image, cv::COLOR_BGR2GRAY);
 
 
  // 
//  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
//   clahe->apply(image, image);
// double gamma = 1.01;

//     // 进行伽马变换
//         image.convertTo(image, CV_32F, 1.0 / 255.0);  // 将像素值缩放到 [0, 1] 范围

//   cv::Mat gammaCorrected;
//     cv::pow(image, gamma, gammaCorrected);

//     // 将浮点型图像转换为8位无符号整数图像
//     gammaCorrected *= 255;
//     gammaCorrected.convertTo(gammaCorrected, CV_8U);
//     image = gammaCorrected;
// cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
// clahe->setTilesGridSize(cv::Size(16, 16)); // 设置tileGridSize的值
// cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(10.0, cv::Size(4,4));
    
// clahe->apply(image, image);
 cv::Mat equalized;
cv::equalizeHist(image.clone(), equalized);

cv:: Mat hist;
    int histSize = 256;
    float range[] = { 0, 256 }; // 像素值范围
    const float* histRange = { range };
    calcHist(&equalized, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false);

    // 计算累积直方图
   cv:: Mat accum_hist = hist.clone();
    for (int i = 1; i < histSize; i++) {
        accum_hist.at<float>(i) += accum_hist.at<float>(i - 1);
    }
    accum_hist /= image.total();

    // 计算自适应Gamma值
    float gamma = 0;
    for (int i = 0; i < histSize; i++) {
        if (accum_hist.at<float>(i) > 0.01) {
            gamma = 1 - i / 255.0; // 根据直方图计算Gamma值
            break;
        }
    }
 equalized.convertTo(equalized, CV_32F, 1.0 / 255.0);
    // 应用Gamma校正
    cv::Mat corrected;
    pow(equalized , gamma, corrected);

    // 将校正后的图像转换回8位图像
    corrected.convertTo(corrected, CV_8U, 255.0);
  // integrate points
  TimeKeeper time_keeper;
  std::unique_ptr<vlcal::PointCloudIntegrator> points_integrator;

  if (dynamic_point==true) {
    vlcal::DynamicPointCloudIntegratorParams params;
    params.visualize = visualize ;
    params.verbose = vm.count("verbose");
    params.k_neighbors = vm["k_neighbors"].as<int>();
  
    params.voxel_resolution = vm["voxel_resolution"].as<double>();
    params.min_distance = vm["min_distance"].as<double>();
    params.num_threads = num_threads;
    points_integrator.reset(new vlcal::DynamicPointCloudIntegrator(params));
  } else {
    vlcal::StaticPointCloudIntegratorParams params;
    params.visualize = visualize;
    params.voxel_resolution = vm["voxel_resolution"].as<double>();
    params.min_distance = vm["min_distance"].as<double>();
    points_integrator.reset(new vlcal::StaticPointCloudIntegrator(params));
  }
  // view_culling_params.enable_depth_buffer_culling = !params.disable_z_buffer_culling;
  // ViewCulling view_culling(proj, {dataset.front()->image.cols, dataset.front()->image.rows}, view_culling_params);
int point_num=0;
  auto points_reader = get_point_cloud_reader(bag_filename, points_topic, intensity_channel);
  while (true) {
    auto raw_points = points_reader->read_next();
    if (!raw_points) {
      break;
    }

    if (!time_keeper.process(raw_points)) {
      std::cerr << vlcal::console::yellow << "warning: skip frame with an invalid timestamp!!" << vlcal::console::reset << std::endl;
      continue;
    }
   //遍历点云数据，移除距离大于30m的点云

   for (int i = 0; i < raw_points->size(); i++) {
  //   if(points_integrator->flag_static!=0)
  //   {
  //    break;
  //   }
    const Eigen::Vector4d& p = raw_points->points[i];
  //   if (p.norm() > 60.0||p.norm()<1.5) {
  //     raw_points->points.erase(raw_points->points.begin() + i);
  //     raw_points->intensities.erase(raw_points->intensities.begin() + i);
  //     raw_points->times.erase(raw_points->times.begin() + i);
  //     i--;
  //   }
    if (p.norm()<30.0) {

     point_num++;
    }
    }

    std::cout << "point_num:" << point_num/14129 << std::endl;
    // 
    auto points = std::make_shared<FrameCPU>(raw_points->points);
    points->add_times(raw_points->times);
    points->add_intensities(raw_points->intensities);
    points = filter(points, [](const Eigen::Vector4d& p) { return p.array().isFinite().all(); });

    points_integrator->insert_points(points);
    if(360/hfov*point_num/(14129)>220)
    {
      break;
    }
    // auto points_now = points_integrator->get_points_now();
  }

  auto points = points_integrator->get_points();

  // histrogram equalization
   std::vector<int> indices(points->size());
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(), [&](const int lhs, const int rhs) { return points->intensities[lhs] < points->intensities[rhs]; });

  const int bins = 256;
  for (int i = 0; i < indices.size(); i++) {
    const double value = std::floor(bins * static_cast<double>(i) / indices.size()) / bins;
    points->intensities[indices[i]] = value;
  }
//   std::vector<int> indices(points->size());
//   std::iota(indices.begin(), indices.end(), 0);
//   std::sort(indices.begin(), indices.end(), [&](const int lhs, const int rhs) { return points->intensities[lhs] < points->intensities[rhs]; });
//  std::vector<double > intensities(points->size());
//  for (int i = 0; i < indices.size(); i++) {
    
//   intensities.push_back(points->intensities[i]) ;
//   }
//   globalHistogramEqualization(intensities);
//   const int bins =512;
//   for (int i = 0; i < indices.size(); i++) {
//     const double value = std::floor(bins * static_cast<double>(i) / indices.size()) / bins;
//     points->intensities[indices[i]] = value;
//       // std::cout<<value<<"sssssssssssssssss"<<intensities[i]<< std::endl;

//   }

  return {corrected, points,image_rgb};
}

}  // namespace vlcal

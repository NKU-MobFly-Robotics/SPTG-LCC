#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <boost/program_options.hpp>
#include <vlcal/common/frame.hpp>
#include <vlcal/common/raw_points.hpp>
#include <ros/ros.h>
namespace vlcal {

class PointCloudReader {
public:
  virtual ~PointCloudReader() {}
  virtual RawPoints::Ptr read_next() = 0;
};

class Preprocess {
public:
  Preprocess();
  ~Preprocess();

  bool run(int argc, char** argv, int status_flag, Eigen::Isometry3d init_L_C );
  bool Preprocess_init(int argc, char** argv, int status_flag, Eigen::Isometry3d init_L_C);
  void save_point_forNID( std::vector<std::string> bag_filenames,std::string dst_path);
  void view_process(std::vector<std::string> bag_filenames,std::string dst_path);
  cv::Mat  hole_process(cv::Mat cropped_intensities_eq,cv::Mat cropped_indices_eq,bool indices,bool depth, int radius_eq );

  ros::NodeHandle nh_;
  std::string data_path  ;
  std::string dst_path;
  std::string camera_intrinsics;
  std::string camera_distortion_coeffs;
  std::string camera_model;
  bool auto_topic;
  bool visualize;
  bool pinhole_if_pre;
  bool cull_hidden_points;
  bool initial_alignment;
   bool dynamic_point;
   int LiDAR_type;
     int hfov;
  Eigen::Isometry3d T_CL_init;
   Eigen::Isometry3d T_L0C0;
     // const auto topics = get_topics(vm, bag_filenames.front());
   std::string camera_info_topic;
       std::string image_topic;

   std::string  points_topic;
protected:
  virtual bool valid_bag(const std::string& bag_filename) = 0;
  virtual std::vector<std::pair<std::string, std::string>> get_topics_and_types(const std::string& bag_filename) = 0;
  virtual std::vector<std::string> get_point_fields(const std::string& bag_filename, const std::string& points_topic) = 0;
  virtual cv::Size get_image_size(const std::string& bag_filename, const std::string& image_topic) = 0;
  virtual std::tuple<std::string, std::vector<double>, std::vector<double>> get_camera_info(const std::string& bag_filename, const std::string& camera_info_topic) = 0;
  virtual std::pair<cv::Mat,cv::Mat> get_image(const std::string& bag_filename, const std::string& image_topic) = 0;
  virtual std::shared_ptr<PointCloudReader> get_point_cloud_reader(const std::string& bag_filename, const std::string& points_topic, const std::string& intensity_channel) = 0;

private:
  std::vector<Frame::ConstPtr> lidar_points;
  std::tuple<std::string, std::string, std::string> get_topics(const boost::program_options::variables_map& vm, const std::string& bag_filename);
  std::string get_intensity_channel(const boost::program_options::variables_map& vm, const std::string& bag_filename, const std::string& points_topic);
  std::tuple<std::string, cv::Size, std::vector<double>, std::vector<double>>
  get_camera_params(const boost::program_options::variables_map& vm, const std::string& bag_filename, const std::string& camera_info_topic, const std::string& image_topic);
 std::tuple<std::string, cv::Size, std::vector<double>, std::vector<double>>
 get_manual_camera_params(
  const boost::program_options::variables_map& vm,
  const std::string& bag_filename,
  const std::string& camera_info_topic,
  const std::string& image_topic) ;
 std::tuple<cv::Mat, Frame::ConstPtr,cv::Mat> get_image_and_points(
    
    const boost::program_options::variables_map& vm,
    const std::string& bag_filename,
    const std::string& image_topic,
    const std::string& points_topic,
    const std::string& intensity_channel,
    const int num_threads);
};

}  // namespace vlcal

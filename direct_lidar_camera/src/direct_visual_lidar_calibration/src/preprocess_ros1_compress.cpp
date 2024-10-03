#include <vlcal/preprocess/preprocess.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp> 
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include "sensor_msgs/image_encodings.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#define GLIM_ROS2
#include <vlcal/common/ros_cloud_converter.hpp>
#include "direct_visual_lidar_calibration/init_LC.h" 

namespace vlcal {

class PointCloudReaderROS1 : public PointCloudReader {
public:
  PointCloudReaderROS1(const std::string& bag_filename, const std::string& points_topic, const std::string& intensity_channel) : intensity_channel(intensity_channel) {
    reader.open(bag_filename);
    view = std::make_shared<rosbag::View>(reader, rosbag::TopicQuery(points_topic));

    msg_itr = view->begin();
  }

  virtual RawPoints::Ptr read_next() override {
    if (msg_itr == view->end()) {
      return nullptr;
    }

    const auto points_msg = msg_itr->instantiate<sensor_msgs::PointCloud2>();
    if (!points_msg) {
      return nullptr;
    }

    msg_itr++;
    return extract_raw_points(points_msg, intensity_channel);
  }

private:
  const std::string intensity_channel;

  rosbag::Bag reader;
  std::shared_ptr<rosbag::View> view;

  rosbag::View::iterator msg_itr;
};

class PreprocessROS1 : public Preprocess {
protected:
  template <typename T>
  boost::shared_ptr<T> get_first_message(const std::string& bag_filename, const std::string& topic) const {
    rosbag::Bag bag(bag_filename);
    rosbag::View view(bag, rosbag::TopicQuery(topic));

    for (const auto m : view) {
      const auto msg = m.instantiate<T>();
      if (msg) {
        return msg;
      }
    }

    std::cerr << "error: bag does not contain topic" << std::endl;
    std::cerr << "     : bag_filename=" << bag_filename << std::endl;
    std::cerr << "     : topic=" << topic << std::endl;
    abort();
    return nullptr;
  }

  virtual bool valid_bag(const std::string& bag_filename) override {
    rosbag::Bag bag(bag_filename);
    return bag.isOpen();
  }

  virtual std::vector<std::pair<std::string, std::string>> get_topics_and_types(const std::string& bag_filename) override {
    rosbag::Bag bag(bag_filename);
    rosbag::View view(bag);

    std::vector<std::pair<std::string, std::string>> topics_and_types;
    for (const auto& conn : view.getConnections()) {
      topics_and_types.emplace_back(conn->topic, conn->datatype);
    }

    return topics_and_types;
  }

  virtual std::vector<std::string> get_point_fields(const std::string& bag_filename, const std::string& points_topic) override {
    const auto msg = get_first_message<sensor_msgs::PointCloud2>(bag_filename, points_topic);
    std::vector<std::string> fields(msg->fields.size());
    std::transform(msg->fields.begin(), msg->fields.end(), fields.begin(), [](const auto& field) { return field.name; });
    return fields;
  }

  virtual cv::Size get_image_size(const std::string& bag_filename, const std::string& image_topic) override {
// 尝试获取，失败的话获取sensor_msgs：CompressedImage类型
   
      const auto image_msg = get_first_message<sensor_msgs::CompressedImage>(bag_filename, image_topic);
      
      // return cv::Size(image_msg->width, image_msg->height);
      cv::Mat image = cv::imdecode(cv::Mat(image_msg->data), cv::IMREAD_COLOR);
      // return image.size();
       return cv::Size(image.cols, image.rows);
     
    // const auto image_msg = get_first_message<sensor_msgs::Image>(bag_filename, image_topic);
    // return cv::Size(image_msg->width, image_msg->height);
  }

  virtual std::tuple<std::string, std::vector<double>, std::vector<double>> get_camera_info(const std::string& bag_filename, const std::string& camera_info_topic) override {
    const auto camera_info_msg = get_first_message<sensor_msgs::CameraInfo>(bag_filename, camera_info_topic);
    std::vector<double> intrinsic(4);
    intrinsic[0] = camera_info_msg->K[0];
    intrinsic[1] = camera_info_msg->K[4];
    intrinsic[2] = camera_info_msg->K[2];
    intrinsic[3] = camera_info_msg->K[5];
    std::vector<double> distortion_coeffs = camera_info_msg->D;
//    intrinsic.insert(intrinsic.end(), {1043.9777149158745,1044.3901630718665,620.3542209837874,343.75690903851563});
  
//     // intrinsic[0] = camera_info_msg->k[0];
//     // intrinsic[1] = camera_info_msg->k[4];
//     // intrinsic[2] = camera_info_msg->k[2];
//     // intrinsic[3] = camera_info_msg->k[5];
//     // std::vector<double> distortion_coeffs = camera_info_msg->d;
//     std::vector<double> distortion_coeffs;
// distortion_coeffs.insert(distortion_coeffs.end(), {-0.4558224833720169, 0.29944271675699663, 6.867280678643097e-5,
//       -0.00012528851559686296, -0.139114502340535});
    return {camera_info_msg->distortion_model, intrinsic, distortion_coeffs};
  }

  virtual std::pair<cv::Mat,cv::Mat> get_image(const std::string& bag_filename, const std::string& image_topic) override {

 
   
    const auto image_msg = get_first_message<sensor_msgs::CompressedImage>(bag_filename, image_topic);
    return { cv_bridge::toCvCopy(*image_msg,sensor_msgs::image_encodings::MONO8)->image,cv_bridge::toCvCopy(*image_msg,sensor_msgs::image_encodings::BGR8)->image};
   
 
      //保存为彩色图像
    
  }

  virtual std::shared_ptr<PointCloudReader> get_point_cloud_reader(const std::string& bag_filename, const std::string& points_topic, const std::string& intensity_channel)
    override {
    return std::make_shared<PointCloudReaderROS1>(bag_filename, points_topic, intensity_channel);
  }
};

}  // namespace vlcal

 int status_flag=0;
int  preprecess_LC=0;
// void init_T_LiDAR_Camera_Callback(const nav_msgs::Odometry::ConstPtr& msg,int argc, char** argv) {
//     // 输出接收到的里程计信息
//    ros::NodeHandle nh;  
//  ros::Publisher pub_preprecess_LC = nh.advertise<std_msgs::Int32>("/preprecess_LC",1);
//   std_msgs::Int32 msg_preprecess_LC;
//   ROS_INFO("Received odometry message: Position(x=%.2f, y=%.2f, z=%.2f), Orientation(x=%.2f, y=%.2f, z=%.2f, w=%.2f)",
//              msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
//              msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
 
//   Eigen::Isometry3d init_T_camera_lidar;
//   //msg赋值给//msg赋值给init_T_came
//   init_T_camera_lidar.translation() 
//         << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
//   init_T_camera_lidar.linear() =
//   Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();
 
  
//    if(msg->pose.pose.orientation.x!=0)
// {    msg_preprecess_LC.data=0;
//   pub_preprecess_LC.publish(msg_preprecess_LC);

//    preprocess.run(argc, argv,status_flag,init_T_camera_lidar.inverse());
//    msg_preprecess_LC.data=1;
//   pub_preprecess_LC.publish(msg_preprecess_LC);
// status_flag=status_flag+1;}
 
  
  
//   //定义一个发布者

// }


int main(int argc, char** argv) {
  ros::init(argc, argv, "my_node");
  vlcal::PreprocessROS1 preprocess;  
  // ros::NodeHandle nh;
    // nh.param<string>("preprocess/data_path",path_en, true);
    // nh.param<string>("preprocess/dst_path",scan_pub_en, true);
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



 
 ros::Publisher pub_preprecess_LC =  preprocess.nh_.advertise<std_msgs::Int32>("/preprecess_LC",1);
  std_msgs::Int32 msg_preprecess_LC;
 
    
    // 创建ROS节点句柄
    Eigen::Isometry3d init_T_camera_lidar;
   init_T_camera_lidar.translation() << 0.0, 0.0, 0.0;
    init_T_camera_lidar.linear() = Eigen::Quaterniond(0.508862,
    0.5018527, -0.4926349, 0.4965018  ).normalized().toRotationMatrix();
 
    preprocess.Preprocess_init(argc, argv,status_flag,init_T_camera_lidar);
    preprocess.run(argc, argv,status_flag,init_T_camera_lidar);
  
   ros::ServiceClient client = preprocess.nh_.serviceClient<direct_visual_lidar_calibration::init_LC>("init_LC");

    // Create a request
    direct_visual_lidar_calibration::init_LC srv;
     if(preprocess.pinhole_if_pre==true)
    {
      srv.request.request_data=1;
    }
    else
    {
      srv.request.request_data=0;
    }
   
   while(ros::ok()){
    // Call the service
    if (client.call(srv))
    {
      status_flag++;
       
  //msg赋值给//msg赋值给init_T_came
     init_T_camera_lidar.translation() 
        <<  srv.response.response_data.pose.pose.position.x, 
         srv.response.response_data.pose.pose.position.y,
          srv.response.response_data.pose.pose.position.z;
     init_T_camera_lidar.linear() =
      Eigen::Quaterniond(srv.response.response_data.pose.pose.orientation.w,
      srv.response.response_data.pose.pose.orientation.x, 
      srv.response.response_data.pose.pose.orientation.y,
      srv.response.response_data.pose.pose.orientation.z).toRotationMatrix();
       
      // preprocess.run(argc, argv,status_flag,init_T_camera_lidar.inverse());
      if(status_flag==4)
      {
           preprocess.run(argc, argv,status_flag,init_T_camera_lidar.inverse());  
           if(preprocess.initial_alignment==true)
      {
         Eigen::Isometry3d T_camera_lidar_fina;
         T_camera_lidar_fina=init_T_camera_lidar.inverse()*preprocess.T_L0C0*preprocess.T_CL_init;
         std::cout<<"T_lidar_camera_fina:"<<T_camera_lidar_fina.matrix().inverse()<<std::endl;
       }
      else{
         std::cout<<"T_lidar_camera_fina:"<<init_T_camera_lidar.matrix().inverse()<<std::endl;
      }
            ros::shutdown();
   
      }
      else{
         preprocess.run(argc, argv,status_flag,init_T_camera_lidar.inverse());
      }
      std::cout<<"status_flagpreprocess.pinhole_if:"<<preprocess.pinhole_if_pre<<std::endl;  
        if(preprocess.pinhole_if_pre==true)
    {
      srv.request.request_data=1+status_flag*10;
    }
    else
    {
      srv.request.request_data=0+status_flag*10;
    }
   

    }
    else
    {
        ROS_ERROR("Failed to call service init_LC");
       
    }
   }
  //  preprecess_LC=1;
  //  msg_preprecess_LC.data=preprecess_LC;
  // pub_preprecess_LC.publish(msg_preprecess_LC);

  // 使用 boost::bind 绑定回调函数和额外的参数
 

  // ros::Subscriber sub = nh.subscribe< nav_msgs::Odometry>
  // ("/init_T_LiDAR_Camera", 1,boost::bind(&init_T_LiDAR_Camera_Callback,_1,argc,argv));
  

   
 
   
 
  return 0;
}

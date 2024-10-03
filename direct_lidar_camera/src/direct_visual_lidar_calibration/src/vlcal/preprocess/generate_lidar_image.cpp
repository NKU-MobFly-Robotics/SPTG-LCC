#include <vlcal/preprocess/generate_lidar_image.hpp>

#include <vlcal/common/estimate_fov.hpp>
#include<opencv2/opencv.hpp>  
namespace vlcal {

std::tuple<cv::Mat, cv::Mat, cv::Mat>
generate_lidar_image(const camera::GenericCameraBase::ConstPtr& proj, const Eigen::Vector2i& image_size, const Eigen::Isometry3d& T_camera_lidar, const Frame::ConstPtr& points) {
  //
  const double camera_fov = estimate_camera_fov(proj, image_size);
  const double min_z = std::cos(camera_fov);
std::vector<double> depths;
  cv::Mat sq_dist_image(image_size[1], image_size[0], CV_64FC1, cv::Scalar::all(std::numeric_limits<double>::max()));
  cv::Mat intensity_image(image_size[1], image_size[0], CV_64FC1, cv::Scalar::all(0));
  cv::Mat index_image(image_size[1], image_size[0], CV_32SC1, cv::Scalar::all(-1));
    cv::Mat depth_image(image_size[1], image_size[0], CV_64FC1, cv::Scalar::all(0));
// 1. 创建直方图
// int max_depth = 50; // 假设最大深度为20米
// std::vector<int> counts(max_depth, 0);

// // 2. 遍历所有的点，将其分配到相应的深度区间
// for (int i = 0; i < points->size(); i++) {
//     double depth = points->points[i].head<3>().norm(); // 计算点的深度
//     int depth_interval = static_cast<int>(depth);
//     if (depth_interval < max_depth) {
//         counts[depth_interval]++;
//     }
// }
// std::vector<std::vector<int>> table(max_depth, std::vector<int>(2));
//  for (int i = 0; i < max_depth; i++) {
//     table[i][0] = i; // 深度值
//     table[i][1] = counts[i]; // 点的数量
// }
// cv::Mat table_image(max_depth * 20 + 50, 200, CV_8UC3, cv::Scalar(255, 255, 255));

// // 将表格数据添加到图像
// for (int i = 0; i < max_depth; i++) {
//     cv::putText(table_image, "Depth: " + std::to_string(table[i][0]), cv::Point(10, i * 20 + 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
//     cv::putText(table_image, "Count: " + std::to_string(table[i][1]), cv::Point(100, i * 20 + 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
// }

// // 保存图像
 
// // 保存图像
// cv::imwrite("/calib_data/direct_lidar_camera/table.png", table_image);

// // 计算直方图的最大值
// int max_count = *std::max_element(counts.begin(), counts.end());

// // 创建一个图像，用于绘制直方图
// cv::Mat hist_image(300, max_depth * 10 + 50, CV_8UC3, cv::Scalar(255, 255, 255));

// // 创建一个颜色数组
// std::vector<cv::Scalar> colors = {cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255)};

// int flag=1;
// // 对于每个bin，绘制一个矩形
// for (int i = 0; i < max_depth; i++) {
//     double height = static_cast<double>((static_cast<double>(counts[i]) / max_count) * 250);
//     if(height<1&&height>0){
//         // std::cout<<counts[i]<<" "<<height<<std::endl;
//         cv::rectangle(hist_image, cv::Point(i * 10 + 20, 250), cv::Point((i + 1) * 10 - 1 + 20, 250 - height ), colors[0 % colors.size()], -1);
//         cv::putText(hist_image, std::to_string(counts[i]), cv::Point(i * 10 + 20, 250 - height - flag*5), cv::FONT_HERSHEY_SIMPLEX, 0.5, colors[i % colors.size()]);
//     flag++;
//     }
//     else
//     {
//         // std::cout<<counts[i]<<" "<<height<<std::endl;
//         cv::rectangle(hist_image, cv::Point(i * 10 + 20, 250), cv::Point((i + 1) * 10 - 1 + 20, 250 - height), colors[i % colors.size()], -1);
//     }
// }
// // 绘制坐标轴
// cv::line(hist_image, cv::Point(20, 0), cv::Point(20, 250), cv::Scalar(0, 0, 0), 2);
// cv::line(hist_image, cv::Point(20, 250), cv::Point(max_depth * 10 + 20, 250), cv::Scalar(0, 0, 0), 2);

// // 添加坐标轴标签
// for (int i = 0; i <= max_depth; i += 10) {
//     cv::putText(hist_image, std::to_string(i), cv::Point(i * 10 + 15, 270), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
// }
// // for (int i = 0; i <= max_count; i += max_count / 5) {
// //     double y = 250.0 - static_cast<double>((static_cast<double>(i) / max_count) * 250);
// //     cv::putText(hist_image, std::to_string(i), cv::Point(5, y + 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
// // }

// // 保存图像
 
// // 3. 找到深度突变的区间
// std::vector<int> outlier_intervals;
// for (int i = 11; i < max_depth - 1; i++) {
//     if (counts[i] < counts[i - 1] / 5 || counts[i] < counts[i + 1] / 5||counts[i] < 500) {
//         outlier_intervals.push_back(i);
//     }
// }

// // 4. 保存图像
// cv::imwrite("/calib_data/direct_lidar_camera/histogram.png", hist_image);
// // 4. 记录深度突变区间的点的序号
// std::vector<int> outlier_indices;
// for (int outlier_interval : outlier_intervals) {
//     for (int i = 0; i < points->size(); i++) {
//         double depth = points->points[i].head<3>().norm(); // 计算点的深度
//         int depth_interval = static_cast<int>(depth);
//         if (depth_interval == outlier_interval) {
//             outlier_indices.push_back(i);
//         }
//     }
// }
  for (int i = 0; i < points->size(); i++) {
    const auto& pt_lidar = points->points[i];
    const Eigen::Vector4d pt_camera = T_camera_lidar * pt_lidar;
// auto it = std::find(outlier_indices.begin(), outlier_indices.end(), i);
// if (it != outlier_indices.end())
// {
//     continue;
 
// }
    if (pt_camera.head<3>().normalized().z() < min_z) {
      continue;
    }

    const Eigen::Vector2i pt_2d = proj->project(pt_camera.head<3>()).cast<int>();
    if ((pt_2d.array() < Eigen::Array2i::Zero()).any() || (pt_2d.array() >= image_size.array()).any()) {
      continue;
    }

    const double sq_dist = pt_camera.head<3>().norm();
    if (sq_dist_image.at<double>(pt_2d.y(), pt_2d.x()) < sq_dist) {
      continue;
    }
    
     
  //  std::cout<<sq_dist<<" ssssssssssssss"<<pt_2d.x()<<std::endl;
    sq_dist_image.at<double>(pt_2d.y(), pt_2d.x()) = sq_dist;
   

    intensity_image.at<double>(pt_2d.y(), pt_2d.x()) =  points->intensities[i];
    index_image.at<std::int32_t>(pt_2d.y(), pt_2d.x()) = i; 
    //  if(sq_dist>30)
    //  {
    //   continue;
    //  }
    depth_image.at<double>(pt_2d.y(), pt_2d.x()) = sq_dist;
  }
  cv::normalize(depth_image, depth_image, 0, 1, cv::NORM_MINMAX, CV_64FC1);
  return std::make_tuple(intensity_image,depth_image ,index_image);
}

}  // namespace vlcal

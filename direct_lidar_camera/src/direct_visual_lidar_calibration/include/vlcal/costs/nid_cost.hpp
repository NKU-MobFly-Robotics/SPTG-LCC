#pragma once

#include <algorithm>
#include <sophus/se3.hpp>
#include <opencv2/core.hpp>
#include <vlcal/common/frame.hpp>
#include <camera/generic_camera_base.hpp>
#include <cmath>
#include <ceres/ceres.h>
namespace vlcal {

template <typename T>
double get_real(const T& x) {
  return x.a;
}

template <>
double get_real(const double& x) {
  return x;
}

class NIDCost {
public:
  NIDCost(const camera::GenericCameraBase::ConstPtr& proj, const cv::Mat& normalized_image, const Frame::ConstPtr& points, const int bins = 16, bool rgb_depth=true)
  : proj(proj),
    normalized_image(normalized_image.clone()),
    points(points),
    bins(bins), 
    rgb_depth(rgb_depth){
      std::cout<<rgb_depth<<"---------------------------"<<std::endl;
    //三次B样条插值
    // spline_coeffs.row(0) << 1.0, -3.0, 3.0, -1.0;
    // spline_coeffs.row(1) << 4.0, 0.0, -6.0, 3.0;
    // spline_coeffs.row(2) << 1.0, 3.0, 3.0, -3.0;
    // spline_coeffs.row(3) << 0.0, 0.0, 0.0, 1.0;
    // spline_coeffs /= 6.0;
    //曲线插值一般指的是给定插值点，得出曲线的方程，曲线会经过所有的插值点。确定三次B样条曲线的输入量有两种，一种是给出控制点和其它边界条件，曲线一般不经过控制点；一种是给出插值点和其它边界条件，曲线会经过所有插值点，显然第二种输入量使用更为广泛
    //二次B样条插值
    // spline_coeffs.row(0) << 1.0, -2.0, 1.0 ;
    // spline_coeffs.row(1) << 1.0, 2.0, -2.0 ;
    // spline_coeffs.row(2) << 0.0, 0.0, 1.0 ;
    //    spline_coeffs /= 2.0;
    //四次B样条插值
    spline_coeffs.row(0) << 1.0, -4.0, 6.0, -4.0, 1.0 ;
    spline_coeffs.row(1) << 11.0, -12.0, -6.0, 12.0, -4.0 ;
    spline_coeffs.row(2) << 11.0, 12.0, -6, -12.0, 6.0 ;
    spline_coeffs.row(3) << 1.0, 4.0, 6.0, 4.0, -4.0 ;
    spline_coeffs.row(4) << 0, 0, 0, 0, 1.0 ;
    spline_coeffs /= 24.0;
  }

  template <typename T>
  bool operator()(const T* T_camera_lidar_params, T* residual) const {
    const Eigen::Map<Sophus::SE3<T> const> T_camera_lidar(T_camera_lidar_params);

    Eigen::Matrix<T, -1, -1> hist = Eigen::Matrix<T, -1, -1>::Zero(bins, bins);

    Eigen::Matrix<T, -1, 1> hist_image = Eigen::Matrix<T, -1, 1>::Zero(bins);
    Eigen::VectorXd hist_points = Eigen::VectorXd::Zero(bins);
     std::vector<double> depth_nid;
 for (int i = 0; i < points->size(); i++) {
//计算所有点的深度并归一化
    double depth =points-> points[i].head<3>().norm(); // 假设点的前三个分量表示坐标
    depth_nid.push_back(depth);
 }
 //depth_nid归一化
  double max_depth=*std::max_element(depth_nid.begin(),depth_nid.end());
  double min_depth=*std::min_element(depth_nid.begin(),depth_nid.end());
  for (int i = 0; i < depth_nid.size(); i++) {
    depth_nid[i] = (depth_nid[i] - min_depth) / (max_depth - min_depth);
}
    int num_outliers = 0;
    for (int i = 0; i < points->size(); i++) {
      const Eigen::Matrix<T, 3, 1> pt_camera = T_camera_lidar * points->points[i].head<3>();
     
       double intensity = points->intensities[i];
        if(rgb_depth==false)
    { 
      //std::cout<<depth_nid[i]<<" "<<lidar_intensity<<std::endl;
      intensity=depth_nid[i];
    }
    double depth =points-> points[i].head<3>().norm(); // 假设点的前三个分量表示坐标
    double w_depth;
    if (depth<10)
    {
      w_depth=0.6+0.4*(1-depth/10);
    } 
    else
    {
      w_depth=6/depth;
    }
      // 通过将强度值映射到直方图上，确定了该点应该在直方图的哪个区间增加计数。
      const int bin_points = std::max<int>(0, std::min<int>(bins - 1, intensity * bins));
// 通过相机投影函数将点云投影到归一化图像上。
      const Eigen::Matrix<T, 2, 1> projected = (*proj)(pt_camera);
    // 根据投影结果计算点在归一化图像上的离散坐标。
    //向下取整 3.8 得到 3。向下取整 7.2 得到 7。向下取整 -4.6 得到 -5。
      const Eigen::Vector2i knot_i(std::floor(get_real(projected[0])), std::floor(get_real(projected[1])));
    //  计算投影点与其离散坐标之间的差距。
      const Eigen::Matrix<T, 2, 1> s = projected - knot_i.cast<double>();
// 如果点在归一化图像外部，则将其视为离群值，增加 num_outliers 计数，并继续下一个点的处理。
      if ((knot_i.array() < Eigen::Array2i(0, 0)).any() || (knot_i.array() >= Eigen::Array2i(normalized_image.cols, normalized_image.rows)).any()) {
        num_outliers++;
        continue;
      }

      hist_points[bin_points]= hist_points[bin_points]+1;

      Eigen::Matrix<T, 5, 2> se;
      se.row(0).setOnes();
      se.row(1) = s.transpose();
      se.row(2) = s.array().square().transpose();
      se.row(3) = (s.array().square() * s.array()).transpose();
      se.row(4) = (s.array()*s.array().square() * s.array()).transpose();

    //  const Eigen::Matrix<T, 3, 2> beta = spline_coeffs * se;

    //   Eigen::Array<int,3,1> knots_x;
    //   knots_x<< knot_i.x() - 1, knot_i.x(), knot_i.x() + 1;
    //   Eigen::Array<int,3,1> knots_y;
    //   knots_y<< knot_i.y() - 1, knot_i.y(), knot_i.y() + 1;

      const Eigen::Matrix<T, 5, 2> beta = spline_coeffs * se;

      Eigen::Array<int,5,1> knots_x;
      knots_x<<knot_i.x() - 2,knot_i.x() - 1, knot_i.x(), knot_i.x() + 1, knot_i.x() + 2;
      Eigen::Array<int,5,1> knots_y;
      knots_y<<knot_i.y() - 2,knot_i.y() - 1, knot_i.y(), knot_i.y() + 1, knot_i.y() + 2;
      knots_x = knots_x.max(0).min(normalized_image.cols - 1);
      knots_y = knots_y.max(0).min(normalized_image.rows - 1);
    //  const double pix = normalized_image.at<double>(cv::Point(knot_i.x(), knot_i.y()));

    // // // 将像素值映射到直方图 bin 中并更新直方图
    // const int bin_image = std::min<int>(pix * bins, bins - 1);
    // hist(bin_image, bin_points)+=1;
    // hist_image[bin_image]+=1;
      for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
          const T w = beta(i, 0) * beta(j, 1);
          const double pix = normalized_image.at<double>(knots_y[j], knots_x[i]);
          const int bin_image = std::min<int>(pix * bins, bins - 1);
          hist(bin_image, bin_points) += w ;
          hist_image[bin_image] += w;
        }
      }
    }

    const double sum = hist_points.sum();

    hist_image = hist_image / sum;
    hist_points = hist_points / sum;
    hist = hist / sum;

    const T H_image = -(hist_image.array() * (hist_image.array() + 1e-6).log()).sum();
    const double H_points = -(hist_points.array() * (hist_points.array() + 1e-6).log()).sum();
    const T H_image_points = -(hist.array() * (hist.array() + 1e-6).log()).sum();
    const T MI = H_image + H_points - H_image_points;
 
    T NID = (H_image_points - MI) / H_image_points;
//  const T NID = 1.0 - get_real(MI) / (std::max(get_real(H_image), H_points));

   const T entropy_image = -(hist_image.array() * (hist_image.array() + 1e-6).log()).sum();
const double entropy_points = -(hist_points.array() * (hist_points.array() + 1e-6).log()).sum();
const T NMI = (entropy_image + entropy_points)/H_image_points;

     
    if (!std::isfinite(get_real(NID))) {
      std::cout << get_real(H_image_points) << " " << get_real(MI) << " " << get_real(NID) << std::endl;

      return false;
    }
   if(rgb_depth==false)
    { 
      //std::cout<<depth_nid[i]<<" "<<lidar_intensity<<std::endl;
      NID=0.1*NID;
    }
    std::cout<<get_real(NID)<<std::endl;
    residual[0] = NID ;  // 1.0-NMI ; //+ 0.5*(-NMI);
    return true;
  }  
    //  const T  NMI = 2.0 * MI / (H_image + H_points);
 // const T NMI = H_image_points / ceres::sqrt(H_image * H_points);
private:
  const camera::GenericCameraBase::ConstPtr proj;
  const cv::Mat normalized_image;
  const Frame::ConstPtr points;
  bool rgb_depth;
  const int bins;
  Eigen::Matrix<double,5, 5> spline_coeffs;
};
}
#include <vlcal/calib/cost_calculator_nid.hpp>

#include <vlcal/common/estimate_fov.hpp>

namespace vlcal {

NIDCostParams::NIDCostParams() {
  bins = 16;
}

NIDCostParams::~NIDCostParams() {}

CostCalculatorNID::CostCalculatorNID(const camera::GenericCameraBase::ConstPtr& proj, const VisualLiDARData::ConstPtr& data, const NIDCostParams& params)
: params(params),
  proj(proj),
  data(data),
  max_fov(estimate_camera_fov(proj, {data->image.cols, data->image.rows})) {}

CostCalculatorNID::~CostCalculatorNID() {}

double CostCalculatorNID::calculate(const Eigen::Isometry3d& T_camera_lidar,bool rgb_depth) {
  const auto& image = data->image;
  const auto& points = data->points;
  const Eigen::Array2i image_size(image.cols, image.rows);
// params.bins 是一个参数，用于指定直方图的 bin 数量。直方图是一种统计工具，用于对数据进行频率统计，特别是对数据的分布情况进行可视化。在这里，直方图用于分析图像像素的强度分布。
// 对于图像像素的强度范围，通常是指像素的灰度值范围。在灰度图像中，每个像素的值代表了其灰度级别，通常在0到255之间，0表示黑色，255表示白色。直方图的 bin 数量决定了对这个范围的划分程度，即将这个范围划分为多少个小区间。
  Eigen::MatrixXd hist = Eigen::MatrixXd::Zero(params.bins, params.bins);
  Eigen::VectorXd hist_image = Eigen::VectorXd::Zero(params.bins);
  Eigen::VectorXd hist_points = Eigen::VectorXd::Zero(params.bins);
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
  for (int i = 0; i < points->size(); i++) {
    // 将点云中的每个点通过相机到激光雷达的坐标变换 T_camera_lidar 转换到相机坐标系中。
    const Eigen::Vector4d pt_camera = T_camera_lidar * points->points[i];
     double depth =points-> points[i].head<3>().norm(); // 假设点的前三个分量表示坐标
    double w_depth;
    if (depth<=15)
    {
      w_depth=1;
    } 
    else
    {
      w_depth=15/depth;
    }
    // 检查经过变换后的点是否在相机的视场之内。如果不在视场内，则跳过这个点，继续处理下一个点。
    if (pt_camera.head<3>().normalized().z() < std::cos(max_fov)) {
      // Out of FoV
      //示点在相机坐标系中的归一化z坐标（深度方向）。std::cos(max_fov) 表示最大视场角的余弦值。通过比较归一化的z坐标与最大视场角的余弦值，可以确定点是否在相机视场内。如果归一化的z坐标小于最大视场角的余弦值，说明点在相机视场内；否则，点就在视场外。
      continue;
    }

    const Eigen::Array2i pt_2d = proj->project(pt_camera.head<3>()).cast<int>();
    if ((pt_2d < Eigen::Array2i::Zero()).any() || (pt_2d >= image_size).any()) {
      // Out of Image
      continue;
    }

    const double pixel = image.at<std::uint8_t>(pt_2d.y(), pt_2d.x()) / 255.0;

    double lidar_intensity = points->intensities[i];
    if(rgb_depth==false)
    { 
      //std::cout<<depth_nid[i]<<" "<<lidar_intensity<<std::endl;
      lidar_intensity=depth_nid[i];
    }
    const int image_bin = std::max<int>(0, std::min<int>(params.bins - 1, pixel * params.bins));
    const int lidar_bin = std::max<int>(0, std::min<int>(params.bins - 1, lidar_intensity * params.bins));

    hist(image_bin, lidar_bin)=hist(image_bin, lidar_bin)+1;
    hist_image[image_bin]=hist_image[image_bin]+1;
    hist_points[lidar_bin]=hist_points[lidar_bin]+1;
  }

  const double sum = hist_image.sum();

  const double sum_lidar = hist_points.sum();
  const Eigen::MatrixXd hist_rs = hist.cast<double>() / sum;
  const Eigen::VectorXd hist_r = hist_image.cast<double>() / sum;
  const Eigen::VectorXd hist_s = hist_points.cast<double>() / sum_lidar;

  const double Hr = -(hist_r.array() * (hist_r.array() + 1e-6).log()).sum();
  const double Hs = -(hist_s.array() * (hist_s.array() + 1e-6).log()).sum();
  const double Hrs = -(hist_rs.array() * (hist_rs.array() + 1e-6).log()).sum();
// std::cout<<"Hr-------------"<<Hr<<" "<<rgb_depth<<" "<<Hrs<<std::endl;
  const double MI = Hr + Hs - Hrs;

  double NID = (Hrs - MI) / Hrs;
  
  if(rgb_depth==true)
  {
    return NID;
  }
  else
  {
    return NID*0.2;
  } 
 }

}  // namespace vlcal
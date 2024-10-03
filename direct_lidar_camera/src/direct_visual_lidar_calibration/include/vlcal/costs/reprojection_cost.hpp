#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <camera/generic_camera_base.hpp>

namespace vlcal {

class ReprojectionCost {
public:
  ReprojectionCost(const camera::GenericCameraBase::ConstPtr& proj, const Eigen::Vector3d& point_3d, const Eigen::Vector2d& point_2d, double flag_depth_intens)
  : proj(proj),
    point_3d(point_3d),
    point_2d(point_2d),
    flag_depth_intens_weight(flag_depth_intens)
     {}

  ~ReprojectionCost() {}

  template <typename T>
  bool operator()(const T* const T_camera_lidar_params, T* residual) const {
    const Eigen::Map<Sophus::SE3<T> const> T_camera_lidar(T_camera_lidar_params);
    const Eigen::Matrix<T, 3, 1> pt_camera = T_camera_lidar * point_3d;
//计算point_3d的深度作为残差权重
double depth =point_3d.norm(); // 假设点的前三个分量表示坐标
double w_depth;
if (depth<=15)
{
  w_depth=1;
}
else if (depth<30)
{
  w_depth=15/depth;
}
else
{
  w_depth=0.3;
}
// w_depth=1;
    const auto pt_2d = (*proj)(pt_camera);

    residual[0] = flag_depth_intens_weight*(pt_2d[0] - point_2d[0]) ;
    residual[1] = flag_depth_intens_weight*( pt_2d[1] - point_2d[1]);
    // std::cout<<"flag_depth_intens_weight:"<<flag_depth_intens_weight<<std::endl;
    // std::cout<<"residual[0]:"<<residual[0]<<std::endl;
    // std::cout<<"residual[1]:"<<residual[1]<<std::endl;
    // std::cout<<"residual[1]:"<<residual[1]<<std::endl;
    // residual[0] = (pt_2d[0] - point_2d[0]) ;
    // residual[1] = ( pt_2d[1] - point_2d[1]);
    return true;
  }

private:
  const camera::GenericCameraBase::ConstPtr proj;
  const Eigen::Vector3d point_3d;
  const Eigen::Vector2d point_2d;
  double flag_depth_intens_weight;
};

}  // namespace vlcal

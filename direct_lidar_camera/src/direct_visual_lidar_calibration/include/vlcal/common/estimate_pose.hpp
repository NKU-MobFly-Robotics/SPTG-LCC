#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <camera/generic_camera_base.hpp>

namespace vlcal {

struct PoseEstimationParams {
  PoseEstimationParams() {
    ransac_iterations = 8192;
    ransac_error_thresh = 5.0;
    robust_kernel_width = 10.0;
  }

  int ransac_iterations;      ///< RANSAC iterations
  double ransac_error_thresh; ///< RANSAC inlier threshold [pix]
  double robust_kernel_width; ///< Robust kernel width for reprojection error minimization
};

/**
 * @brief Pose estimation based on 2D-3D correspondences
 */
class PoseEstimation {
public:
  PoseEstimation(const PoseEstimationParams& params = PoseEstimationParams());
  ~PoseEstimation();

  /**
   * @brief Estimate the LiDAR pose relative to the camera (T_camera_lidar)
   * @param proj             Camera model
   * @param correspondences  2D-3D correspondences
   * @param inliers          [optional] Flags to represent inliers of RANSAC
   * @param flag_depth_intens  2D-3D correspondences
   * @return T_camera_lidar
   */
  Eigen::Isometry3d
  estimate(const camera::GenericCameraBase::ConstPtr& proj, 
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>& correspondences,
   std::vector<bool>* inliers = nullptr,std::vector<int> flag_depth_intens=std::vector<int>(),bool pinhole_if=false);

private:
  Eigen::Matrix3d estimate_rotation_ransac(
    const camera::GenericCameraBase::ConstPtr& proj,
    const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>& correspondences,
    std::vector<bool>* inliers);
Eigen::Matrix3d LO_ransac(
  const camera::GenericCameraBase::ConstPtr& proj,
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>& correspondences,
  std::vector<bool>* inliers);
  Eigen::Isometry3d estimate_pose_lsq(
    const camera::GenericCameraBase::ConstPtr& proj,
    const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>& correspondences,
    const Eigen::Isometry3d& T_camera_lidar,
    std::vector<int> flag_depth_intens);

private:
  const PoseEstimationParams params;
};

}  // namespace vlcal

#include <vlcal/common/estimate_pose.hpp>

#include <random>
#include <fstream>
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <sophus/se3.hpp>
#include <sophus/ceres_manifold.hpp>
#include <vlcal/common/estimate_fov.hpp>
#include <vlcal/costs/reprojection_cost.hpp>

namespace vlcal {

PoseEstimation::PoseEstimation(const PoseEstimationParams& params) {}

PoseEstimation::~PoseEstimation() {}

Eigen::Isometry3d PoseEstimation::estimate(
  const camera::GenericCameraBase::ConstPtr& proj,
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>& correspondences,
  std::vector<bool>* inliers,std::vector<int> flag_depth_intens,bool pinhole_if) {
  // RANSAC
  Eigen::Isometry3d T_camera_lidar = Eigen::Isometry3d::Identity();
  T_camera_lidar.linear() = estimate_rotation_ransac(proj, correspondences, inliers);
  
  std::cout << "--- T_camera_lidar (Ransac) ---" << std::endl;
  std::cout << T_camera_lidar.matrix() << std::endl;

//  T_camera_lidar.linear() = LO_ransac(proj, correspondences, inliers);

//   std::cout << "--- T_camera_lidar (LO_ransac) ---" << std::endl;
//   std::cout << T_camera_lidar.matrix() << std::endl;



  // Reprojection error minimization
  T_camera_lidar = estimate_pose_lsq(proj, correspondences, T_camera_lidar,flag_depth_intens);

  std::cout << "--- T_camera_lidar (LSQ) ---" << std::endl;
  std::cout << T_camera_lidar.matrix() << std::endl;

  return T_camera_lidar;
}
Eigen::Matrix3d PoseEstimation::LO_ransac(
  const camera::GenericCameraBase::ConstPtr& proj,
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>& correspondences,
  std::vector<bool>* inliers) {
  std::cout << "estimating bearing vectors" << std::endl;
  // Compute bearing vectors
  std::vector<Eigen::Vector4d> directions_camera(correspondences.size());
  std::vector<Eigen::Vector4d> directions_lidar(correspondences.size());
  for (int i = 0; i < correspondences.size(); i++) {
    directions_camera[i] << estimate_direction(proj, correspondences[i].first), 0.0;
    directions_lidar[i] << correspondences[i].second.head<3>().normalized(), 0.0;
  }

  // LSQ rotation estimation
  // https://web.stanford.edu/class/cs273/refs/umeyama.pdf
  const auto find_rotation = [&](const std::vector<int>& indices) {
    Eigen::Matrix<double, 3, -1> A(3, indices.size());
    Eigen::Matrix<double, 3, -1> B(3, indices.size());

    for (int i = 0; i < indices.size(); i++) {
      const int index = indices[i];
      const auto& d_c = directions_camera[index];
      const auto& d_l = directions_lidar[index];

      A.col(i) = d_c.head<3>();
      B.col(i) = d_l.head<3>();
    }

    const Eigen::Matrix3d AB = A * B.transpose();

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(AB, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix3d U = svd.matrixU();
    const Eigen::Matrix3d V = svd.matrixV();
    const Eigen::Matrix3d D = svd.singularValues().asDiagonal();
    Eigen::Matrix3d S = Eigen::Matrix3d::Identity();

    double det = U.determinant() * V.determinant();
    if (det < 0.0) {
      S(2, 2) = -1.0;
    }

    const Eigen::Matrix3d R_camera_lidar = U * S * V.transpose();
    return R_camera_lidar;
  };

  const double error_thresh_sq = std::pow(params.ransac_error_thresh, 2);

  int best_num_inliers = 0;
  Eigen::Matrix4d best_R_camera_lidar;

  std::mt19937 mt;
  std::vector<std::mt19937> mts(omp_get_max_threads());
  for (int i = 0; i < mts.size(); i++) {
    mts[i] = std::mt19937(mt() + 8192 * i);
  }

  const int num_samples = 2;

  std::cout << "estimating rotation using LO-RANSAC" << std::endl;
#pragma omp parallel for
  for (int i = 0; i < params.ransac_iterations; i++) {
    const int thread_id = omp_get_thread_num();

    // Sample correspondences
    std::vector<int> indices(num_samples);
    std::uniform_int_distribution<> udist(0, correspondences.size() - 1);
    for (int i = 0; i < num_samples; i++) {
      indices[i] = udist(mts[thread_id]);
    }

    // Estimate rotation
    Eigen::Matrix4d R_camera_lidar = Eigen::Matrix4d::Zero();
    R_camera_lidar.topLeftCorner<3, 3>() = find_rotation(indices);

    // Local optimization
    for (int j = 0; j < 100; j++) {
      // Compute inliers with the current R_camera_lidar
      std::vector<int> inlier_indices;
      for (int k = 0; k < correspondences.size(); k++) {
        const Eigen::Vector4d direction_cam = R_camera_lidar * directions_lidar[k];
        const Eigen::Vector2d pt_2d = proj->project(direction_cam.head<3>());
        if ((correspondences[k].first - pt_2d).squaredNorm() < error_thresh_sq) {
          inlier_indices.push_back(k);
        }
      }

      // Re-estimate rotation with the inliers
      R_camera_lidar.topLeftCorner<3, 3>() = find_rotation(inlier_indices);
    }

    // Count num of inliers
    int num_inliers = 0;
    for (int j = 0; j < correspondences.size(); j++) {
      const Eigen::Vector4d direction_cam = R_camera_lidar * directions_lidar[j];
      const Eigen::Vector2d pt_2d = proj->project(direction_cam.head<3>());

      if ((correspondences[j].first - pt_2d).squaredNorm() < error_thresh_sq) {
        num_inliers++;
      }
    }

#pragma omp critical
    if (num_inliers > best_num_inliers) {
      // Update the best rotation
      best_num_inliers = num_inliers;
      best_R_camera_lidar = R_camera_lidar;
    }
  }

  std::cout << "num_inliers: " << best_num_inliers << " / " << correspondences.size() << std::endl;

  if (inliers) {
    inliers->resize(correspondences.size());
    for (int i = 0; i < correspondences.size(); i++) {
      const Eigen::Vector4d direction_cam = best_R_camera_lidar * directions_lidar[i];
      const Eigen::Vector2d pt_2d = proj->project(direction_cam.head<3>());
      (*inliers)[i] = (correspondences[i].first - pt_2d).squaredNorm() < error_thresh_sq;
    }
  }

  return best_R_camera_lidar.topLeftCorner<3, 3>(0, 0);
}
Eigen::Matrix3d PoseEstimation::estimate_rotation_ransac(
  const camera::GenericCameraBase::ConstPtr& proj,
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>& correspondences,
  std::vector<bool>* inliers) {
  std::cout << "estimating bearing vectors" << std::endl;
  // Compute bearing vectors
  std::vector<Eigen::Vector4d> directions_camera(correspondences.size());
  std::vector<Eigen::Vector4d> directions_lidar(correspondences.size());
  for (int i = 0; i < correspondences.size(); i++) {
      directions_camera[i] << estimate_direction(proj, correspondences[i].first), 0.0;
      directions_lidar[i] << correspondences[i].second.head<3>().normalized(), 0.0;
  }
  using Matrix3Dynamic = Eigen::Matrix<double, 3, Eigen::Dynamic>;

// 转换 directions_camera 到三维矩阵
Matrix3Dynamic directions_camera_3d(3, directions_camera.size());
for (size_t i = 0; i < directions_camera.size(); ++i) {
    directions_camera_3d.col(i) = directions_camera[i].head<3>();
}

// 转换 directions_lidar 到三维矩阵
Matrix3Dynamic directions_lidar_3d(3, directions_lidar.size());
for (size_t i = 0; i < directions_lidar.size(); ++i) {
    directions_lidar_3d.col(i) = directions_lidar[i].head<3>();
}
//    Eigen::Matrix4d rt = Eigen::umeyama(directions_lidar_3d, directions_camera_3d, true);
//  std::cout << "Matrix4d rt:" << std::endl;
    // std::cout << rt << std::endl;
  // LSQ rotation estimation
  // https://web.stanford.edu/class/cs273/refs/umeyama.pdf
  const auto find_rotation = [&](const std::vector<int>& indices) {
    Eigen::Matrix<double, 3, -1> A(3, indices.size());
    Eigen::Matrix<double, 3, -1> B(3, indices.size());

    for (int i = 0; i < indices.size(); i++) {
      const int index = indices[i];
      const auto& d_c = directions_camera[index];
      const auto& d_l = directions_lidar[index];

      A.col(i) = d_c.head<3>();
      B.col(i) = d_l.head<3>();
    }

    const Eigen::Matrix3d AB = A * B.transpose();

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(AB, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix3d U = svd.matrixU();
    const Eigen::Matrix3d V = svd.matrixV();
    const Eigen::Matrix3d D = svd.singularValues().asDiagonal();
    Eigen::Matrix3d S = Eigen::Matrix3d::Identity();

    double det = U.determinant() * V.determinant();
    if (det < 0.0) {
      S(2, 2) = -1.0;
    }

    const Eigen::Matrix3d R_camera_lidar = U * S * V.transpose();
    return R_camera_lidar;
  };

  const double error_thresh_sq = std::pow(params.ransac_error_thresh, 2);

  int best_num_inliers = 0;
  Eigen::Matrix4d best_R_camera_lidar;

  std::mt19937 mt;
  std::vector<std::mt19937> mts(omp_get_max_threads());
  for (int i = 0; i < mts.size(); i++) {
    mts[i] = std::mt19937(mt() + 8192 * i);
  }

  const int num_samples = 2;

  std::cout << "estimating rotation using RANSAC" << std::endl;
#pragma omp parallel for
  for (int i = 0; i < params.ransac_iterations; i++) {
    const int thread_id = omp_get_thread_num();

    // Sample correspondences
    std::vector<int> indices(num_samples);
    std::uniform_int_distribution<> udist(0, correspondences.size() - 1);
    for (int i = 0; i < num_samples; i++) {
      indices[i] = udist(mts[thread_id]);
    }

    // Estimate rotation
    Eigen::Matrix4d R_camera_lidar = Eigen::Matrix4d::Zero();
    R_camera_lidar.topLeftCorner<3, 3>() = find_rotation(indices);

    // Count num of inliers
    int num_inliers = 0;
    for (int j = 0; j < correspondences.size(); j++) {
      const Eigen::Vector4d direction_cam = R_camera_lidar * directions_lidar[j];
      const Eigen::Vector2d pt_2d = proj->project(direction_cam.head<3>());

      if ((correspondences[j].first - pt_2d).squaredNorm() < error_thresh_sq) {
        num_inliers++;
      }
    }

#pragma omp critical
    if (num_inliers > best_num_inliers) {
      // Update the best rotation
      best_num_inliers = num_inliers;
      best_R_camera_lidar = R_camera_lidar;
    }
  }

  std::cout << "num_inliers: " << best_num_inliers << " / " << correspondences.size() << std::endl;

  if (inliers) {
    inliers->resize(correspondences.size());
    for (int i = 0; i < correspondences.size(); i++) {
      const Eigen::Vector4d direction_cam = best_R_camera_lidar * directions_lidar[i];
      const Eigen::Vector2d pt_2d = proj->project(direction_cam.head<3>());
      (*inliers)[i] = (correspondences[i].first - pt_2d).squaredNorm() < error_thresh_sq;
    }
  }

  return best_R_camera_lidar.topLeftCorner<3, 3>(0, 0);
}


Eigen::Isometry3d PoseEstimation::estimate_pose_lsq(
  const camera::GenericCameraBase::ConstPtr& proj,
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>& correspondences,
  const Eigen::Isometry3d& init_T_camera_lidar,std::vector<int> flag_depth_intens) {
  // 
  Sophus::SE3d T_camera_lidar = Sophus::SE3d(init_T_camera_lidar.matrix());
  
  ceres::Problem problem;
  problem.AddParameterBlock(T_camera_lidar.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());
  // The default ceres (2.0.0) on Ubuntu 20.04 does not have manifold.hpp yet
  // problem.AddParameterBlock(T_camera_lidar.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());
  assert(flag_depth_intens.size() % 3 == 0);
int flag=0;
    for (size_t i = 0; i < flag_depth_intens.size(); i += 3) {
        int zero_flag=0;
        if(i==0)
        zero_flag=0;
        else
         zero_flag= flag_depth_intens[i-1];
        int first = flag_depth_intens[i];
        int second = flag_depth_intens[i + 1];
        int third = flag_depth_intens[i + 2];
        double design=double(third-second)/double(first-zero_flag);
      std::cout << "first: " << first << " second: " << second << " third: " << third <<"--------------->"<<design<< std::endl;
  // Create reprojection error costs
  
 for(int j=zero_flag;j<third;j++){
   const auto& [pt_2d, pt_3d] =correspondences[j] ;
     const Eigen::Matrix pt_camera = T_camera_lidar* pt_3d.head<3>();
   
       const auto pt_2d_Lidar = (*proj)(pt_camera);
       Eigen::Vector2d reprojection_error = pt_2d - pt_2d_Lidar;
       // 计算pt_2d和pt_2d_Lidar之间的欧氏距离
double distance = (pt_2d - pt_2d_Lidar).squaredNorm();
  std::ofstream outfile;

    outfile.open("/calib_data/data_keypoint/3d_2d1.txt", std::ios::app);//文件不存在的话会自动创建

    outfile << pt_3d.head<3>().transpose() << " " << pt_2d.transpose() << std::endl;
    outfile.close();
// 打印距离
// std::cout << "Distance: " << distance << std::endl;
       //计算重投影误差
    // if(distance>1000)
    // {
    // flag=flag+1;
    //   continue;
    // }
    if(flag<=first)
    {auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d,1); //200.0/(first-zero_flag)*0.4
       
      auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error);
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }
    else if(flag<=second)
    {
      auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d,1);
     
      auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error); 
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }
    else
    {auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d,1);
 
     auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error);
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }

    flag=flag+1;
  }
 
    }
  // Solve!
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << std::endl;
std::cout << "T_camera_lidaroutliner-------:" << std::endl;
    std::cout << T_camera_lidar.matrix().inverse()<< std::endl;
ceres::Problem problem2;
  problem2.AddParameterBlock(T_camera_lidar.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());
  // The default ceres (2.0.0) on Ubuntu 20.04 does not have manifold.hpp yet
  // problem.AddParameterBlock(T_camera_lidar.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());
  assert(flag_depth_intens.size() % 3 == 0);
  flag=0;
  int remove_in=0;
    for (size_t i = 0; i < flag_depth_intens.size(); i += 3) {
        int zero_flag=0;
        if(i==0)
        zero_flag=0;
        else
         zero_flag= flag_depth_intens[i-1];
        int first = flag_depth_intens[i];
        int second = flag_depth_intens[i + 1];
        int third = flag_depth_intens[i + 2];
        double design=double(third-second)/double(first-zero_flag);
      std::cout << "first: " << first << " second: " << second << " third: " << third <<"--------------->"<<design<< std::endl;
  // Create reprojection error costs
  
 for(int j=zero_flag;j<third;j++){
   const auto& [pt_2d, pt_3d] =correspondences[j] ;
     const Eigen::Matrix pt_camera = T_camera_lidar* pt_3d.head<3>();
   
       const auto pt_2d_Lidar = (*proj)(pt_camera);
       Eigen::Vector2d reprojection_error = pt_2d - pt_2d_Lidar;
       // 计算pt_2d和pt_2d_Lidar之间的欧氏距离
double distance = (pt_2d - pt_2d_Lidar).squaredNorm();

// 打印距离
// std::cout << "Distance: " << distance << std::endl;
      
    if(distance>1000||pt_camera.norm()>60)
    { 
      // std::cout << "Distance: " << distance << std::endl;
    flag=flag+1;
    remove_in=remove_in+1;
      continue;
    }
      std::ofstream outfile;

    outfile.open("/calib_data/data_keypoint/3d_2d2.txt", std::ios::app);//文件不存在的话会自动创建

    outfile << pt_3d.head<3>().transpose() << " " << pt_2d.transpose() << std::endl;
    outfile.close();
    if(flag<=first)
    {auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d,1); //200.0/(first-zero_flag)*0.4
       
      auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error);
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem2.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }
    else if(flag<=second)
    {
      auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d,1);
     
      auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error); 
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem2.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }
    else
    {auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d,1);
 
     auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error);
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem2.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }

    flag=flag+1;
  }
 
    }
  // Solve!
  std::cout << "remove_in:" << remove_in << std::endl;
  ceres::Solver::Options options2;
  options2.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary2;
  ceres::Solve(options2, &problem2, &summary2);

  std::cout << summary2.BriefReport() << std::endl;
std::cout << "T_camera_lidar--inlier:" << std::endl;
    std::cout << T_camera_lidar.matrix().inverse() << std::endl;


ceres::Problem problem3;
  problem3.AddParameterBlock(T_camera_lidar.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());
  // The default ceres (2.0.0) on Ubuntu 20.04 does not have manifold.hpp yet
  // problem.AddParameterBlock(T_camera_lidar.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());
  assert(flag_depth_intens.size() % 3 == 0);
  flag=0;
  int remove_in3=0;
    for (size_t i = 0; i < flag_depth_intens.size(); i += 3) {
        int zero_flag=0;
        if(i==0)
        zero_flag=0;
        else
         zero_flag= flag_depth_intens[i-1];
        int first = flag_depth_intens[i];
        int second = flag_depth_intens[i + 1];
        int third = flag_depth_intens[i + 2];
        double design=double(third-second)/double(first-zero_flag);
      std::cout << "first: " << first << " second: " << second << " third: " << third <<"--------------->"<<design<< std::endl;
  // Create reprojection error costs
  
 for(int j=zero_flag;j<third;j++){
   const auto& [pt_2d, pt_3d] =correspondences[j] ;
     const Eigen::Matrix pt_camera = T_camera_lidar* pt_3d.head<3>();
   
       const auto pt_2d_Lidar = (*proj)(pt_camera);
       Eigen::Vector2d reprojection_error = pt_2d - pt_2d_Lidar;
       // 计算pt_2d和pt_2d_Lidar之间的欧氏距离
double distance = (pt_2d - pt_2d_Lidar).squaredNorm();

// 打印距离
// std::cout << "Distance: " << distance << std::endl;
      
    if(distance>500||pt_camera.norm()>40)
    { 
      // std::cout << "Distance: " << distance << std::endl;
    flag=flag+1;
    remove_in3=remove_in3+1;
      continue;
    }
      std::ofstream outfile;

    outfile.open("/calib_data/data_keypoint/3d_2d3.txt", std::ios::app);//文件不存在的话会自动创建

    outfile << pt_3d.head<3>().transpose() << " " << pt_2d.transpose() << std::endl;
    outfile.close();
    if(flag<=first)
    {auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d,1); //200.0/(first-zero_flag)*0.4
       
      auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error);
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem3.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }
    else if(flag<=second)
    {
      auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d,1);
     
      auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error); 
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem3.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }
    else
    {auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d,1);
 
     auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error);
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem3.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }

    flag=flag+1;
  }
 
    }
  // Solve!
  std::cout << "remove_in3:" << remove_in3 << std::endl;
  ceres::Solver::Options options3;
  options3.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary3;
  ceres::Solve(options3, &problem3, &summary3);

  std::cout << summary3.BriefReport() << std::endl;
std::cout << "T_camera_lidar--inlier---3:" << std::endl;
    std::cout << T_camera_lidar.matrix().inverse() << std::endl;

  
ceres::Problem problem4;

 problem4.AddParameterBlock(T_camera_lidar.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());
  //  std::vector<int> constant_parameters = {0, 1, 2, 3, 4, 5}; // 这里表示我们希望旋转部分固定
  //   ceres::SubsetManifold* subset_manifold = new ceres::SubsetManifold(Sophus::SE3d::num_parameters, {0, 1, 2, 3, 4, 5});
  //   problem4.SetManifold(T_camera_lidar.data(), subset_manifold);
  // The default ceres (2.0.0) on Ubuntu 20.04 does not have manifold.hpp yet
  // problem.AddParameterBlock(T_camera_lidar.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());
  assert(flag_depth_intens.size() % 3 == 0);
  flag=0;
  int remove_in4=0;
    for (size_t i = 0; i < flag_depth_intens.size(); i += 3) {
        int zero_flag=0;
        if(i==0)
        zero_flag=0;
        else
         zero_flag= flag_depth_intens[i-1];
        int first = flag_depth_intens[i];
        int second = flag_depth_intens[i + 1];
        int third = flag_depth_intens[i + 2];
        double design=double(third-second)/double(first-zero_flag);
      std::cout << "first: " << first << " second: " << second << " third: " << third <<"--------------->"<<design<< std::endl;
  // Create reprojection error costs
  
 for(int j=zero_flag;j<third;j++){
   const auto& [pt_2d, pt_3d] =correspondences[j] ;
     const Eigen::Matrix pt_camera = T_camera_lidar* pt_3d.head<3>();
   
       const auto pt_2d_Lidar = (*proj)(pt_camera);
       Eigen::Vector2d reprojection_error = pt_2d - pt_2d_Lidar;
       // 计算pt_2d和pt_2d_Lidar之间的欧氏距离
double distance = (pt_2d - pt_2d_Lidar).squaredNorm();

// 打印距离
// std::cout << "Distance: " << distance << std::endl;
  



    if(distance>150||pt_camera.norm()>40)
    { 
      // std::cout << "Distance: " << distance << std::endl;
    flag=flag+1;
    remove_in4=remove_in4+1;
      continue;
    }  
    std::ofstream outfile;
    //计算所有3d点的模长，除以最大长度作为权重
    double length=pt_3d.head<3>().norm();
    double weight=length/30.0;
    if(weight>1)
    {
       continue;
    }
    else
    {
      weight=1;
    }
    outfile.open("/calib_data/data_keypoint/3d_2d4.txt", std::ios::app);//文件不存在的话会自动创建

    outfile << pt_3d.head<3>().transpose() << " " << pt_2d.transpose() << std::endl;
    outfile.close();
    if(flag<=first)
    {auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d,1.0/weight); //200.0/(first-zero_flag)*0.4
       //追加保存3d和2d点到txt文件，一组为一行
    // std::ofstream outfile;

    // outfile.open("3d_2d.txt", std::ios::app);
    // outfile << pt_3d.head<3>().transpose() << " " << pt_2d.transpose() << std::endl;
    // outfile.close();

      auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error);
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem4.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }
    else if(flag<=second)
    {
      auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d,1.0/weight);
     
      auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error); 
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem4.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }
    else
    {auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d, 1.0/weight);
 
     auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error);
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem4.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }

    flag=flag+1;
  }
 
    }
  // Solve!
  std::cout << "remove_in4:" << remove_in4 << std::endl;
  ceres::Solver::Options options4;
  options4.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary4;
  ceres::Solve(options4, &problem4, &summary4);

  std::cout << summary4.BriefReport() << std::endl;
std::cout << "T_camera_lidar--inlier---4:" << std::endl;
    std::cout << T_camera_lidar.matrix().inverse() << std::endl;


    ceres::Problem problem4_1;

 problem4_1.AddParameterBlock(T_camera_lidar.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());
  //  std::vector<int> constant_parameters = {0, 1, 2, 3, 4, 5}; // 这里表示我们希望旋转部分固定
  //   ceres::SubsetManifold* subset_manifold = new ceres::SubsetManifold(Sophus::SE3d::num_parameters, {0, 1, 2, 3, 4, 5});
  //   problem4.SetManifold(T_camera_lidar.data(), subset_manifold);
  // The default ceres (2.0.0) on Ubuntu 20.04 does not have manifold.hpp yet
  // problem.AddParameterBlock(T_camera_lidar.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());
  assert(flag_depth_intens.size() % 3 == 0);
  flag=0;
  int remove_in4_1=0;
    for (size_t i = 0; i < flag_depth_intens.size(); i += 3) {
        int zero_flag=0;
        if(i==0)
        zero_flag=0;
        else
         zero_flag= flag_depth_intens[i-1];
        int first = flag_depth_intens[i];
        int second = flag_depth_intens[i + 1];
        int third = flag_depth_intens[i + 2];
        double design=double(third-second)/double(first-zero_flag);
      std::cout << "first: " << first << " second: " << second << " third: " << third <<"--------------->"<<design<< std::endl;
  // Create reprojection error costs
  
 for(int j=zero_flag;j<third;j++){
   const auto& [pt_2d, pt_3d] =correspondences[j] ;
     const Eigen::Matrix pt_camera = T_camera_lidar* pt_3d.head<3>();
   
       const auto pt_2d_Lidar = (*proj)(pt_camera);
       Eigen::Vector2d reprojection_error = pt_2d - pt_2d_Lidar;
       // 计算pt_2d和pt_2d_Lidar之间的欧氏距离
double distance = (pt_2d - pt_2d_Lidar).squaredNorm();

// 打印距离
// std::cout << "Distance: " << distance << std::endl;
  



    if(distance>50)
    { 
      // std::cout << "Distance: " << distance << std::endl;
    flag=flag+1;
    remove_in4_1=remove_in4_1+1;
      continue;
    }  
    std::ofstream outfile;
    //计算所有3d点的模长，除以最大长度作为权重
    double length=pt_3d.head<3>().norm();
    double weight=length/30.0;
    if(weight>1)
    {
       continue;
    }
    else
    {
      weight=1;
    }
    outfile.open("/calib_data/data_keypoint/3d_2d5.txt", std::ios::app);//文件不存在的话会自动创建

    outfile << pt_3d.head<3>().transpose() << " " << pt_2d.transpose() << std::endl;
    outfile.close();
    if(flag<=first)
    {auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d,1.0/weight); //200.0/(first-zero_flag)*0.4
       //追加保存3d和2d点到txt文件，一组为一行
    // std::ofstream outfile;

    // outfile.open("3d_2d.txt", std::ios::app);
    // outfile << pt_3d.head<3>().transpose() << " " << pt_2d.transpose() << std::endl;
    // outfile.close();

      auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error);
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem4_1.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }
    else if(flag<=second)
    {
      auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d,1.0/weight);
     
      auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error); 
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem4_1.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }
    else
    {auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d, 1.0/weight);
 
     auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error);
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem4_1.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }

    flag=flag+1;
  }
 
    }
  // Solve!
  std::cout << "remove_in4_1:" << remove_in4_1 << std::endl;
  ceres::Solver::Options options4_1;
  options4_1.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary4_1;
  ceres::Solve(options4_1, &problem4_1, &summary4_1);

  std::cout << summary4_1.BriefReport() << std::endl;
std::cout << "T_camera_lidar--inlier---4_14_1:" << std::endl;
    std::cout << T_camera_lidar.matrix().inverse() << std::endl;

  ceres::Problem problem4_2;

 problem4_2.AddParameterBlock(T_camera_lidar.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());
  //  std::vector<int> constant_parameters = {0, 1, 2, 3, 4, 5}; // 这里表示我们希望旋转部分固定
  //   ceres::SubsetManifold* subset_manifold = new ceres::SubsetManifold(Sophus::SE3d::num_parameters, {0, 1, 2, 3, 4, 5});
  //   problem4.SetManifold(T_camera_lidar.data(), subset_manifold);
  // The default ceres (2.0.0) on Ubuntu 20.04 does not have manifold.hpp yet
  // problem.AddParameterBlock(T_camera_lidar.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());
  assert(flag_depth_intens.size() % 3 == 0);
  flag=0;
  int remove_in4_2=0;
    for (size_t i = 0; i < flag_depth_intens.size(); i += 3) {
        int zero_flag=0;
        if(i==0)
        zero_flag=0;
        else
         zero_flag= flag_depth_intens[i-1];
        int first = flag_depth_intens[i];
        int second = flag_depth_intens[i + 1];
        int third = flag_depth_intens[i + 2];
        double design=double(third-second)/double(first-zero_flag);
      std::cout << "first: " << first << " second: " << second << " third: " << third <<"--------------->"<<design<< std::endl;
  // Create reprojection error costs
  
 for(int j=zero_flag;j<third;j++){
   const auto& [pt_2d, pt_3d] =correspondences[j] ;
     const Eigen::Matrix pt_camera = T_camera_lidar* pt_3d.head<3>();
   
       const auto pt_2d_Lidar = (*proj)(pt_camera);
       Eigen::Vector2d reprojection_error = pt_2d - pt_2d_Lidar;
       // 计算pt_2d和pt_2d_Lidar之间的欧氏距离
double distance = (pt_2d - pt_2d_Lidar).squaredNorm();

// 打印距离
// std::cout << "Distance: " << distance << std::endl;
  



    if(distance>20||pt_camera.norm()>40)
    { 
      // std::cout << "Distance: " << distance << std::endl;
    flag=flag+1;
    remove_in4_2=remove_in4_2+1;
      continue;
    }  
    std::ofstream outfile;
    //计算所有3d点的模长，除以最大长度作为权重
    double length=pt_3d.head<3>().norm();
    double weight=length/30.0;
    if(weight>1)
    {
       continue;
    }
    else
    {
      weight=1;
    }
    outfile.open("/calib_data/data_keypoint/3d_2d6.txt", std::ios::app);//文件不存在的话会自动创建

    outfile << pt_3d.head<3>().transpose() << " " << pt_2d.transpose() << std::endl;
    outfile.close();
    if(flag<=first)
    {auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d,1.0/weight); //200.0/(first-zero_flag)*0.4
       //追加保存3d和2d点到txt文件，一组为一行
    // std::ofstream outfile;

    // outfile.open("3d_2d.txt", std::ios::app);
    // outfile << pt_3d.head<3>().transpose() << " " << pt_2d.transpose() << std::endl;
    // outfile.close();

      auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error);
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem4_2.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }
    else if(flag<=second)
    {
      auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d,1.0/weight);
     
      auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error); 
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem4_2.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }
    else
    {auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d, 1.0/weight);
 
     auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error);
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem4_2.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }

    flag=flag+1;
  }
 
    }
  // Solve!
  std::cout << "remove_in4_2:" << remove_in4_2 << std::endl;
  ceres::Solver::Options options4_2;
  options4_2.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary4_2;
  ceres::Solve(options4_2, &problem4_2, &summary4_2);

  std::cout << summary4_2.BriefReport() << std::endl;
std::cout << "T_camera_lidar--inlier---4_24_2:" << std::endl;
    std::cout << T_camera_lidar.matrix().inverse() << std::endl;


ceres::Problem problem5;

 problem5.AddParameterBlock(T_camera_lidar.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());
  //  std::vector<int> constant_parameters = {0, 1, 2, 3, 4, 5}; // 这里表示我们希望旋转部分固定
    ceres::SubsetManifold* subset_manifold = new ceres::SubsetManifold(Sophus::SE3d::num_parameters, {0, 1, 2, 3, 4, 5});
    problem5.SetManifold(T_camera_lidar.data(), subset_manifold);
  // The default ceres (2.0.0) on Ubuntu 20.04 does not have manifold.hpp yet
  // problem.AddParameterBlock(T_camera_lidar.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());
  assert(flag_depth_intens.size() % 3 == 0);
  flag=0;
  int remove_in5=0;
    for (size_t i = 0; i < flag_depth_intens.size(); i += 3) {
        int zero_flag=0;
        if(i==0)
        zero_flag=0;
        else
         zero_flag= flag_depth_intens[i-1];
        int first = flag_depth_intens[i];
        int second = flag_depth_intens[i + 1];
        int third = flag_depth_intens[i + 2];
        double design=double(third-second)/double(first-zero_flag);
      std::cout << "first: " << first << " second: " << second << " third: " << third <<"--------------->"<<design<< std::endl;
  // Create reprojection error costs
  
 for(int j=zero_flag;j<third;j++){
   const auto& [pt_2d, pt_3d] =correspondences[j] ;
     const Eigen::Matrix pt_camera = T_camera_lidar* pt_3d.head<3>();
   
       const auto pt_2d_Lidar = (*proj)(pt_camera);
       Eigen::Vector2d reprojection_error = pt_2d - pt_2d_Lidar;
       // 计算pt_2d和pt_2d_Lidar之间的欧氏距离
double distance = (pt_2d - pt_2d_Lidar).squaredNorm();

// 打印距离
// std::cout << "Distance: " << distance << std::endl;
  



    if(distance>200)
    { 
      // std::cout << "Distance: " << distance << std::endl;
    flag=flag+1;
    remove_in5=remove_in5+1;
      continue;
    }  
    std::ofstream outfile;
    //计算所有3d点的模长，除以最大长度作为权重
    double length=pt_3d.head<3>().norm();
    double weight=length/30.0;
    if(weight>1)
    {
       continue;
    }
    outfile.open("/calib_data/data_keypoint/3d_2d4.txt", std::ios::app);//文件不存在的话会自动创建

    outfile << pt_3d.head<3>().transpose() << " " << pt_2d.transpose() << std::endl;
    outfile.close();
    if(flag<=first)
    {auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d,1.0/weight); //200.0/(first-zero_flag)*0.4
       //追加保存3d和2d点到txt文件，一组为一行
    // std::ofstream outfile;

    // outfile.open("3d_2d.txt", std::ios::app);
    // outfile << pt_3d.head<3>().transpose() << " " << pt_2d.transpose() << std::endl;
    // outfile.close();

      auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error);
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem5.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }
    else if(flag<=second)
    {
      auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d,1.0/weight);
     
      auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error); 
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem5.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }
    else
    {auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d, 1.0/weight);
 
     auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error);
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem5.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
    }

    flag=flag+1;
  }
 
    }
  // Solve!
  std::cout << "remove_in5:" << remove_in5 << std::endl;
  ceres::Solver::Options options5;
  options5.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary5;
  ceres::Solve(options5, &problem5, &summary5);

  std::cout << summary5.BriefReport() << std::endl;
std::cout << "T_camera_lidar--inlier---5:" << std::endl;
    std::cout << T_camera_lidar.matrix().inverse() << std::endl;

  return Eigen::Isometry3d(T_camera_lidar.matrix());
}

}  // namespace vlcal

#include <vlcal/preprocess/static_point_cloud_integrator.hpp>
#include <unordered_map>
#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <flann/flann.hpp>
#include <omp.h>
namespace vlcal {

StaticPointCloudIntegratorParams::StaticPointCloudIntegratorParams() {
  visualize = false;
  voxel_resolution = 0.05;
  min_distance = 1.0;
 }

StaticPointCloudIntegratorParams::~StaticPointCloudIntegratorParams() {}

StaticPointCloudIntegrator::StaticPointCloudIntegrator(const StaticPointCloudIntegratorParams& params) : params(params) {
  if (params.visualize) {
    auto viewer = guik::LightViewer::instance();
    viewer->clear_drawables();
  }
}


StaticPointCloudIntegrator::~StaticPointCloudIntegrator() {
  flag_static=0;
}

void StaticPointCloudIntegrator::insert_points(const Frame::ConstPtr& raw_points) {
  
  //  std::unordered_map<Eigen::Vector3i, int, Vector3iHash> compute_voxel_point_counts(raw_points->size());
  for (int i = 0; i < raw_points->size(); i++) {
    const auto& pt = raw_points->points[i];
    const double intensity = raw_points->intensities[i];

    if (pt.head<3>().norm() < params.min_distance) {
      continue;
    }

    const Eigen::Vector3i coord = (pt / params.voxel_resolution).array().floor().cast<int>().head<3>();
    voxelgrid[coord] = Eigen::Vector4d(pt[0], pt[1], pt[2], intensity);
    compute_voxel_point_counts[coord]++;
  }

  if (params.visualize) {
    auto viewer = guik::LightViewer::instance();
    auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(raw_points->points, raw_points->size());
    viewer->update_drawable(guik::anon(), cloud_buffer, guik::Rainbow());
    viewer->update_drawable("current", cloud_buffer, guik::FlatOrange().set_point_scale(2.0f));
    viewer->spin_once();
  }
}

float distance(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2) {
    return (p1 - p2).norm();
}

// 移除周围点很少的点及其对应的强度值

 std::vector<bool>  removeSparsePoints(std::vector<Eigen::Vector3f>& points, std::vector<float>& intensities, float minNeighborDistance) {
    // 构建 FLANN 的数据集
    flann::Matrix<float> dataset(new float[points.size() * 3], points.size(), 3);
    for (size_t i = 0; i < points.size(); ++i) {
        dataset[i][0] = points[i].x();
        dataset[i][1] = points[i].y();
        dataset[i][2] = points[i].z();
    }
    // 构建 FLANN 的索引
    flann::Index<flann::L2<float>> index(dataset, flann::KDTreeIndexParams(4)); // 使用 KD 树，参数 4 表示每个叶子节点包含的最大数据点数
    index.buildIndex();
    std::vector<bool> toRemove(points.size(), false); // 记录要移除的点
    // 对于每个点
#pragma omp parallel for
for (size_t i = 0; i < points.size(); ++i) {
    // 查询最近的 10 个点
    flann::Matrix<int> indices(new int[10], 1, 10);
    flann::Matrix<float> distances(new float[10], 1, 10);
    flann::Matrix<float> query(new float[1 * 3], 1, 3);
    query[0][0] = points[i].x();
    query[0][1] = points[i].y();
    query[0][2] = points[i].z();
    index.knnSearch(query, indices, distances, 10, flann::SearchParams());
    int count = 0; // 计算邻居数量
    for (size_t j = 0; j < 10; ++j) {
        if (distances[0][j] < minNeighborDistance) {
            count++;
        }
    }
    // 如果周围点很少，则标记要移除该点
    if (count < 10) { // 根据需要修改阈值
        toRemove[i] = true;
    }
    delete[] query.ptr();
    delete[] indices.ptr();
    delete[] distances.ptr();
}
return toRemove;
}
Frame::ConstPtr StaticPointCloudIntegrator::get_points() {
  std::vector<Eigen::Vector3f> points;
  std::vector<float> intensities;

  points.reserve(voxelgrid.size());
  intensities.reserve(voxelgrid.size());
// for (const auto& voxel : compute_voxel_point_counts) {
//         std::cout << "Voxel at [" << voxel.first.transpose() << "] has " << voxel.second << " points." << std::endl;
//     }
  //查看  for (const auto& voxel : voxelgrid)的循环次数

  for (const auto& voxel : voxelgrid) {
  if (compute_voxel_point_counts[voxel.first] >0) { 
    points.emplace_back(voxel.second.cast<float>().head<3>());
    intensities.emplace_back(voxel.second.w());
  }
  else
  {
    std::cout << "Voxel at [" << voxel.first.transpose() << "] has " <<compute_voxel_point_counts[voxel.first]  << " points." << std::endl;

  }
  }
//    float minNeighborDistance = 0.01; // 设置最小邻居距离阈值
//     std::vector<bool> toRemove= removeSparsePoints(points, intensities, minNeighborDistance);
// auto itPoint = points.begin();
// auto itIntensity = intensities.begin();
// for (auto itRemove = toRemove.begin(); itRemove != toRemove.end(); ) {
//     if (*itRemove) {
//         itPoint = points.erase(itPoint);
//         itIntensity = intensities.erase(itIntensity);
//         itRemove = toRemove.erase(itRemove);
//     } else {
//         ++itPoint;
//         ++itIntensity;
//         ++itRemove;
//     }
// }
  
  auto frame = std::make_shared<FrameCPU>(points);
  //计算点云的深度
  std::vector<float> depth;
  depth.reserve(points.size());
  for (const auto& pt : points) {
    depth.emplace_back(pt.norm());
  }
  frame->add_intensities(intensities);
  return frame;
}

}  // namespace vlcal
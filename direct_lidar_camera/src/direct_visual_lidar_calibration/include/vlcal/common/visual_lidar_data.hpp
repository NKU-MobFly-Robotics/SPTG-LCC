#pragma once

#include <memory>
#include <iostream>
#include <opencv2/core.hpp>
#include <vlcal/common/frame_cpu.hpp>

namespace vlcal {

struct VisualLiDARData {
public:
  using Ptr = std::shared_ptr<VisualLiDARData>;
  using ConstPtr = std::shared_ptr<const VisualLiDARData>;

  VisualLiDARData() {}
  VisualLiDARData(const cv::Mat& image, const FrameCPU::Ptr& points) : image(image), points(points) {}
   VisualLiDARData(const cv::Mat& image,const cv::Mat& image2, const FrameCPU::Ptr& points) : image(image), image2(image2),points(points) {}

  VisualLiDARData(const std::string& data_path, const std::string& bag_name);
  ~VisualLiDARData();

public:
  cv::Mat image;
   cv::Mat image_depth;
    cv::Mat image2;
  FrameCPU::Ptr points;
};

}  // namespace vlcal

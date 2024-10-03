#pragma once

#include <vlcal/common/frame.hpp>

namespace vlcal {

class PointCloudIntegrator {
public:
  virtual ~PointCloudIntegrator() {}
       int flag_static;
  virtual void insert_points(const Frame::ConstPtr& raw_points) = 0;
  virtual Frame::ConstPtr get_points() = 0;
  
};

}
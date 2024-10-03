#pragma once

#include <deque>
#include <vlcal/common/raw_points.hpp>

namespace vlcal {

/**
 * @brief Parameter to control the absolute point time handling behavior
 */
struct AbsPointTimeParams {
public:
  AbsPointTimeParams() {
    replace_frame_timestamp = true;
    wrt_first_frame_timestamp = true;
  }

  bool replace_frame_timestamp;    ///< If true, replace the frame timestamp with point timestamp
  bool wrt_first_frame_timestamp;  ///< If true, use the timestamp with respect to the very first points msg
};

/**
 * @brief Utility class to unify timestamp convension
 */
class TimeKeeper {
public:
  TimeKeeper(const AbsPointTimeParams& abs_params = AbsPointTimeParams());
  ~TimeKeeper();

  /**
   * @brief Replace frame and point timestamps
   * @note  Frame timestamp must be the one at the moment when the first point is acquired
   * @note  Point timestamps must be relative with respect to the first point
   * @return Return false if the timestamp is invalid and the frame should be skipped
   */
  bool process(const vlcal::RawPoints::Ptr& points);

  /**
   * @brief Check if IMU and LiDAR data are (very roughly) synchronized
   * @return Return false if the IMU data is invalid and should be skipped
   */
  bool validate_imu_stamp(const double imu_stamp);

private:
  void replace_points_stamp(const vlcal::RawPoints::Ptr& points);
  double estimate_scan_duration(const double stamp);

private:
  const AbsPointTimeParams abs_params;

  bool first_warning;        ///< Flag to show warning messages only once
  double last_points_stamp;  ///< Timestamp of the last LiDAR frame
  double last_imu_stamp;     ///< Timestamp of the last IMU data

  int num_scans;                   ///< Number of frames for scan duration estimation
  double first_points_stamp;       ///< Timestamp of the first frame for scan duration estimation
  double estimated_scan_duration;  ///< Estimated scan duration

  double point_time_offset;  ///< Offset to correct time shift of point times
};

}  // namespace vlcal
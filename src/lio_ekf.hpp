// MIT License
//
// Copyright (c) 2024 Yibin Wu, Tiziano Guadagnino
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <Eigen/Dense>
#include <deque>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>

#include "imuPropagation.hpp"
#include "kiss_icp/core/Threshold.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"
#include "kiss_icp/pipeline/KissICP.hpp"
#include "lio_types.hpp"
// namespace Eigen
namespace lio_ekf {

#define STATE_RANK 15
#define STATE_NOISE_RANK 12

using Vector3dVector = std::vector<Eigen::Vector3d>;

class LIOEKF {

public:
  explicit LIOEKF(LIOPara &lio_para)
      : liopara_(lio_para), lio_map_(lio_para.voxel_size, lio_para.max_range,
                                     lio_para.max_points_per_voxel) {}

  LIOEKF()
      : lio_map_(liopara_.voxel_size, liopara_.max_range,
                 liopara_.max_points_per_voxel) {}

  ~LIOEKF() = default;

  void init();

  inline void addImuData(std::deque<lio_ekf::IMU> &imu_buffer_,
                         bool compensate = false) {
    const IMU &imu = imu_buffer_.front();
    imupre_ = imucur_;
    imucur_ = imu;
    // set current IMU timestamp as the current state timestamp
    imu_t_ = imucur_.timestamp;

    if (compensate) {
      imuCompensate(imucur_, imuerror_);
    }
    imu_buffer_.pop_front();
  }

  inline void
  addLidarData(std::deque<std::vector<Eigen::Vector3d>> &lidar_buffer_,
               std::deque<double> &lidar_time_buffer_,
               std::deque<std_msgs::Header> &lidar_header_buffer_,
               std::deque<std::vector<double>> &points_per_scan_time_buffer_) {
    const std::vector<Eigen::Vector3d> &points = lidar_buffer_.front();
    double timestamp = lidar_time_buffer_.front();

    lidar_header_ = lidar_header_buffer_.front();
    curpoints_ = points;

    lidar_t_ = timestamp;
    if (!points_per_scan_time_buffer_.empty()) {
      timestamps_per_points_ = points_per_scan_time_buffer_.front();
      points_per_scan_time_buffer_.pop_front();
    } else {
      timestamps_per_points_ = std::vector<double>(curpoints_.size(), 0.0);
    }

    lidar_header_buffer_.pop_front();
    lidar_buffer_.pop_front();
    lidar_time_buffer_.pop_front();
  }

  void newImuProcess();

  inline double getImutimestamp() const { return imu_t_; }

  inline double getLiDARtimestamp() const { return lidar_t_; }

  NavState getNavState();

  inline Eigen::MatrixXd getCovariance() { return Cov_; }

  inline std::vector<Eigen::Vector3d> LocalMap() const {
    return lio_map_.Pointcloud();
  };
  inline std::vector<Eigen::Vector3d> getFrame_w() const {
    return curpoints_w_;
  };
  inline std::vector<Eigen::Vector3d> getKetPoints_w() const {
    return keypoints_w_;
  };

  std_msgs::Header lidar_header_;

  bool lidar_updated_ = false; // if has done lidar update in the filter system

  inline bool lidarinOutage();

  Eigen::Matrix4d poseTran(const Eigen::Matrix4d pose1,
                           const Eigen::Matrix4d pose2);

  inline void TransformPoints(const Eigen::Matrix4d &T,
                              std::vector<Eigen::Vector3d> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) {
                     const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
                     const Eigen::Vector3d translation = T.block<3, 1>(0, 3);
                     return Eigen::Vector3d{R * point + translation};
                   });
  }

  Eigen::Vector3d deskewPoint(
      const Eigen::Vector3d &point, const double &timestamp,
      const std::vector<std::pair<double, Sophus::SE3d>> &posesWithinScan);

private:
  void navStateInitialization(const NavState &initstate,
                              const NavState &initstate_std);

  int isToUpdate(double imutime1, double imutime2, double updatetime) const;

  void statePropagation(IMU &imupre, IMU &imucur);

  auto processScan();
  void lidarUpdate();

  void ekfPredict(Eigen::Matrix15d &Phi, Eigen::Matrix15d &Qd);

  void ekfUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H, Eigen::MatrixXd &R);

  void stateFeedback();

  inline void checkStateCov() {

    for (int i = 0; i < STATE_RANK; i++) {
      if (Cov_(i, i) < 0) {
        std::cout << "Covariance is negative at " << std::setprecision(10)
                  << getImutimestamp() << " !" << std::endl;
        std::exit(EXIT_FAILURE);
      }
    }
  }

  // void lio_initialization();
  Eigen::Matrix4d interp_pose(double t1, double t2, Eigen::Matrix4d pose1,
                              Eigen::Matrix4d pose2, double mid_t);

  // kiss-icp pipeline
  std::pair<Vector3dVector, Vector3dVector>
  Voxelize(const std::vector<Eigen::Vector3d> &frame) const;

  void initFirstLiDAR(const int lidarUpdateFlag);

  void resetCov(Eigen::Matrix15d &Cov);

private:
  LIOPara liopara_;

  double imu_t_, last_update_t_, lidar_t_, first_lidar_t = 0;

  // time align error threshold
  double TIME_ALIGN_ERR = 0.001;

  // raw imudata
  IMU imupre_; // previous imu data
  IMU imucur_; // current imu data

  // imu state (position, velocity, attitude and imu error)
  BodyState bodystate_cur_;
  BodyState bodystate_pre_;

  ImuError imuerror_;

  // ekf variables

  Eigen::Matrix15d Cov_;
  Eigen::Matrix12d State_Noise_Cov_;
  Eigen::Vector15d delta_x_;
  Eigen::Matrix15d Imu_Prediction_Covariance_ = Eigen::Matrix15d::Zero();

  // state ID and noise ID
  enum StateID {
    POS_ID = 0,
    VEL_ID = 3,
    ATT_ID = 6,
    GYRO_BIAS_ID = 9,
    ACC_BIAS_ID = 12
  };
  enum NoiseID {
    VEL_RANDOMWALK_ID = 0,
    ANGEL_RANDOMWALK_ID = 3,
    GYRO_BIAS_STD_ID = 6,
    ACC_BIAS_STD_ID = 9
  };

  kiss_icp::VoxelHashMap lio_map_;

  Vector3dVector curpoints_, curpoints_w_, keypoints_w_;

  std::vector<double>
      timestamps_per_points_; // Timestamps for each points in one frame

  bool lio_initialized_ = false;
  bool is_first_imu_ = true;
  bool is_first_lidar_ = true;
};
} // namespace lio_ekf

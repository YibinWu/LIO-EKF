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
#include <iomanip>
#include <iostream>
#include <sophus/se3.hpp>
#include <vector>

namespace Eigen {
using Matrix15d = Eigen::Matrix<double, 15, 15>;
using Matrix12d = Eigen::Matrix<double, 12, 12>;
using Matrix15_12d = Eigen::Matrix<double, 15, 12>;
using Matrix3_15d = Eigen::Matrix<double, 3, 15>;
using Vector15d = Eigen::Matrix<double, 15, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
} // namespace Eigen

namespace lio_ekf {

static constexpr double D2R = (M_PI / 180.0);
static constexpr double R2D = (180.0 / M_PI);
static constexpr double NormG = 9.782940329221166;
struct BodyState {
  Sophus::SE3d pose = Sophus::SE3d();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
};

struct ImuError {
  Eigen::Vector3d gyrbias = Eigen::Vector3d::Zero();
  ;
  Eigen::Vector3d accbias = Eigen::Vector3d::Zero();
  ;
};

struct NavState {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  ;
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  ;
  Eigen::Vector3d euler = Eigen::Vector3d::Zero();
  ;

  ImuError imuerror;
};

struct ImuNoise {
  Eigen::Vector3d angle_randomwalk = Eigen::Vector3d::Zero();
  ;
  Eigen::Vector3d velocity_randomwalk = Eigen::Vector3d::Zero();
  ;
  Eigen::Vector3d gyrbias_std = Eigen::Vector3d::Zero();
  ; // gyro bias standard error
  Eigen::Vector3d accbias_std = Eigen::Vector3d::Zero();
  ; // accelerometer bias standard error
  double correlation_time;
};

struct IMU {
  double timestamp;
  double dt; // time interval between two imu eppoch

  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  ;
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  ;
};

struct ResultTuple {
  ResultTuple() {
    HTRH.setZero();
    HTRz.setZero();
  }

  ResultTuple operator+(const ResultTuple &other) {
    this->HTRH += other.HTRH;
    this->HTRz += other.HTRz;
    return *this;
  }

  Eigen::Matrix15d HTRH;
  Eigen::Vector15d HTRz;
};

struct LIOPara {

  bool deskew;
  bool preprocess;
  float max_range;
  float min_range;
  int max_points_per_voxel;

  float voxel_size;

  int max_iteration;

  // initial state and state standard deviation
  NavState initstate;
  NavState initstate_std;

  // imu noise parameters
  ImuNoise imunoise;

  // extrinsic parameters between lidar and imu Trans_lidar_imu, lidar->imu
  Eigen::Matrix4d Trans_lidar_imu = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d imu_tran_R = Eigen::Matrix3d::Identity();
  // the extrinsic lidar-inertial parameters before transforming the imu frame
  Eigen::Matrix4d Trans_lidar_imu_origin = Eigen::Matrix4d::Identity();
};
} // namespace lio_ekf
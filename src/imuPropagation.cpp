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
#include "imuPropagation.hpp"
#include "rotation.hpp"
#include <sophus/so3.hpp>

namespace lio_ekf {

void insMechanization(const BodyState &pvapre, BodyState &pvacur,
                      const IMU &imupre, const IMU &imucur) {

  // perform velocity update, position updata and attitude update in sequence,
  // irreversible order

  Eigen::Vector3d d_vfb, d_vfn, d_vgn, gl;
  Eigen::Vector3d temp1, temp2, temp3;
  Eigen::Vector3d imucur_dvel, imucur_dtheta, imupre_dvel, imupre_dtheta;

  imucur_dvel = imucur.linear_acceleration * imucur.dt;
  imucur_dtheta = imucur.angular_velocity * imucur.dt;
  imupre_dvel = imupre.linear_acceleration * imupre.dt;
  imupre_dtheta = imupre.angular_velocity * imupre.dt;

  // rotational and sculling motion
  temp1 = imucur_dtheta.cross(imucur_dvel) / 2;
  temp2 = imupre_dtheta.cross(imucur_dvel) / 12;
  temp3 = imupre_dvel.cross(imucur_dtheta) / 12;

  // velocity increment due to the specific force
  d_vfb = imucur_dvel + temp1 + temp2 + temp3;

  // velocity increment dut to the specfic force projected to the n-frame
  d_vfn = pvapre.pose.rotationMatrix() * d_vfb;

  // velocity increment due to the gravity and Coriolis force
  gl << 0, 0, NormG;
  d_vgn = gl * imucur.dt;

  // velocity update finish
  pvacur.vel = pvapre.vel + d_vfn + d_vgn;

  Eigen::Vector3d midvel;

  // recompute velocity and position at k-1/2
  midvel = (pvacur.vel + pvapre.vel) / 2;
  pvacur.pose.translation() += midvel * imucur.dt;
  Eigen::Vector3d rot_bframe;

  // b-frame rotation vector (b(k) with respect to b(k-1)-frame)
  // compensate the second-order coning correction term.
  rot_bframe = imucur_dtheta + imupre_dtheta.cross(imucur_dtheta) / 12;

  pvacur.pose.so3() = pvapre.pose.so3() * Sophus::SO3d::exp(rot_bframe);
}

void imuInterpolate(const IMU &imu1, IMU &imu2, const double timestamp,
                    IMU &midimu) {

  if (imu1.timestamp > timestamp || imu2.timestamp < timestamp) {
    return;
  }

  double lambda =
      (timestamp - imu1.timestamp) / (imu2.timestamp - imu1.timestamp);

  midimu.timestamp = timestamp;

  midimu.dt = timestamp - imu1.timestamp;
  midimu.angular_velocity =
      lambda * imu2.angular_velocity + (1 - lambda) * imu1.angular_velocity;
  midimu.linear_acceleration = lambda * imu2.linear_acceleration +
                               (1 - lambda) * imu1.linear_acceleration;

  imu2.dt = imu2.dt - midimu.dt;
}

void imuCompensate(IMU &imu, ImuError &imuerror) {

  // compensate the imu bias error
  imu.angular_velocity -= imuerror.gyrbias;
  imu.linear_acceleration -= imuerror.accbias;
}

} // namespace lio_ekf
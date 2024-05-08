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

#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

class Rotation {

public:
  static Quaterniond matrix2quaternion(const Matrix3d &matrix) {
    return Quaterniond(matrix);
  }

  static Matrix3d quaternion2matrix(const Quaterniond &quaternion) {
    return quaternion.toRotationMatrix();
  }

  static Vector3d matrix2euler(const Eigen::Matrix3d &dcm) {
    Vector3d euler;

    euler[1] =
        atan(-dcm(2, 0) / sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));

    if (dcm(2, 0) <= -0.999) {
      euler[0] = atan2(dcm(2, 1), dcm(2, 2));
      euler[2] = atan2((dcm(1, 2) - dcm(0, 1)), (dcm(0, 2) + dcm(1, 1)));
    } else if (dcm(2, 0) >= 0.999) {
      euler[0] = atan2(dcm(2, 1), dcm(2, 2));
      euler[2] = M_PI + atan2((dcm(1, 2) + dcm(0, 1)), (dcm(0, 2) - dcm(1, 1)));
    } else {
      euler[0] = atan2(dcm(2, 1), dcm(2, 2));
      euler[2] = atan2(dcm(1, 0), dcm(0, 0));
    }

    // heading 0~2PI
    if (euler[2] < 0) {
      euler[2] = M_PI * 2 + euler[2];
    }
    // else if(euler[2] > 2 * M_PI)

    return euler;
  }

  static Vector3d quaternion2euler(const Quaterniond &quaternion) {
    return matrix2euler(quaternion.toRotationMatrix());
  }

  static Quaterniond rotvec2quaternion(const Vector3d &rotvec) {
    double angle = rotvec.norm();
    Vector3d vec = rotvec.normalized();
    return Quaterniond(Eigen::AngleAxisd(angle, vec));
  }

  static Vector3d quaternion2vector(const Quaterniond &quaternion) {
    Eigen::AngleAxisd axisd(quaternion);
    return axisd.angle() * axisd.axis();
  }

  static Matrix3d euler2matrix(const Vector3d &euler) {
    return Matrix3d(Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                    Eigen::AngleAxisd(euler[0], Vector3d::UnitX()));
  }

  static Quaterniond euler2quaternion(const Vector3d &euler) {
    return Quaterniond(Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                       Eigen::AngleAxisd(euler[0], Vector3d::UnitX()));
  }

  static Matrix3d skewSymmetric(const Vector3d &vector) {
    Matrix3d mat;
    mat << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1),
        vector(0), 0;
    return mat;
  }

  static Eigen::Matrix4d quaternionleft(const Quaterniond &q) {
    Eigen::Matrix4d ans;
    ans(0, 0) = q.w();
    ans.block<1, 3>(0, 1) = -q.vec().transpose();
    ans.block<3, 1>(1, 0) = q.vec();
    ans.block<3, 3>(1, 1) =
        q.w() * Eigen::Matrix3d::Identity() + skewSymmetric(q.vec());
    return ans;
  }

  static Eigen::Matrix4d quaternionright(const Quaterniond &p) {
    Eigen::Matrix4d ans;
    ans(0, 0) = p.w();
    ans.block<1, 3>(0, 1) = -p.vec().transpose();
    ans.block<3, 1>(1, 0) = p.vec();
    ans.block<3, 3>(1, 1) =
        p.w() * Eigen::Matrix3d::Identity() - skewSymmetric(p.vec());
    return ans;
  }
};
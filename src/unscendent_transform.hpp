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
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <algorithm>
#include <iostream>
#include <sophus/se3.hpp>
#include <vector>

static constexpr int dim = 6;
static constexpr int num_sigma_points = 2 * dim + 1;
static constexpr double w = 1.0 / num_sigma_points;

using MatrixType = Eigen::Matrix<double, dim, dim>;
using SigmaPoints = std::vector<Sophus::SE3d>;

SigmaPoints toSigmaPoints(const Sophus::SE3d &mean,
                          const MatrixType &covariance) {
  SigmaPoints sigma_points(num_sigma_points);
  sigma_points[0] = mean;
  Eigen::LLT<MatrixType> chol(covariance);
  const MatrixType L = chol.matrixL();
  auto positive_transform = std::transform(
      L.colwise().begin(), L.colwise().end(), std::next(sigma_points.begin()),
      [&](const auto &column) { return Sophus::SE3d::exp(column) * mean; });
  std::transform(L.colwise().begin(), L.colwise().end(), positive_transform,
                 [&](const auto &column) {
                   return Sophus::SE3d::exp(-1.0 * column) * mean;
                 });
  return sigma_points;
}

std::vector<double>
propagate(const SigmaPoints &sigma_points,
          std::function<double(const Sophus::SE3d &)> callable) {
  std::vector<double> new_sigma_points(sigma_points.size());
  std::transform(sigma_points.cbegin(), sigma_points.cend(),
                 new_sigma_points.begin(), callable);
  return new_sigma_points;
}

double getVariance(const std::vector<double> &sigma_points_single) {
  double mu = 0.0;
  mu = w * std::reduce(sigma_points_single.cbegin(), sigma_points_single.cend(),
                       mu, [](double sum, const double &element) {
                         sum += element;
                         return std::move(sum);
                       });
  auto square = [](const double &x) -> double { return x * x; };
  double variance = 0.0;
  variance = 0.5 * std::reduce(sigma_points_single.cbegin(),
                               sigma_points_single.cend(), variance,
                               [&](double sum, const double &element) {
                                 sum += square(element - mu);
                                 return std::move(sum);
                               });
  return variance;
}

double propagateUscendent(const Sophus::SE3d &mean,
                          const MatrixType &covariance) {
  auto tau = [](const Sophus::SE3d &pose) {
    return pose.translation().norm() +
           2.0 * 100 * std::sin(0.5 * pose.so3().log().norm());
  };
  SigmaPoints sigmas = toSigmaPoints(mean, covariance);
  auto new_sigmas = propagate(sigmas, tau);
  double sigma = getVariance(new_sigmas);
  return sigma;
}

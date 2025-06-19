/******************************************************************************
 * Copyright (c) 2022, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "path_smoother_base/spline_interpolation.h"

#include <chrono>  // NOLINT

#include "glog/logging.h"

namespace path_smoother_base {

void SplineInterpolation::interpolate(
    const std::vector<geometry_msgs::PoseStamped>& path,
    std::vector<geometry_msgs::PoseStamped>* spline) {
  const auto start_timestamp = std::chrono::system_clock::now();

  // setup auxiliary "time grid"
  double tmin = 0.0, tmax = 0.0;
  std::vector<double> T, X, Y;

  for (size_t i = 0; i < path.size(); ++i) {
    X.push_back(path[i].pose.position.x);
    Y.push_back(path[i].pose.position.y);
  }

  createTimeGrid(&T, &tmin, &tmax, X, Y);

  // define a spline for each coordinate x, y
  tk::spline sx, sy;
  sx.set_points(T, X);
  sy.set_points(T, Y);

  spline->clear();
  geometry_msgs::PoseStamped tmp;
  tmp.header = path.front().header;
  // evaluates spline and outputs data to be used with gnuplot
  size_t n = 1000;  // number of grid points to plot the spline
  for (size_t i = 0; i < n; ++i) {
    double t = tmin + static_cast<double>(i) * (tmax - tmin) /
                          static_cast<double>(n - 1);
    tmp.pose.position.x = sx(t);
    tmp.pose.position.y = sy(t);
    spline->push_back(tmp);
  }

  spline->front().pose.orientation = path.front().pose.orientation;
  for (size_t i = 1; i < spline->size(); i++) {
    double dx =
        spline->at(i).pose.position.x - spline->at(i - 1).pose.position.x;
    double dy =
        spline->at(i).pose.position.y - spline->at(i - 1).pose.position.y;
    double yaw = atan2(dy, dx);
    spline->at(i).pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  // printf("Cubic spline interpolation costs = %f ms\n", diff.count() * 1e3);
}

void SplineInterpolation::createTimeGrid(std::vector<double>* T, double* tmin,
                                         double* tmax,
                                         const std::vector<double>& X,
                                         const std::vector<double>& Y) {
  CHECK_EQ(X.size(), Y.size());
  CHECK_GT(X.size(), 2);

  // setup a "time variable" so that we can interpolate x and y
  // coordinates as a function of time: (X(t), Y(t))
  T->resize(X.size());
  T->front() = 0.0;
  for (size_t i = 1; i < T->size(); i++) {
    // time is proportional to the distance, i.e. we go at a const speed
    T->at(i) = T->at(i - 1) + hypot(X[i] - X[i - 1], Y[i] - Y[i - 1]);
  }

  *tmin = T->front() - 0.0;
  *tmax = T->back() + 0.0;
}

}  // namespace path_smoother_base

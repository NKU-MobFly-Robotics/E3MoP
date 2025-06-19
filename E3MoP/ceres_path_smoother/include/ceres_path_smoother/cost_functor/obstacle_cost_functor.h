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

#pragma once

#include <vector>

#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"
#include "path_smoother_base/utils.h"

#include "ceres_path_smoother/utils.h"

namespace ceres_path_smoother {

class ObstacleCostFunctor {
 public:
  ObstacleCostFunctor(const path_smoother_base::EuclidDistGrid& grid,
                      const double inflation_radius_m,
                      const Eigen::Matrix<double, 1, 1>& information)
      : grid_(grid),
        inflation_radius_m_(inflation_radius_m),
        sqrt_information_(information.llt().matrixL()) {}

  template <typename T>
  bool operator()(const T* const x, const T* const y, T* residuals_ptr) const {
    ceres::Grid2D<double, 1, false> ceres_grid_2d(
        grid_.data.data(), 0, grid_.info.size_x, 0, grid_.info.size_y);

    ceres::BiCubicInterpolator<ceres::Grid2D<double, 1, false>> interpolator(
        ceres_grid_2d);

    const T r =
        (*x - T(grid_.info.origin.position.x)) / T(grid_.info.resolution);
    const T c =
        (*y - T(grid_.info.origin.position.y)) / T(grid_.info.resolution);

    Eigen::Map<Eigen::Matrix<T, 1, 1>> residuals_map(residuals_ptr);

    interpolator.Evaluate(r, c, &residuals_map(0));

    residuals_map(0) = Bound(residuals_map(0), T(inflation_radius_m_));

    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    return true;
  }

  static ceres::CostFunction* Create(
      const path_smoother_base::EuclidDistGrid& grid,
      const double inflation_radius_m,
      const Eigen::Matrix<double, 1, 1>& information) {
    return (new ceres::AutoDiffCostFunction<ObstacleCostFunctor, 1, 1, 1>(
        new ObstacleCostFunctor(grid, inflation_radius_m, information)));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  path_smoother_base::EuclidDistGrid grid_;
  double inflation_radius_m_;
  Eigen::Matrix<double, 1, 1> sqrt_information_;
};

}  // namespace ceres_path_smoother

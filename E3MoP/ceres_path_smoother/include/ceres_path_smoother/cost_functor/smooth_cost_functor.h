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

#include "ceres/ceres.h"

namespace ceres_path_smoother {

class SmoothCostFunctor {
 public:
  explicit SmoothCostFunctor(const Eigen::Matrix2d& information)
      : sqrt_information_(information.llt().matrixL()) {}

  template <typename T>
  bool operator()(const T* const x1, const T* const y1, const T* const x2,
                  const T* const y2, const T* const x3, const T* const y3,
                  T* residuals_ptr) const {
    const Eigen::Matrix<T, 2, 1> conf1(*x1, *y1);
    const Eigen::Matrix<T, 2, 1> conf2(*x2, *y2);
    const Eigen::Matrix<T, 2, 1> conf3(*x3, *y3);

    Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals_map(residuals_ptr);

    residuals_map.template head<2>() = conf3 - T(2) * conf2 + conf1;

    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Matrix2d& information) {
    return (
        new ceres::AutoDiffCostFunction<SmoothCostFunctor, 2, 1, 1, 1, 1, 1, 1>(
            new SmoothCostFunctor(information)));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  Eigen::Matrix2d sqrt_information_;
};

}  // namespace ceres_path_smoother

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

class DeviationCostFunctor {
 public:
  DeviationCostFunctor(const Eigen::Matrix2d& information,
                       const Eigen::Vector2d& ref_point)
      : sqrt_information_(information.llt().matrixL()), ref_point_(ref_point) {}

  template <typename T>
  bool operator()(const T* const x, const T* const y, T* residuals_ptr) const {
    const Eigen::Matrix<T, 2, 1> conf(*x, *y);

    Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals_map(residuals_ptr);

    residuals_map.template head<2>() = conf - ref_point_.cast<T>();

    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Matrix2d& information,
                                     const Eigen::Vector2d& ref_point) {
    return (new ceres::AutoDiffCostFunction<DeviationCostFunctor, 2, 1, 1>(
        new DeviationCostFunctor(information, ref_point)));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  Eigen::Matrix2d sqrt_information_;
  Eigen::Vector2d ref_point_;
};

}  // namespace ceres_path_smoother

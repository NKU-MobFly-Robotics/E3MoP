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

#include "costmap_2d/costmap_2d_ros.h"
#include "geometry_msgs/PoseStamped.h"

#include "path_smoother_base/spline_interpolation.h"
#include "path_smoother_base/utils.h"

namespace path_smoother_base {

class PathSmootherBase {
 public:
  PathSmootherBase() = default;
  virtual ~PathSmootherBase() = default;

  virtual bool smooth(
      const std::vector<geometry_msgs::PoseStamped>& initial_path,
      costmap_2d::Costmap2DROS* costmap_2d_ros,
      std::vector<geometry_msgs::PoseStamped>* smoothed_path) = 0;

  virtual bool smooth(
      const std::vector<geometry_msgs::PoseStamped>& initial_path,
      const EuclidDistGrid& grid,
      const std::vector<geometry_msgs::Point>& footprint,
      std::vector<geometry_msgs::PoseStamped>* smoothed_path) = 0;

  void set_min_separation(const double min_separation) {
    min_separation_ = min_separation;
  }

  void set_max_plan_length(const double max_plan_length) {
    max_plan_length_ = max_plan_length;
  }

  double min_separation() const { return min_separation_; }

  double max_plan_length() const { return max_plan_length_; }

 protected:
  double generateRawPoints(
      const std::vector<geometry_msgs::PoseStamped>& initial_path,
      std::vector<geometry_msgs::PoseStamped>* raw_points2d);

 protected:
  SplineInterpolation interpolator_;
  double min_separation_ = 0.1;
  double max_plan_length_ = 10.0;
};

}  // namespace path_smoother_base

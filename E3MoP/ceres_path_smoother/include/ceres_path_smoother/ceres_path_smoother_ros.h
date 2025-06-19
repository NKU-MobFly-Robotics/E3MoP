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

#include <memory>
#include <vector>

#include "costmap_2d/costmap_2d_ros.h"
#include "distance_map_layer/distance_map_layer.h"
#include "geometry_msgs/PoseStamped.h"
#include "path_smoother_base/path_smoother_base.h"

#include "ceres_path_smoother/ceres_path_smoother.h"
#include "ceres_path_smoother/utils.h"

namespace ceres_path_smoother {

class CeresPathSmootherROS : public path_smoother_base::PathSmootherBase {
 public:
  explicit CeresPathSmootherROS(const double inflation_radius_m);
  virtual ~CeresPathSmootherROS() = default;

  bool smooth(const std::vector<geometry_msgs::PoseStamped>& initial_path,
              costmap_2d::Costmap2DROS* costmap_2d_ros,
              std::vector<geometry_msgs::PoseStamped>* smoothed_path) override;

  bool smooth(const std::vector<geometry_msgs::PoseStamped>& initial_path,
              const path_smoother_base::EuclidDistGrid& grid,
              const std::vector<geometry_msgs::Point>& footprint,
              std::vector<geometry_msgs::PoseStamped>* smoothed_path) override;

 private:
  bool checkDistanceMapLayer(costmap_2d::Costmap2DROS* costmap_2d_ros,
                             path_smoother_base::EuclidDistGrid* grid);

  void populateSmoothedPath(
      const std::vector<geometry_msgs::PoseStamped>& raw_points2d,
      const std::vector<double>& opt_x, const std::vector<double>& opt_y,
      std::vector<geometry_msgs::PoseStamped>* smoothed_path);

 private:
  double inflation_radius_m_;
};

}  // namespace ceres_path_smoother

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

#include "ceres_path_smoother/ceres_path_smoother_ros.h"

#include <chrono>  // NOLINT

#include "distance_map_layer/distance_map_layer.h"
#include "glog/logging.h"

FILE* file = fopen("ceres_time.txt", "w+");

namespace ceres_path_smoother {

CeresPathSmootherROS::CeresPathSmootherROS(const double inflation_radius_m)
    : inflation_radius_m_(inflation_radius_m) {}

bool CeresPathSmootherROS::smooth(
    const std::vector<geometry_msgs::PoseStamped>& initial_path,
    costmap_2d::Costmap2DROS* costmap_2d_ros,
    std::vector<geometry_msgs::PoseStamped>* smoothed_path) {
  CHECK(!initial_path.empty());
  CHECK_NOTNULL(costmap_2d_ros);

  path_smoother_base::EuclidDistGrid grid;
  if (!checkDistanceMapLayer(costmap_2d_ros, &grid)) {
    return false;
  }

  const auto start_timestamp = std::chrono::system_clock::now();

  std::vector<geometry_msgs::PoseStamped> raw_points2d;
  generateRawPoints(initial_path, &raw_points2d);

  CeresPathSmoother ceres_path_smoother;
  std::vector<double> opt_x, opt_y;
  if (!ceres_path_smoother.optimize(raw_points2d, grid, inflation_radius_m_,
                                    &opt_x, &opt_y)) {
    return false;
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  ROS_INFO("Time used to smooth path=%.3fms", diff.count() * 1e3);

  fprintf(file, "%.2f\n", diff.count() * 1e3);

  populateSmoothedPath(raw_points2d, opt_x, opt_y, smoothed_path);
  return true;
}

bool CeresPathSmootherROS::smooth(
    const std::vector<geometry_msgs::PoseStamped>& initial_path,
    const path_smoother_base::EuclidDistGrid& grid,
    const std::vector<geometry_msgs::Point>& footprint,
    std::vector<geometry_msgs::PoseStamped>* smoothed_path) {
  CHECK(!initial_path.empty());
  const auto start_timestamp = std::chrono::system_clock::now();

  std::vector<geometry_msgs::PoseStamped> raw_points2d;
  generateRawPoints(initial_path, &raw_points2d);

  CeresPathSmoother ceres_path_smoother;
  std::vector<double> opt_x, opt_y;
  if (!ceres_path_smoother.optimize(raw_points2d, grid, inflation_radius_m_,
                                    &opt_x, &opt_y)) {
    return false;
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  ROS_INFO("Time used to smooth path=%.3fms", diff.count() * 1e3);
  fprintf(file, "%.2f\n", diff.count() * 1e3);
  // std::cout<<"1111 populateSmoothedPath";
  populateSmoothedPath(raw_points2d, opt_x, opt_y, smoothed_path);
  return true;
}

bool CeresPathSmootherROS::checkDistanceMapLayer(
    costmap_2d::Costmap2DROS* costmap_2d_ros,
    path_smoother_base::EuclidDistGrid* grid) {
  const costmap_2d::Costmap2D* costmap_2d = costmap_2d_ros->getCostmap();
  grid->info.size_x = costmap_2d->getSizeInCellsX();
  grid->info.size_y = costmap_2d->getSizeInCellsY();
  grid->info.resolution = costmap_2d->getResolution();
  grid->info.origin.position.x = costmap_2d->getOriginX();
  grid->info.origin.position.y = costmap_2d->getOriginY();

  bool has_distance_map_layer = false;
  // check if the costmap has a distance map layer
  for (auto layer = costmap_2d_ros->getLayeredCostmap()->getPlugins()->begin();
       layer != costmap_2d_ros->getLayeredCostmap()->getPlugins()->end();
       ++layer) {
    boost::shared_ptr<costmap_2d::DistanceMapLayer> distance_map_layer =
        boost::dynamic_pointer_cast<costmap_2d::DistanceMapLayer>(*layer);
    if (distance_map_layer) {
      has_distance_map_layer = true;
      boost::unique_lock<boost::mutex> lock(distance_map_layer->getMutex());
      grid->data = distance_map_layer->getDistmap();
      break;
    }
  }

  if (!has_distance_map_layer) {
    ROS_ERROR("Failed to get a distance map layer for path smoothing");
    return false;
  }
  return true;
}

void CeresPathSmootherROS::populateSmoothedPath(
    const std::vector<geometry_msgs::PoseStamped>& raw_points2d,
    const std::vector<double>& opt_x, const std::vector<double>& opt_y,
    std::vector<geometry_msgs::PoseStamped>* smoothed_path) {
  CHECK_EQ(opt_x.size(), opt_y.size());
  CHECK_GT(opt_x.size(), 2U);
  std::cout<<"2222 populateSmoothedPath";
  geometry_msgs::PoseStamped tmp;
  tmp.header = raw_points2d.front().header;
  smoothed_path->clear();

  for (size_t i = 0; i < opt_x.size(); ++i) {
    tmp.pose.position.x = opt_x[i];
    tmp.pose.position.y = opt_y[i];
    smoothed_path->push_back(tmp);
  }

  smoothed_path->front().pose.orientation =
      raw_points2d.front().pose.orientation;
  // Get finite difference approximated dx and dy for heading calculation
  for (size_t i = 1; i < smoothed_path->size() - 1; i++) {
    double dx = 0.5 * (smoothed_path->at(i + 1).pose.position.x -
                       smoothed_path->at(i - 1).pose.position.x);
    double dy = 0.5 * (smoothed_path->at(i + 1).pose.position.y -
                       smoothed_path->at(i - 1).pose.position.y);
    double yaw = atan2(dy, dx);
    smoothed_path->at(i).pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  }
  smoothed_path->back().pose.orientation = raw_points2d.back().pose.orientation;
}

}  // namespace ceres_path_smoother

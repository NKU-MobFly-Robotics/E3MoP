/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/

#pragma once

#include <string>
#include <vector>

#include <boost/thread.hpp>

#include "costmap_2d/costmap_2d.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_core/base_local_planner.h"
#include "tf2_ros/buffer.h"

namespace path_smoother_base {
/**
 * @class LocalPlannerUtil
 * @brief Helper class implementing infrastructure code many local planner
 * implementations may need.
 */
class LocalPlannerUtil {
 public:
  LocalPlannerUtil() = default;

  virtual ~LocalPlannerUtil() = default;

  void initialize(tf2_ros::Buffer* tf, costmap_2d::Costmap2D* costmap,
                  std::string global_frame);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  bool getLocalPlan(const geometry_msgs::PoseStamped& global_pose,
                    std::vector<geometry_msgs::PoseStamped>* transformed_plan,
                    bool prune_plan, const double dist_behind_robot);

  costmap_2d::Costmap2D* getCostmap();

  const std::string& getGlobalFrame() { return global_frame_; }

 private:
  bool transformGlobalPlan(
      const tf2_ros::Buffer& tf,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const geometry_msgs::PoseStamped& global_pose,
      const costmap_2d::Costmap2D& costmap, const std::string& global_frame,
      std::vector<geometry_msgs::PoseStamped>* transformed_plan);

  void prunePlan(const geometry_msgs::PoseStamped& global_pose,
                 std::vector<geometry_msgs::PoseStamped>* plan,
                 std::vector<geometry_msgs::PoseStamped>* global_plan,
                 const double dist_behind_robot);

 private:
  // things we get from move_base
  std::string name_;
  std::string global_frame_;
  costmap_2d::Costmap2D* costmap_ = nullptr;
  tf2_ros::Buffer* tf_ = nullptr;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  bool initialized_ = false;
};

};  // namespace path_smoother_base

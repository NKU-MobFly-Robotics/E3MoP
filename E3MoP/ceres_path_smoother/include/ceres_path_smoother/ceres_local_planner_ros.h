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
#include <string>
#include <vector>

#include "base_local_planner/goal_functions.h"
#include "geometry_msgs/PoseStamped.h"
#include "lattice_path_planner/lattice_path_planner_ros.h"
#include "nav_core/base_local_planner.h"
#include "nav_msgs/Path.h"
#include "path_smoother_base/local_planner_util.h"
#include "ros/ros.h"
#include "teb_local_planner/teb_local_planner_ros.h"
#include "tf/tf.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"

#include "ceres_path_smoother/ceres_path_smoother_ros.h"

namespace ceres_path_smoother {

class CeresLocalPlannerROS : public nav_core::BaseLocalPlanner {
 public:
  /**
   * @brief Default constructor of the ceres local planner plugin
   */
  CeresLocalPlannerROS() = default;

  /**
   * @brief  Destructor of the plugin
   */
  virtual ~CeresLocalPlannerROS() = default;

  /**
   * @brief Initializes the ceres local planner plugin
   * @param name The name of the instance
   * @param tf Pointer to a tf buffer
   * @param costmap_ros Cost map representing occupied and free space
   */
  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros) override;

  /**
   * @brief Set the plan that the ceres local planner is following
   * @param orig_global_plan The plan to pass to the local planner
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(
      const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) override;

  /**
   * @brief Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base
   * @param cmd_vel Will be filled with the velocity command to be passed to the
   * robot base
   * @return True if a valid trajectory was found, false otherwise
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

  /**
   * @brief  Check if the goal pose has been achieved
   *
   * The actual check is performed in computeVelocityCommands().
   * Only the status flag is checked here.
   * @return True if achieved, false otherwise
   */
  bool isGoalReached() override;

 private:
  // Pointer to the costmap ros wrapper, received from the navigation stack.
  costmap_2d::Costmap2DROS* costmap_ros_;
  // Pointer to the 2d costmap (obtained from the costmap ros wrapper)
  costmap_2d::Costmap2D* costmap_;
  // Pointer to tf buffer.
  tf2_ros::Buffer* tf_;

  std::unique_ptr<path_smoother_base::LocalPlannerUtil> local_planner_util_ =
      nullptr;

  geometry_msgs::PoseStamped global_goal_;
  ros::Publisher local_lattice_plan_pub_;
  ros::Publisher smoothed_plan_pub_;
  bool initialized_ = false;

  std::string global_frame_;      // The frame in which the controller will run
  std::string robot_base_frame_;  // Used as the base frame id of the robot
  std::string name_;              // For use with the ros nodehandle
  std::string odom_topic_;

  std::unique_ptr<CeresPathSmootherROS> path_smoother_ = nullptr;
  std::unique_ptr<lattice_path_planner::LatticePathPlannerROS> path_planner_ =
      nullptr;
  std::unique_ptr<teb_local_planner::TebLocalPlannerROS> velocity_planner_ =
      nullptr;
};

}  // namespace ceres_path_smoother

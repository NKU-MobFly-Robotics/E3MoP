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

#include "ceres_path_smoother/ceres_local_planner_ros.h"

#include <chrono>  // NOLINT

#include "pluginlib/class_list_macros.h"

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ceres_path_smoother::CeresLocalPlannerROS,
                       nav_core::BaseLocalPlanner)

namespace ceres_path_smoother {

void CeresLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf,
                                      costmap_2d::Costmap2DROS* costmap_ros) {
  // check if the plugin is already initialized
  if (!initialized_) {
    name_ = name;
    // create Node Handle with name of plugin (as used in move_base for loading)
    ros::NodeHandle private_nh("~/" + name);

    // init other variables
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    // locking should be done in MoveBase.
    costmap_ = costmap_ros_->getCostmap();

    global_frame_ = costmap_ros_->getGlobalFrameID();
    robot_base_frame_ = costmap_ros_->getBaseFrameID();

    // create and initialize local planner util
    local_planner_util_ =
        std::make_unique<path_smoother_base::LocalPlannerUtil>();
    local_planner_util_->initialize(tf, costmap_, global_frame_);

    path_smoother_ = std::make_unique<CeresPathSmootherROS>(0.55);
    path_smoother_->set_max_plan_length(3.0);
    path_smoother_->set_min_separation(0.05);

    velocity_planner_ =
        std::make_unique<teb_local_planner::TebLocalPlannerROS>();
    velocity_planner_->initialize(name, tf, costmap_ros_);

    path_planner_ =
        std::make_unique<lattice_path_planner::LatticePathPlannerROS>();
    path_planner_->initialize(name, costmap_ros_);

    local_lattice_plan_pub_ =
        private_nh.advertise<nav_msgs::Path>("local_lattice_plan", 1);
    smoothed_plan_pub_ =
        private_nh.advertise<nav_msgs::Path>("smoothed_plan", 1);

    // set initialized flag
    initialized_ = true;

    ROS_DEBUG("ceres_local_planner plugin initialized.");
  } else {
    ROS_WARN(
        "ceres_local_planner has already been initialized, doing nothing.");
  }
}

bool CeresLocalPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  // check if plugin is initialized
  if (!initialized_) {
    ROS_ERROR(
        "ceres_local_planner has not been initialized, please call "
        "initialize() before using this planner");
    return false;
  }

  // store the global plan
  local_planner_util_->setPlan(orig_global_plan);

  global_goal_ = orig_global_plan.back();

  return true;
}

bool CeresLocalPlannerROS::isGoalReached() {
  // check if plugin is initialized
  if (!initialized_) {
    ROS_ERROR(
        "ceres_local_planner has not been initialized, please call "
        "initialize() before using this planner");
    return false;
  }

  // Get current robot pose
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);

  double dx = robot_pose.pose.position.x - global_goal_.pose.position.x;
  double dy = robot_pose.pose.position.y - global_goal_.pose.position.y;
  if (hypot(dx, dy) < 0.2) {
    ROS_INFO("GOAL Reached!");
    return true;
  } else {
    return false;
  }
}

bool CeresLocalPlannerROS::computeVelocityCommands(
    geometry_msgs::Twist& cmd_vel) {
  // check if plugin is initialized
  if (!initialized_) {
    ROS_ERROR(
        "ceres_local_planner has not been initialized, please call "
        "initialize() before using this planner");
    return false;
  }

  // Get current robot pose
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);

  // get global plan within local costmap
  std::vector<geometry_msgs::PoseStamped> transformed_plan, local_lattice_plan,
      smoothed_plan;
  if (!local_planner_util_->getLocalPlan(robot_pose, &transformed_plan, true,
                                         1.0)) {
    ROS_WARN("Failed to get local plan");
    return false;
  }

  if (!path_planner_->makePlan(robot_pose, transformed_plan.back(),
                               local_lattice_plan)) {
    ROS_WARN("Failed to find local lattice plan");
    return false;
  }

  auto start_timestamp = std::chrono::system_clock::now();
  // smooth local plan
  if (!path_smoother_->smooth(local_lattice_plan, costmap_ros_,
                              &smoothed_plan)) {
    ROS_WARN("Failed to smooth local plan");
    return false;
  }
    std::cout<<"smoothed_plan.size():"<<smoothed_plan.size()<<std::endl;

  auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;

  // publish global and smoothed plans
  base_local_planner::publishPlan(local_lattice_plan, local_lattice_plan_pub_);
  base_local_planner::publishPlan(smoothed_plan, smoothed_plan_pub_);

  velocity_planner_->setPlan(smoothed_plan);
  velocity_planner_->computeVelocityCommands(cmd_vel);

  return true;
}

}  // namespace ceres_path_smoother

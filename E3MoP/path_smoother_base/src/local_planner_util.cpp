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

#include "path_smoother_base/local_planner_util.h"

#include "glog/logging.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace path_smoother_base {

void LocalPlannerUtil::initialize(tf2_ros::Buffer* tf,
                                  costmap_2d::Costmap2D* costmap,
                                  std::string global_frame) {
  if (!initialized_) {
    tf_ = tf;
    costmap_ = costmap;
    global_frame_ = global_frame;
    initialized_ = true;
  } else {
    ROS_WARN("Planner utils have already been initialized, doing nothing.");
  }
}

costmap_2d::Costmap2D* LocalPlannerUtil::getCostmap() { return costmap_; }

bool LocalPlannerUtil::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if (!initialized_) {
    ROS_ERROR(
        "Planner utils have not been initialized, please call initialize() "
        "first");
    return false;
  }

  // reset the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;
  return true;
}

bool LocalPlannerUtil::getLocalPlan(
    const geometry_msgs::PoseStamped& global_pose,
    std::vector<geometry_msgs::PoseStamped>* transformed_plan, bool prune_plan,
    const double dist_behind_robot) {
  // get the global plan in our frame
  if (!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_,
                           global_frame_, transformed_plan)) {
    ROS_WARN(
        "Could not transform the global plan to the frame of the controller");
    return false;
  }

  // now we'll prune the plan based on the position of the robot
  if (prune_plan) {
    prunePlan(global_pose, transformed_plan, &global_plan_, dist_behind_robot);
  }
  return true;
}

bool LocalPlannerUtil::transformGlobalPlan(
    const tf2_ros::Buffer& tf,
    const std::vector<geometry_msgs::PoseStamped>& global_plan,
    const geometry_msgs::PoseStamped& global_pose,
    const costmap_2d::Costmap2D& costmap, const std::string& global_frame,
    std::vector<geometry_msgs::PoseStamped>* transformed_plan) {
  transformed_plan->clear();

  if (global_plan.empty()) {
    ROS_ERROR("Received plan with zero length");
    return false;
  }

  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];
  try {
    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped plan_to_global_transform =
        tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id,
                           plan_pose.header.stamp, plan_pose.header.frame_id,
                           ros::Duration(0.5));

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::PoseStamped robot_pose;
    tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

    // we'll discard points on the plan that are outside the local costmap
    double dist_threshold =
        std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                 costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    // just consider 85% of the costmap size to better incorporate point
    // obstacle that are located on the border of the local costmap
    dist_threshold *= 0.85;

    size_t i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 0;

    // we need to loop to a point on the plan that is within a certain distance
    // of the robot
    while (i < global_plan.size()) {
      double x_diff =
          robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff =
          robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (sq_dist <= sq_dist_threshold) {
        break;
      }
      ++i;
    }

    geometry_msgs::PoseStamped newer_pose;

    // now we'll transform until points are outside of our distance threshold
    while (i < global_plan.size() && sq_dist <= sq_dist_threshold) {
      const geometry_msgs::PoseStamped& pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan->push_back(newer_pose);

      double x_diff =
          robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff =
          robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;

      ++i;
    }
  } catch (tf2::LookupException& ex) {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  } catch (tf2::ConnectivityException& ex) {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  } catch (tf2::ExtrapolationException& ex) {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (!global_plan.empty()) {
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n",
                global_frame.c_str(), (unsigned int)global_plan.size(),
                global_plan[0].header.frame_id.c_str());
    }
    return false;
  }
  return true;
}

void LocalPlannerUtil::prunePlan(
    const geometry_msgs::PoseStamped& global_pose,
    std::vector<geometry_msgs::PoseStamped>* plan,
    std::vector<geometry_msgs::PoseStamped>* global_plan,
    double dist_behind_robot) {
  CHECK_GE(global_plan->size(), plan->size());
  std::vector<geometry_msgs::PoseStamped>::iterator it = plan->begin();
  std::vector<geometry_msgs::PoseStamped>::iterator global_it =
      global_plan->begin();
  while (it != plan->end()) {
    const geometry_msgs::PoseStamped& w = *it;
    // Fixed error bound of 2 meters for now. Can reduce to a portion of the map
    // size or based on the resolution
    double x_diff = global_pose.pose.position.x - w.pose.position.x;
    double y_diff = global_pose.pose.position.y - w.pose.position.y;
    double distance_sq = x_diff * x_diff + y_diff * y_diff;
    if (distance_sq < dist_behind_robot) {
      ROS_DEBUG("Nearest waypoint to <%f, %f> is <%f, %f>\n",
                global_pose.pose.position.x, global_pose.pose.position.y,
                w.pose.position.x, w.pose.position.y);
      break;
    }
    it = plan->erase(it);
    global_it = global_plan->erase(global_it);
  }
}

}  // namespace path_smoother_base

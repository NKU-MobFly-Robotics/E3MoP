#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <chrono>
#include <geometry_msgs/Pose.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;

// 修改回调函数参数，将last_pose改为引用
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, double& path_length, geometry_msgs::Pose& last_pose) {
    geometry_msgs::Pose current_pose = msg->pose.pose;
    bool is_initial_position = (last_pose.position.x == 0.0 && last_pose.position.y == 0.0);

    if (!is_initial_position) {
        double dx = current_pose.position.x - last_pose.position.x;
        double dy = current_pose.position.y - last_pose.position.y;
        double segment_length = std::sqrt(dx * dx + dy * dy);
        path_length += segment_length;
    } else {
        ROS_INFO("Initial position set to (%.2f, %.2f)", current_pose.position.x, current_pose.position.y);
    }
    last_pose = current_pose;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigation_goals");

  MoveBaseActionClient ac("move_base", true);

  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  ros::NodeHandle private_nh("~");
  std::string global_frame;
  private_nh.param("global_costmap/global_frame", global_frame, std::string("map"));

  double goal_pose_x, goal_pose_y, goal_pose_a;
  if (!private_nh.getParam("goal_pose_x", goal_pose_x))
  {
    ROS_ERROR("Goal pose is unavailable");
    return -1;
  }
  if (!private_nh.getParam("goal_pose_y", goal_pose_y))
  {
    ROS_ERROR("Goal pose is unavailable");
    return -1;
  }
  if (!private_nh.getParam("goal_pose_a", goal_pose_a))
  {
    ROS_ERROR("Goal pose is unavailable");
    return -1;
  }

  goal.target_pose.header.frame_id = global_frame;
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = goal_pose_x;
  goal.target_pose.pose.position.y = goal_pose_y;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal_pose_a);

  const auto start_t = std::chrono::high_resolution_clock::now();

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // 定义路径长度和上一个位置
  double path_length = 0.0;
  geometry_msgs::Pose last_pose;

  ros::NodeHandle nh;

  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "/RosAria/odom", 10, 
        boost::bind(odomCallback, _1, boost::ref(path_length), boost::ref(last_pose))
  );

  ros::Rate rate(10); // 10Hz
  while (ros::ok() && !ac.getState().isDone()) {
      ros::spinOnce(); // 处理回调
      rate.sleep();
  }

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The base reached the goal successfully");
  else
    ROS_INFO("The base failed to reach the goal for some reason");

  const auto end_t = std::chrono::high_resolution_clock::now();
  const std::chrono::duration<double> timediff = end_t - start_t;
  ROS_INFO("Total runtime=%f secs", timediff.count());
  ROS_INFO("Total path length=%f meters", path_length);

  return 0;
}
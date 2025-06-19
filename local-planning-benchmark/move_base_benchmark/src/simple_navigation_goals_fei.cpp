#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <chrono>
#include <geometry_msgs/Pose.h>
#include <random>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/bind.hpp>
#include <sensor_msgs/JointState.h> // 用于碰撞检测
#include <vector> // 用于存储成功案例的数据
#include <geometry_msgs/PoseWithCovarianceStamped.h> // 新增头文件

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;

nav_msgs::OccupancyGrid current_map;
bool map_received = false;
bool odom_received = false;
geometry_msgs::Pose current_initial_pose;
std::mt19937 gen;
bool collision_occurred = false;
bool robot_stuck = false;
ros::Time last_movement_time;
geometry_msgs::Pose last_pose_for_stuck;

const double STUCK_TIME_THRESHOLD = 8.0;
const double MOVEMENT_THRESHOLD = 0.15;

void odomSyncCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_initial_pose = msg->pose.pose;
    odom_received = true;
}

bool isGoalValid(const geometry_msgs::PoseStamped& goal) {
    if (!map_received) return false;

    double x = goal.pose.position.x;
    double y = goal.pose.position.y;

    if (x < current_map.info.origin.position.x || x > current_map.info.origin.position.x + current_map.info.width * current_map.info.resolution ||
        y < current_map.info.origin.position.y || y > current_map.info.origin.position.y + current_map.info.height * current_map.info.resolution) {
        return false;
    }

    int map_x = (x - current_map.info.origin.position.x) / current_map.info.resolution;
    int map_y = (y - current_map.info.origin.position.y) / current_map.info.resolution;

    if (map_x >= 0 && map_x < current_map.info.width && map_y >= 0 && map_y < current_map.info.height) {
        int index = map_y * current_map.info.width + map_x;
        if (current_map.data[index] == 0) {
            return true;
        }
    }

    return false;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    current_map = *msg;
    map_received = true;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, double& path_length, geometry_msgs::Pose& last_pose, bool& initialized) {
    geometry_msgs::Pose current_pose = msg->pose.pose;
    
    if (!initialized) {
        last_pose = current_pose;
        last_pose_for_stuck = current_pose;
        last_movement_time = ros::Time::now();
        initialized = true;
        return;
    }
    
    double dx = current_pose.position.x - last_pose.position.x;
    double dy = current_pose.position.y - last_pose.position.y;
    path_length += std::hypot(dx, dy);
    last_pose = current_pose;

    double moved_distance = std::hypot(
        current_pose.position.x - last_pose_for_stuck.position.x,
        current_pose.position.y - last_pose_for_stuck.position.y
    );
    
    if (moved_distance > MOVEMENT_THRESHOLD) {
        last_pose_for_stuck = current_pose;
        last_movement_time = ros::Time::now();
    } else {
        if ((ros::Time::now() - last_movement_time).toSec() > STUCK_TIME_THRESHOLD) {
            robot_stuck = true;
            ROS_WARN("Robot stuck detected! No movement for %.1f seconds", 
                    (ros::Time::now() - last_movement_time).toSec());
        }
    }
}

void collisionCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    for (auto effort : msg->effort) {
        if (effort > 5.0) {
            collision_occurred = true;
            ROS_WARN("Collision detected!");
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle nh;
    
    std::vector<double> success_times;
    std::vector<double> success_path_lengths;
    
    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, mapCallback);
    MoveBaseActionClient ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    ros::NodeHandle private_nh("~");
    std::string global_frame;
    private_nh.param("global_costmap/global_frame", global_frame, std::string("map"));

    while (!map_received) {
        ROS_INFO("Waiting for map...");
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    std::random_device rd;
    gen = std::mt19937(rd());
    std::uniform_real_distribution<double> dist(4.2, 7.0);
    std::uniform_real_distribution<double> angle(0.0, 2 * M_PI);

    int num_episodes = 50;
    int current_episode = 0;
    int success_episodes = 0;
    
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    ros::NodeHandle nh_odom;
    ros::Subscriber collision_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, collisionCallback);

    // 主循环部分
while (ros::ok() && current_episode < num_episodes) {
    collision_occurred = false;
    robot_stuck = false;
    bool episode_counted = false;

    // 同步获取初始位置
    odom_received = false;
    ros::Subscriber initial_pose_sub = nh.subscribe<nav_msgs::Odometry>("/RosAria/odom", 1, odomSyncCallback);
    while (!odom_received && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    initial_pose_sub.shutdown();

    ROS_INFO("Current initial pose: (%.2f, %.2f)", current_initial_pose.position.x, current_initial_pose.position.y);

    // 生成目标点
    geometry_msgs::PoseStamped random_goal;
    random_goal.header.frame_id = "map";
    random_goal.header.stamp = ros::Time::now();
    double distance = dist(gen);
    double theta = angle(gen);

    random_goal.pose.position.x = current_initial_pose.position.x + distance * cos(theta);
    random_goal.pose.position.y = current_initial_pose.position.y + distance * sin(theta);
    random_goal.pose.position.z = 0.0;
    random_goal.pose.orientation = tf::createQuaternionMsgFromYaw(angle(gen));

    // 验证距离
    double dx = random_goal.pose.position.x - current_initial_pose.position.x;
    double dy = random_goal.pose.position.y - current_initial_pose.position.y;
    double actual_distance = std::hypot(dx, dy);

    if (actual_distance < 4.2 || actual_distance > 7.0) {
        ROS_WARN("Distance invalid: %.2f m (required 4.2-7.0 m), retrying...", actual_distance);
        continue;
    }

    double goal_theta = angle(gen);
    random_goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_theta);

    // 检查目标点是否有效
    if (isGoalValid(random_goal)) {
        goal.target_pose = random_goal;
        ROS_INFO("Sending goal: (%f, %f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        ac.sendGoal(goal);

        bool reached = false;
        double path_length = 0.0;
        geometry_msgs::Pose last_pose;
        bool initialized = false;
        auto start_t = std::chrono::high_resolution_clock::now();

        ros::Subscriber odom_sub = nh_odom.subscribe<nav_msgs::Odometry>("/RosAria/odom", 10, boost::bind(odomCallback, _1, boost::ref(path_length), boost::ref(last_pose), boost::ref(initialized)));
        ros::Rate rate(10);
        while (ros::ok()) {
            if (collision_occurred || robot_stuck) {
                ac.cancelGoal();
                ROS_WARN("Episode aborted due to %s", collision_occurred ? "collision" : "stagnation");
                break;
            }

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("The base reached the goal successfully");
                reached = true;
                break;
            } else if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED || ac.getState() == actionlib::SimpleClientGoalState::REJECTED) {
                ROS_INFO("The base failed to reach the goal for some reason");
                break;
            }

            ros::spinOnce();
            rate.sleep();
        }

        // 新增随机初始化逻辑
        if (robot_stuck) {
            ROS_WARN("Randomly resetting robot position due to stagnation");
            // 生成随机位置
            std::uniform_real_distribution<double> x_dist(current_map.info.origin.position.x + 1.0, current_map.info.origin.position.x + current_map.info.width * current_map.info.resolution - 1.0);
            std::uniform_real_distribution<double> y_dist(current_map.info.origin.position.y + 1.0, current_map.info.origin.position.y + current_map.info.height * current_map.info.resolution - 1.0);
            double new_x = x_dist(gen);
            double new_y = y_dist(gen);

            // 验证新位置是否有效
            if (isGoalValid(random_goal)) {
                // 发布初始位置
                geometry_msgs::PoseWithCovarianceStamped init_pose;
                init_pose.header.frame_id = "map";
                init_pose.header.stamp = ros::Time::now();
                init_pose.pose.pose.position.x = new_x;
                init_pose.pose.pose.position.y = new_y;
                init_pose.pose.pose.position.z = 0.0;
                init_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle(gen));
                init_pose.pose.covariance[0] = 0.25; // 设置适当协方差
                init_pose.pose.covariance[7] = 0.25;
                init_pose.pose.covariance[35] = 0.06853892326654783;

                ros::Publisher initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
                initial_pose_pub.publish(init_pose);
                ROS_INFO("Reset robot position to: (%f, %f)", new_x, new_y);
            } else {
                ROS_WARN("Generated position invalid, retrying...");
                // 这里可以添加重试逻辑，或者保持原位置
            }
        } else {
            current_episode++;
            episode_counted = true;

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && !collision_occurred) {
                success_episodes++;
                auto end_t = std::chrono::high_resolution_clock::now();
                double duration = std::chrono::duration<double>(end_t - start_t).count();
                success_times.push_back(duration);
                success_path_lengths.push_back(path_length);
                ROS_INFO("Episode %d SUCCESS | Time: %.2fs | Path: %.2fm", current_episode, duration, path_length);
            } else if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED || ac.getState() == actionlib::SimpleClientGoalState::REJECTED) {
                ROS_WARN("Episode %d FAILED", current_episode);
            }
        }

        odom_sub.shutdown();

        if (episode_counted) {
            ROS_INFO("Progress: %d/%d (Success: %d)", current_episode, num_episodes, success_episodes);
        }
    }
}

    ROS_INFO("\n=== Final Statistics ===");
    ROS_INFO("Total episodes: %d", num_episodes);
    ROS_INFO("Success rate: %.1f%%", (success_episodes * 100.0) / num_episodes);
    
    if (!success_times.empty()) {
        double avg_time = std::accumulate(success_times.begin(), success_times.end(), 0.0) / success_times.size();
        double avg_path = std::accumulate(success_path_lengths.begin(), success_path_lengths.end(), 0.0) / success_times.size();
        ROS_INFO("Average time (successful): %.2fs", avg_time);
        ROS_INFO("Average path (successful): %.2fm", avg_path);
    }

    return 0;
}
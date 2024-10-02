#include "../include/follower_robot.h"

void follower::FollowerRobot::set_targets(std::map<int, std::vector<double>> targets){
    follower_targets = targets;
    follower_targets[follower_targets.size()] = initial_pos;
    
};

move_base_msgs::MoveBaseGoal follower::FollowerRobot::fetch_next_goal(){
    move_base_msgs::MoveBaseGoal explorer_goal;
    
    std::vector<double> next_target{follower_targets[target_id]};
    explorer_goal.target_pose.header.frame_id = "map";
    explorer_goal.target_pose.header.stamp = ros::Time::now();
    explorer_goal.target_pose.pose.position.x = next_target[0];
    explorer_goal.target_pose.pose.position.y = next_target[1];
    explorer_goal.target_pose.pose.orientation.w = 1.0;

    return explorer_goal;
}

void follower::FollowerRobot::move_to_target(MoveBaseClient &follower_client){
    if(!goal_sent){
        ROS_INFO("Sending goal for follower");
        follower_client.sendGoal(fetch_next_goal());//this should be sent only once
        goal_sent = true;
    }    
};

bool follower::FollowerRobot::target_reached(MoveBaseClient &follower_client){
    if(follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        return true;
    }
    return false;
};
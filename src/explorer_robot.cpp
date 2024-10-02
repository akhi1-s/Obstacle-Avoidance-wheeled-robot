#include "../include/explorer_robot.h"

void explorer::ExplorerRobot::fetch_exploration_targets(ros::NodeHandle &nh){

    for(int i=1; i<5; i++){
        std::string target_topic{"/aruco_lookup_locations/target_" + std::to_string(i)};
        std::vector<double> target;    
        nh.getParam(target_topic, target);
        targets.push_back(target);
    }
    targets.push_back(initial_pos); 
};

move_base_msgs::MoveBaseGoal explorer::ExplorerRobot::fetch_next_goal(){
    move_base_msgs::MoveBaseGoal explorer_goal;
    
    std::vector<double> next_target{targets[target_id]};
    explorer_goal.target_pose.header.frame_id = "map";
    explorer_goal.target_pose.header.stamp = ros::Time::now();
    explorer_goal.target_pose.pose.position.x = next_target[0];
    explorer_goal.target_pose.pose.position.y = next_target[1];
    explorer_goal.target_pose.pose.orientation.w = 1.0;

    return explorer_goal;
}

void explorer::ExplorerRobot::move_to_target(MoveBaseClient &explorer_client){
    if(!goal_sent){
        ROS_INFO("Sending goal for explorer");
        explorer_client.sendGoal(fetch_next_goal());//this should be sent only once
        goal_sent = true;
    }    
};

bool explorer::ExplorerRobot::target_reached(MoveBaseClient &explorer_client){
    if(explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        return true;
    }
    return false;
};

// function to rotate the robot about z axis
void explorer::ExplorerRobot::start_robot_rotation(ros::Publisher &cmd_publisher){
  
  ROS_INFO("Start Rotating the robot");
  geometry_msgs::Twist twist_msg;
  twist_msg.angular.z = 0.3;
  cmd_publisher.publish(twist_msg);    
}

void explorer::ExplorerRobot::stop_robot_rotation(ros::Publisher &cmd_publisher){
  
  ROS_INFO("Stop Rotating the robot");
  geometry_msgs::Twist twist_msg;
  twist_msg.angular.z = 0;
  cmd_publisher.publish(twist_msg);  
}
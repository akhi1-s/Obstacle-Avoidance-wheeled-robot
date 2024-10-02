/**
 * @file main.cpp
 * @author Vaishanth Ramaraj, Pranav Limbekar, Rohit M Patil
 * @brief Final project for ENPM809Y
 * @version 0.1
 * @date 2021-12-15
 * 
 * @copyright Copyright (c) 2050
 * 
 */
#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <stdlib.h>  
#include <cmath>

#include "../include/explorer_robot.h"
#include "../include/follower_robot.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// global variable
int current_aruco_maker_id{1};
bool aruco_marker_found{false};
bool exploration_completed{false};
bool rescue_completed{false};
bool is_robot_rotating{false};

std::map<int, std::vector<double>> follower_targets;


// for broadcasting the turtlebot tf
void broadcast() {
  //for broadcaster
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  //broadcast the new frame to /tf Topic
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
  transformStamped.child_frame_id = "my_frame";

  transformStamped.transform.translation.x = 0.5;
  transformStamped.transform.translation.y = 0.5;
  transformStamped.transform.translation.z = 0.2;
  transformStamped.transform.rotation.x = 0;
  transformStamped.transform.rotation.y = 0;
  transformStamped.transform.rotation.z = 0;
  transformStamped.transform.rotation.w = 1;

  // ROS_INFO("Broadcasting tf");

  br.sendTransform(transformStamped);
}

// for listening the turtlebot tf
void listen(tf2_ros::Buffer& tfBuffer) {  

  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "my_frame", ros::Time(0), ros::Duration(4.0));
    auto trans_x = transformStamped.transform.translation.x;
    auto trans_y = transformStamped.transform.translation.y;
    auto trans_z = transformStamped.transform.translation.z;
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

// callback function for fiducial transforms
void fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr &fiducial_tf)
{
  
  if (!fiducial_tf->transforms.empty() && is_robot_rotating && !aruco_marker_found) {//check marker is detected
   
    ROS_INFO("Aruco Marker has been detected");
    aruco_marker_found =  true;
    current_aruco_maker_id = fiducial_tf->transforms[0].fiducial_id;

    //broadcaster object
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    //broadcast the new frame to /tf Topic
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame"; //name of the frame
    transformStamped.transform.translation.x = fiducial_tf->transforms[0].transform.translation.x;
    transformStamped.transform.translation.y = fiducial_tf->transforms[0].transform.translation.y;
    transformStamped.transform.translation.z = fiducial_tf->transforms[0].transform.translation.z;
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;

    ROS_INFO_STREAM("Aruco Marker Position in camera_rgb_optical_frame frame: ["
      << transformStamped.transform.translation.x << ","
      << transformStamped.transform.translation.y << ","
      << transformStamped.transform.translation.z << "]"
    );

    br.sendTransform(transformStamped); //broadcast the transform on /tf Topic
  }
}

// listerner function to transform marker pos with respect to /map frame
void tf_listener(tf2_ros::Buffer& tfBuffer){
  geometry_msgs::TransformStamped transformStamped;

  
  std::vector<double> follower_target;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0), ros::Duration(4.0));
    ROS_INFO_STREAM("Aruco Marker in /map frame: ["
    << transformStamped.transform.translation.x << ","
    << transformStamped.transform.translation.y << ","
    << transformStamped.transform.translation.z << "]"
    );

    follower_target.push_back(transformStamped.transform.translation.x - 0.4); // offset of 0.4 is given to the aruco marker frame
    follower_target.push_back(transformStamped.transform.translation.y - 0.4);

    follower_targets[current_aruco_maker_id] = follower_target;
    std::cout << "Adding marker to targets " << current_aruco_maker_id << " ; target size - " << follower_targets.size() << '\n';
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}


int main(int argc, char** argv)
{
  explorer::ExplorerRobot explorer_bot{-4.0, 2.5};
  follower::FollowerRobot follower_bot{-4.0, 3.5};
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  // publishers
  ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 2);
  ros::Publisher follower_cmd_publisher = nh.advertise<geometry_msgs::Twist>("/follower/cmd_vel", 2);
  ros::Subscriber fiducial_tf_subscriber = nh.subscribe<fiducial_msgs::FiducialTransformArray>("/fiducial_transforms", 2, fiducial_callback);

  // fetch all the targets from the parameter server
  explorer_bot.fetch_exploration_targets(nh);

  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);

  // tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);


  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  while (!follower_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for follower");
  }

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);

  while (ros::ok()) {

    if(!exploration_completed){
      explorer_bot.move_to_target(explorer_client);

      if(!explorer_bot.is_robot_rotating && explorer_bot.target_reached(explorer_client)) {
        ROS_INFO("Start robot rotation");
        if(explorer_bot.target_id != (explorer_bot.targets.size()-1)){
          explorer_bot.start_robot_rotation(cmd_publisher); // start rotating the robot
          explorer_bot.is_robot_rotating = true; 
          is_robot_rotating = explorer_bot.is_robot_rotating;  

        } else {
          exploration_completed = true;
          std::cout << follower_targets.size() << '\n';
          follower_bot.set_targets(follower_targets);
          ROS_INFO("Exploration completed");
        }          
      }

      if(explorer_bot.is_robot_rotating && aruco_marker_found) {
        ROS_INFO("Robot rotation done seting target to next goal");
        explorer_bot.stop_robot_rotation(cmd_publisher); // stop the robot rotation
        tf_listener(tfBuffer);

        // change the target id
        if(explorer_bot.target_id != explorer_bot.targets.size()+1){
          explorer_bot.target_id ++;
          explorer_bot.goal_sent = false;
        }   

        explorer_bot.is_robot_rotating = false;
        is_robot_rotating = explorer_bot.is_robot_rotating;
        aruco_marker_found = false;   
      }
    }

    if(exploration_completed && !rescue_completed){
      follower_bot.move_to_target(follower_client);

      if(follower_bot.target_reached(follower_client)) {     
        if(follower_bot.target_id != (follower_bot.follower_targets.size()-1)){
          follower_bot.target_id ++;
          follower_bot.goal_sent = false;

        } else {
          rescue_completed = true;
          ROS_INFO("Rescue completed");          
        }    
      }
    }   

    if(exploration_completed && rescue_completed){
      ros::shutdown();
      ROS_INFO("ROS is shutdown");
    } 

    broadcast();    
    listen(tfBuffer);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
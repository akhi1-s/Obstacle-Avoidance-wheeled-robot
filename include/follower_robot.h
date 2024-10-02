#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <stdlib.h>  
#include <cmath>

namespace follower {

    class FollowerRobot{

        public:

        /**
         * @brief Construct a new Follower Robot object
         * 
         * @param x 
         * @param y 
         */
        FollowerRobot(double x, double y): init_pos_x{x}, init_pos_y{y}{
            initial_pos.push_back(init_pos_x);
            initial_pos.push_back(init_pos_y);
        }

        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

        /**
         * @brief local variables
         * 
         */
        double init_pos_x; 
        double init_pos_y;
        std::vector<double> initial_pos{};
        std::map<int, std::vector<double>> follower_targets;
        int target_id{0};

        bool goal_sent{false};
        bool is_robot_rotating{false};

        tf2_ros::Buffer tfBuffer;

        /**
         * @brief Set the targets for the follower
         * 
         * @param follower_targets 
         */
        void set_targets(std::map<int, std::vector<double>> follower_targets);

        /**
         * @brief to fetch the next goal
         * 
         * @return move_base_msgs::MoveBaseGoal 
         */
        move_base_msgs::MoveBaseGoal fetch_next_goal();

        /**
         * @brief method to move the follower robot to next target
         * 
         * @param follower_client 
         */
        void move_to_target(MoveBaseClient &follower_client);

        /**
         * @brief check if the follower has reached the next target
         * 
         * @param follower_client 
         * @return true 
         * @return false 
         */
        bool target_reached(MoveBaseClient &follower_client);
    };
}
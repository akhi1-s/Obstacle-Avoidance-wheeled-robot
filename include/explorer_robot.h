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

namespace explorer {

    class ExplorerRobot{

        public:

        /**
         * @brief Construct a new Explorer Robot object
         * 
         * @param x 
         * @param y 
         */
        ExplorerRobot(double x, double y): init_pos_x{x}, init_pos_y{y}{
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
        std::vector<std::vector<double>> targets;        
        int target_id{0};

        bool goal_sent{false};
        bool is_robot_rotating{false};
        bool force_stop_rotation{false};
        
        tf2_ros::Buffer tfBuffer;

        /**
         * @brief method to fetch all the targets from parameter server
         * 
         * @param nh 
         */
        void fetch_exploration_targets(ros::NodeHandle &nh);

        /**
         * @brief called by another method to provide the next target
         * 
         * @return move_base_msgs::MoveBaseGoal 
         */
        move_base_msgs::MoveBaseGoal fetch_next_goal();

        /**
         * @brief method to send the the goal using the explorer client
         * 
         * @param explorer_client 
         */
        void move_to_target(MoveBaseClient &explorer_client);

        /**
         * @brief method to check if the robot has reached the target
         * 
         * @param explorer_client 
         * @return true 
         * @return false 
         */
        bool target_reached(MoveBaseClient &explorer_client);

        /**
         * @brief method to start rotating the robot
         * 
         * @param cmd_publisher 
         */
        void start_robot_rotation(ros::Publisher &cmd_publisher);

        /**
         * @brief method to stop rotating the robot
         * 
         * @param cmd_publisher 
         */
        void stop_robot_rotation(ros::Publisher &cmd_publisher);
    };
}
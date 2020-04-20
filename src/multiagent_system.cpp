#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "gazebo_msgs/ModelState.h"

#define M_PI 3.14159265358979323846  

// Global variables
ros::Publisher pub;

double start_x = 0.0, start_y = 0.0, vel, rot,
start_rot = 0, goal_rot = 0;
int count = 0;
bool should_move = true, start;


//////////////////////////////////////////////////////////////
//
// main fn
//
/////////////////////////////////////////////////////////////
int main(int argc, char **argv) {

  // Init the connection with the ROS system
  ros::init(argc, argv, "lab1_node");

  /**
   * NodeHandle is the main access point to communications with
   * the ROS system.
   */
  ros::NodeHandle n;

  /**
   * We need to tell ROS that we are interested in receiving
   * messages that need to be handled by the poseCallback fn.
   */
  //ros::Subscriber sub = n.subscribe("/robot_0/base_pose_ground_truth",
	//			    1000,
		//		    poseCallback);
  /**
   * To tell ROS that we are going to publish velocity (Twist)
   * messages, we use the advertise() fn.
   */
	//TESTA ATT SKICKA ETT geometry_message med twist
  pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",
					  1000);
  // Start the ROS main loop
  ros::Rate loop_rate(100);
  while (ros::ok()) {
	gazebo_msgs::ModelState vel;
	vel.model_name = "turtlebot";
	vel.reference_frame = "turtlebot";
	vel.twist.linear.x = 0.3;
	vel.twist.angular.z = 0.3;
    /**
     * Calls poseCallback once for each iteration.
     */

    ros::spinOnce();
    loop_rate.sleep();
    pub.publish(vel);

  }
  return 0;
}


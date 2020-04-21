#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "gazebo_msgs/ModelState.h"
#include "sensor_msgs/LaserScan.h"

#define M_PI 3.14159265358979323846  

// Global variables
ros::Publisher pub;
ros::Subscriber sensor_sub1, sensor_sub2;

double start_x = 0.0, start_y = 0.0, vel, rot,
start_rot = 0, goal_rot = 0;
int count = 0;
bool should_move = true, start;

bool lock = false;
std::vector<std::vector<float>> distances(2);
std::vector<std::vector<float>> angles(2);

void sensorCallback1(const sensor_msgs::LaserScan::ConstPtr& msg) {
  // Clear data from previous scan reading.
  distances[0].clear();
  angles[0].clear();

  // Store distance readings in class-scope vector.
  for( int i = 0; i < msg->ranges.size(); i++) {
    distances[0].push_back(msg->ranges[i]);
  }

  // Store scan angularities in class-scope vector.
  float angle = msg->angle_min;
  for( ; angle < msg->angle_max; angle += msg->angle_increment) {
    angles[0].push_back(angle);
  }
  for(float &d : distances[0]){
              d = d < 0.08 ? std::numeric_limits<float>::infinity() : d;
      }
}

void sensorCallback2(const sensor_msgs::LaserScan::ConstPtr& msg) {
  // Clear data from previous scan reading.
  distances[1].clear();
  angles[1].clear();

  // Store distance readings in class-scope vector.
  for( int i = 0; i < msg->ranges.size(); i++) {
    distances[1].push_back(msg->ranges[i]);
  }

  // Store scan angularities in class-scope vector.
  float angle = msg->angle_min;
  for( ; angle < msg->angle_max; angle += msg->angle_increment) {
    angles[1].push_back(angle);
  }
  for(float &d : distances[1]){
              d = d < 0.08 ? std::numeric_limits<float>::infinity() : d;
      }
}

float regionDistance(const float &start, const float &stop, int r) {
  int i = 0, j = 0;
  if(angles[r].empty()){ return 8.0;}
  for(auto &angle : angles[r]) {
    if( angle < start ) i++;
    if( angle < stop  ) j++;
  }
  if( i > j )
    return *std::min_element( distances[r].begin() + j, distances[r].begin() + i);
  return *std::min_element( distances[r].begin() + i, distances[r].begin() + j);
}

bool avoid(geometry_msgs::Twist &msg, int r) {
      float minLeft = regionDistance(M_PI/2, 3*M_PI/10, r);
      float minLeftFront = regionDistance(3*M_PI/10, M_PI/10.0, r);
      float minFront = regionDistance(M_PI/10.0, -M_PI/10.0, r);
      float minRightFront = regionDistance(-M_PI/10.0, -3*M_PI/10, r);
      float minRight = regionDistance(-3*M_PI/10, -M_PI/2, r);
      float direction = 0.0;
      if(minFront < 1.0/2) {
          if(minLeftFront < minRightFront)
              direction = -0.1;
          else
              direction = 0.1;

          lock = true;
          ROS_INFO("WALL FRONT");
          ROS_INFO("%f", minFront);
          if(minFront < 0.8/2) {
              if(minFront < 0.6/2) {
                  msg.linear.x = 0.0;
                  msg.angular.z = direction;
                  ROS_INFO("TURN");
              }
              else {
                  msg.linear.x = 0.07;
                  msg.angular.z = direction;
                  ROS_INFO("Move slow while Turn");
              }
          }
          else
          {
              msg.linear.x = 0.1;
              msg.angular.z = direction;
              ROS_INFO("Start Truning");
          }
          msg.angular.z = msg.angular.z*10;
          return false;
      }
      if(minLeftFront < 0.9/2) {
          lock = true;
          ROS_INFO("WALL LEFTFRONT");
          if(minLeftFront < 0.7/2) {
              if(minLeftFront < 0.5/2) {
                  msg.linear.x = 0.0;
                  msg.angular.z = -0.1;
                  ROS_INFO("TURN");
              }
              else {
                  msg.linear.x = 0.07;
                  msg.angular.z = -0.1;
                  ROS_INFO("Move slow while Turn");
              }
          }
          else
          {
              msg.linear.x = 0.1;
              msg.angular.z = -0.1;
              ROS_INFO("Start Truning");
          }
          msg.angular.z = msg.angular.z*10;
          return false;
      }
      if(minRightFront < 0.9/2) {
          lock = true;
          ROS_INFO("WALL RIGHTFRONT");
          if(minRightFront < 0.7/2) {
              if(minRightFront < 0.5/2) {
                  msg.linear.x = 0.0;
                  msg.angular.z = 0.1;
                  ROS_INFO("TURN");
              }
              else {
                  msg.linear.x = 0.07;
                  msg.angular.z = 0.1;
                  ROS_INFO("Move slow while Turn");
              }
          }
          else
          {
              msg.linear.x = 0.1;
              msg.angular.z = 0.1;
              ROS_INFO("Start Truning");
          }
          msg.angular.z = msg.angular.z*10;
          return false;
      }
      if(minLeft < 0.5/2) {
          lock = true;
          ROS_INFO("WALL LEFT");
          msg.linear.x = 0.1;
          msg.angular.z = -0.1;
          ROS_INFO("TURN");
          msg.angular.z = msg.angular.z*10;
          return false;
      }
      if(minRight < 0.5/2) {
          lock = true;
          ROS_INFO("WALL RIGHT");
          msg.linear.x = 0.1;
          msg.angular.z = 0.1;
          ROS_INFO("TURN");
          msg.angular.z = msg.angular.z*10;
          return false;
      }
      lock = false;
      msg.angular.z = msg.angular.z*10;
      return false;
  }

bool chaseCan(geometry_msgs::Twist &msg, int r)
{

    return false;
}

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
  sensor_sub1 = n.subscribe("/robot1/scan", 1000, sensorCallback1);
  sensor_sub2 = n.subscribe("/robot2/scan", 1000, sensorCallback2);
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

        gazebo_msgs::ModelState vel, vel2;
        vel.model_name = "turtlebot3_burger";
        vel.reference_frame = "turtlebot3_burger";
        vel.twist.linear.x = 1.0;
        vel.twist.angular.z = 0.0;
        avoid(vel.twist,0);

        vel2.model_name = "turtlebot3_burger1";
        vel2.reference_frame = "turtlebot3_burger1";
        vel2.twist.linear.x = 1.0;
        vel2.twist.angular.z = 0.0;
        avoid(vel2.twist,1);

    /**
     * Calls poseCallback once for each iteration.
     */

    ros::spinOnce();
    loop_rate.sleep();
    pub.publish(vel);
    pub.publish(vel2);

  }
  return 0;
}


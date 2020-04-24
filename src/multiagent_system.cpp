#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "gazebo_msgs/ModelState.h"
#include "sensor_msgs/LaserScan.h"

#define M_PI 3.14159265358979323846  

// Global variables
ros::Publisher pub;
ros::Subscriber sensor_sub1, sensor_sub1_2;
ros::Subscriber sensor_sub2, sensor_sub2_2;

double start_x = 0.0, start_y = 0.0, vel, rot,
start_rot = 0, goal_rot = 0;
int count = 0;
bool should_move = true, start;

bool canInPlace1 = false;
bool canInPlace2 = false;
bool pushesCan = false;
bool lock = false;
std::vector<std::vector<float>> distances(4);
std::vector<std::vector<float>> angles(4);

void sensorCallback0(const sensor_msgs::LaserScan::ConstPtr& msg) {
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

void sensorCallback0_2(const sensor_msgs::LaserScan::ConstPtr& msg) {
  // Clear data from previous scan reading.
  distances[2].clear();
  angles[2].clear();

  // Store distance readings in class-scope vector.
  for( int i = 0; i < msg->ranges.size(); i++) {
    distances[2].push_back(msg->ranges[i]);
  }

  // Store scan angularities in class-scope vector.
  float angle = msg->angle_min;
  for( ; angle < msg->angle_max; angle += msg->angle_increment) {
    angles[2].push_back(angle);
  }
  for(float &d : distances[2]){
              d = d < 0.08 ? std::numeric_limits<float>::infinity() : d;
      }
}

void sensorCallback1(const sensor_msgs::LaserScan::ConstPtr& msg) {
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

void sensorCallback1_2(const sensor_msgs::LaserScan::ConstPtr& msg) {
  // Clear data from previous scan reading.
  distances[3].clear();
  angles[3].clear();

  // Store distance readings in class-scope vector.
  for( int i = 0; i < msg->ranges.size(); i++) {
    distances[3].push_back(msg->ranges[i]);
  }

  // Store scan angularities in class-scope vector.
  float angle = msg->angle_min;
  for( ; angle < msg->angle_max; angle += msg->angle_increment) {
    angles[3].push_back(angle);
  }
  for(float &d : distances[3]){
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
      float minLeft = regionDistance(M_PI/2.0, 3.0*M_PI/10.0, r);
      float minLeftFront = regionDistance(3.0*M_PI/10.0, M_PI/10.0, r);
      float minFront = regionDistance(M_PI/10.0, -M_PI/10.0, r);
      float minRightFront = regionDistance(-M_PI/10.0, -3.0*M_PI/10.0, r);
      float minRight = regionDistance(-3.0*M_PI/10.0, -M_PI/2.0, r);
      float direction = 0.0;
      if(minFront < 1.0) {
          if(minLeftFront < minRightFront)
              direction = -0.2;
          else
              direction = 0.2;

          lock = true;
          ROS_INFO("WALL FRONT");
          ROS_INFO("%f", minFront);
          if(minFront < 0.8) {
              if(minFront < 0.6) {
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
      if(minLeftFront < 0.9) {
          lock = true;
          ROS_INFO("WALL LEFTFRONT");
          if(minLeftFront < 0.7) {
              if(minLeftFront < 0.5) {
                  msg.linear.x = 0.0;
                  msg.angular.z = -0.2;
                  ROS_INFO("TURN");
              }
              else {
                  msg.linear.x = 0.07;
                  msg.angular.z = -0.2;
                  ROS_INFO("Move slow while Turn");
              }
          }
          else
          {
              msg.linear.x = 0.1;
              msg.angular.z = -0.2;
              ROS_INFO("Start Truning");
          }
          msg.angular.z = msg.angular.z*10;
          return false;
      }
      if(minRightFront < 0.9) {
          lock = true;
          ROS_INFO("WALL RIGHTFRONT");
          if(minRightFront < 0.7) {
              if(minRightFront < 0.5) {
                  msg.linear.x = 0.0;
                  msg.angular.z = 0.2;
                  ROS_INFO("TURN");
              }
              else {
                  msg.linear.x = 0.07;
                  msg.angular.z = 0.2;
                  ROS_INFO("Move slow while Turn");
              }
          }
          else
          {
              msg.linear.x = 0.1;
              msg.angular.z = 0.2;
              ROS_INFO("Start Truning");
          }
          msg.angular.z = msg.angular.z*10;
          return false;
      }
      if(minLeft < 0.5) {
          lock = true;
          ROS_INFO("WALL LEFT");
          msg.linear.x = 0.1;
          msg.angular.z = -0.2;
          ROS_INFO("TURN");
          msg.angular.z = msg.angular.z*10;
          return false;
      }
      if(minRight < 0.5) {
          lock = true;
          ROS_INFO("WALL RIGHT");
          msg.linear.x = 0.1;
          msg.angular.z = 0.2;
          ROS_INFO("TURN");
          msg.angular.z = msg.angular.z*10;
          return false;
      }
      lock = false;
      msg.angular.z = msg.angular.z*10;
      return false;
  }

bool sortCan(int r)
{
    float minLeft = regionDistance(M_PI/2.0, 3.0*M_PI/10.0, r);
    float minLeftFront = regionDistance(3.0*M_PI/10.0, M_PI/10.0, r);
    float minFront = regionDistance(M_PI/10.0, -M_PI/10.0, r);
    float minRightFront = regionDistance(-M_PI/10.0, -3.0*M_PI/10.0, r);
    float minRight = regionDistance(-3.0*M_PI/10.0, -M_PI/2.0, r);

    if(minLeftFront < 0.2 && minRightFront < 0.2)
    {
        ROS_INFO("Can In Place");
       return true;
    }

    if(r == 2 && canInPlace1 && minLeftFront < 1.0 && minRightFront < 1.0)
    {
        ROS_INFO("Backing Away From Can");
        return true;
    }
    if(r == 3 && canInPlace2 && minLeftFront < 1.0 && minRightFront < 1.0)
    {
        ROS_INFO("Backing Away From Can");
        return true;
    }

    return false;
}

bool chaseCan(geometry_msgs::Twist &msg, int r)
{
    float minLeftWall = regionDistance(M_PI/2.0, 3.0*M_PI/10.0, (r-2));
    float minLeftFrontWall = regionDistance(3.0*M_PI/10.0, M_PI/10.0, (r-2));
    float minFrontWall = regionDistance(M_PI/10.0, -M_PI/10.0, (r-2));
    float minRightFrontWall = regionDistance(-M_PI/10.0, -3.0*M_PI/10.0, (r-2));
    float minRightWall = regionDistance(-3.0*M_PI/10.0, -M_PI/2.0, (r-2));
    if(!pushesCan && (minFrontWall < 1.0 || minLeftFrontWall < 1.0 || minRightFrontWall < 1.0))
    {
        ROS_INFO("NO CAN AVOID WALL");
        return false;
    }
    bool chaseLock;
    if(r==2)
        chaseLock = canInPlace1;
    else if(r==3)
        chaseLock = canInPlace2;

    chaseLock = sortCan(r);
    if(chaseLock)
    {
        ROS_INFO("Can in Place ChaseCan");
        msg.linear.x = -1.0;
        msg.angular.z = 0.7;
        return false;
    }
    float minLeft = regionDistance(M_PI/2.0, 3.0*M_PI/10.0, r);
    float minLeftFront = regionDistance(3.0*M_PI/10.0, M_PI/10.0, r);
    float minFront = regionDistance(M_PI/10.0, -M_PI/10.0, r);
    float minRightFront = regionDistance(-M_PI/10.0, -3.0*M_PI/10.0, r);
    float minRight = regionDistance(-3.0*M_PI/10.0, -M_PI/2.0, r);
    float direction = 0.0;

    pushesCan = false;

    if(minFront < 0.5)
    {
        direction = 0.0;
        ROS_INFO("Pushes Can %d", r);
        pushesCan = true;
    }
    else if(minLeftFront < 0.3 && minLeftFront < minFront)
    {
        direction = 3.0;
        msg.linear.x = 0.0;
        ROS_INFO("Targeting Can LeftFront %d", r);
    }
    else if(minRightFront < 0.3 && minRightFront < minFront)
    {
        direction = -3.0;
        msg.linear.x = 0.0;
        ROS_INFO("Targeting Can RightFront %d", r);
    }
    else if(minLeft < 0.3 && minLeftFront < minFront)
    {
        direction = 7.0;
        msg.linear.x = 0.0;
        ROS_INFO("Targeting Can Left %d", r);
    }
    else if(minRight < 0.3 && minRightFront < minFront)
    {
        direction = -7.0;
        msg.linear.x = 0.0;
        ROS_INFO("Targeting Can Right %d", r);
    }

    msg.angular.z = direction;
    return false;
}

void checkDirection(geometry_msgs::Twist &msg, int r)
{
    float minLeft = regionDistance(M_PI/2.0, 3.0*M_PI/10.0, r);
    float minLeftFront = regionDistance(3.0*M_PI/10.0, M_PI/10.0, r);
    float minFront = regionDistance(M_PI/10.0, -M_PI/10.0, r);
    float minRightFront = regionDistance(-M_PI/10.0, -3.0*M_PI/10.0, r);
    float minRight = regionDistance(-3.0*M_PI/10.0, -M_PI/2.0, r);
    float direction = 0.0;

    if(minFront < 6.0 &&
       minFront < minLeftFront &&
       minFront < minRightFront &&
       minFront < minLeft &&
       minFront < minRight)
        ROS_INFO("Front: %f", minFront);

    else if(minLeftFront < 6.0 &&
            minLeftFront < minRightFront &&
            minLeftFront < minLeft &&
            minLeftFront < minRight)
        ROS_INFO("LeftFront: %f", minLeftFront);

    else if(minRightFront < 6.0 &&
            minLeftFront < minLeft &&
            minLeftFront < minRight)
        ROS_INFO("RightFront: %f", minRightFront);

    else if(minLeft < 6.0 &&
            minLeft < minRight)
        ROS_INFO("Left: %f", minLeft);

    else if(minRight < 6.0)
        ROS_INFO("Right: %f", minRight);

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
  sensor_sub1 = n.subscribe("/robot1/scan", 2, sensorCallback0);
  sensor_sub1_2 = n.subscribe("/robot1/scan2", 2, sensorCallback0_2);

  sensor_sub2 = n.subscribe("/robot2/scan", 2, sensorCallback1);
  sensor_sub2_2 = n.subscribe("/robot2/scan2", 2, sensorCallback1_2);
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
        vel.twist.linear.x = 0.5;
        vel.twist.angular.z = 0.4;
        //avoid(vel.twist,0);
        //chaseCan(vel.twist,2);
        checkDirection(vel.twist,2);

        vel2.model_name = "turtlebot3_burger1";
        vel2.reference_frame = "turtlebot3_burger1";
        vel2.twist.linear.x = 0.5;
        vel2.twist.angular.z = 0.4;
        //avoid(vel2.twist,1);
        //chaseCan(vel2.twist,3);

    /**
     * Calls poseCallback once for each iteration.
     */

    ros::spinOnce();
    loop_rate.sleep();
    //pub.publish(vel);
    //pub.publish(vel2);

  }
  return 0;
}


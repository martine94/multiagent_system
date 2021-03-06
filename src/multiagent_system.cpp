#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "gazebo_msgs/ModelState.h"
#include "sensor_msgs/LaserScan.h"

#define M_PI 3.14159265358979323846  

// Global variables
ros::Publisher pub, movement_pub_rob1, movement_pub_rob2;
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
bool backing1 = false;
bool backing2 = false;
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


void checkDirection(std::vector<float>& sensorValues, int r)
{
    float minRight = regionDistance(M_PI*(12.0/8.0), M_PI*(14.0/8.0), r);
    float minRightFront = regionDistance(M_PI*(14.0/8.0), M_PI*(15.5/8.0), r);
    float minFront1 = regionDistance(M_PI*(15.5/8.0), M_PI*(16.0/8.0), r);
    float minFront2 = regionDistance(M_PI*(0.0/8.0), M_PI*(0.5/8.0), r);
    float minLeftFront = regionDistance(M_PI*(0.5/8.0), M_PI*(2.0/8.0), r);
    float minLeft = regionDistance(M_PI*(2.0/8.0), M_PI*(4.0/8.0), r);
    float minBehind = regionDistance(M_PI*(6.0/8.0), M_PI*(10.0/8.0), r);
    float minFront = std::min(minFront1,minFront2);

    sensorValues[0] = minLeft;
    sensorValues[1] = minLeftFront;
    sensorValues[2] = minFront;
    sensorValues[3] = minRightFront;
    sensorValues[4] = minRight;
    sensorValues[5] = minBehind;
/*
    ROS_INFO("Front: %f", minFront);
    ROS_INFO("Front1: %f", minFront1);
    ROS_INFO("Front2: %f", minFront2);
    ROS_INFO("LeftFront: %f", minLeftFront);
    ROS_INFO("RightFront: %f", minRightFront);
    ROS_INFO("Left: %f", minLeft);
    ROS_INFO("Right: %f", minRight);
*/
}


bool avoid(geometry_msgs::Twist &msg, int r) {
    std::vector<float> sensorBox(6);
    checkDirection(sensorBox, r);
    float minLeft = sensorBox[0], minLeftFront = sensorBox[1], minFront = sensorBox[2],
            minRightFront = sensorBox[3], minRight = sensorBox[4];
      float direction = 0.0;
      if(minFront < 0.5) {
          if(minLeftFront < minRightFront)
              direction = -0.3;
          else
              direction = 0.3;

          lock = true;
          //ROS_INFO("WALL FRONT");
          //ROS_INFO("%f", minFront);
          if(minFront < 0.3) {
              if(minFront < 0.2) {
                  msg.linear.x = 0.0;
                  msg.angular.z = direction;
                  //ROS_INFO("TURN");
              }
              else {
                  msg.linear.x = 0.1;
                  msg.angular.z = direction;
                  //ROS_INFO("Move slow while Turn");
              }
          }
          else
          {
              msg.linear.x = 0.1;
              msg.angular.z = direction;
              //ROS_INFO("Start Truning");
          }
          //msg.angular.z = msg.angular.z*10;
          return false;
      }
      if(minLeftFront < 0.4) {
          lock = true;
          //ROS_INFO("WALL LEFTFRONT");
          if(minLeftFront < 0.2) {
              if(minLeftFront < 0.25) {
                  msg.linear.x = 0.0;
                  msg.angular.z = -0.3;
                  //ROS_INFO("TURN");
              }
              else {
                  msg.linear.x = 0.1;
                  msg.angular.z = -0.3;
                  //ROS_INFO("Move slow while Turn");
              }
          }
          else
          {
              msg.linear.x = 0.1;
              msg.angular.z = -0.3;
              //ROS_INFO("Start Truning");
          }
          //msg.angular.z = msg.angular.z*10;
          return false;
      }
      if(minRightFront < 0.4) {
          lock = true;
          //ROS_INFO("WALL RIGHTFRONT");
          if(minRightFront < 0.2) {
              if(minRightFront < 0.5) {
                  msg.linear.x = 0.0;
                  msg.angular.z = 0.3;
                  //ROS_INFO("TURN");
              }
              else {
                  msg.linear.x = 0.1;
                  msg.angular.z = 0.3;
                  //ROS_INFO("Move slow while Turn");
              }
          }
          else
          {
              msg.linear.x = 0.1;
              msg.angular.z = 0.3;
              //ROS_INFO("Start Truning");
          }
          //msg.angular.z = msg.angular.z*10;
          return false;
      }
      if(minLeft < 0.1) {
          lock = true;
          //ROS_INFO("WALL LEFT");
          msg.linear.x = 0.1;
          msg.angular.z = -0.3;
          //ROS_INFO("TURN");
          //msg.angular.z = msg.angular.z*10;
          return false;
      }
      if(minRight < 0.1) {
          lock = true;
          //ROS_INFO("WALL RIGHT");
          msg.linear.x = 0.1;
          msg.angular.z = 0.3;
          //ROS_INFO("TURN");
          //msg.angular.z = msg.angular.z*10;
          return false;
      }
      lock = false;
      //msg.angular.z = msg.angular.z*10;
      return false;
  }

bool sortCan(int r)
{
    std::vector<float> sensorWall(6);
    checkDirection(sensorWall, r);
    float minLeft = sensorWall[0], minLeftFront = sensorWall[1], minFront = sensorWall[2],
            minRightFront = sensorWall[3], minRight = sensorWall[4], minBackWall = sensorWall[5];
    ROS_INFO("SRTCAN");
    ROS_INFO("Front: %f", minFront);
    ROS_INFO("minLeftFront: %f", minLeftFront);
    ROS_INFO("minRightFront: %f", minRightFront);
    //if((minLeft < 0.2 && minRight < 0.2) || (minFront < 0.2 && minRight < 0.2) || (minFront < 0.2 && minLeft < 0.2))
    if(minFront < 0.5 || minLeftFront < 0.5 || minRightFront < 0.5)
    {
        ROS_INFO("Can In Place");
        if(r == 0)
            canInPlace1 = true;
        else if(r == 1)
            canInPlace2 = true;
    }

    if(r == 0 && canInPlace1 && (minFront < 2.0 || minLeftFront < 2.0 || minRightFront < 2.0 ) && minBackWall > 0.3)
    {
        ROS_INFO("Backing Away From Can");
        backing1 = true;
        return true;
    }
    if(r == 1 && canInPlace2 && (minFront < 2.0 || minLeftFront < 2.0 || minRightFront < 2.0) && minBackWall > 0.3)
    {
        ROS_INFO("Backing Away From Can");
        backing2 = true;
        return true;
    }

    canInPlace1 = false;
    canInPlace2 = false;
    backing1 = false;
    backing2 = false;
    return false;
}

bool chaseCan(geometry_msgs::Twist &msg, int r)
{
    float direction = 0.0;
    std::vector<float> sensorWall(6), sensorBox(6);
    checkDirection(sensorBox, r);
    checkDirection(sensorWall, r-2);
    float minLeft = sensorBox[0], minLeftFront = sensorBox[1], minFront = sensorBox[2],
            minRightFront = sensorBox[3], minRight = sensorBox[4];
    float minLeftWall = sensorWall[0], minLeftFrontWall = sensorWall[1], minFrontWall = sensorWall[2],
            minRightFrontWall = sensorWall[3], minRightWall = sensorWall[4];

    /*if(!pushesCan && (minFrontWall < 1.0 || minLeftFrontWall < 1.0 || minRightFrontWall < 1.0))
    {
        ROS_INFO("NO CAN AVOID WALL");
        return false;
    }*/
    bool chaseLock;

    chaseLock = sortCan(r-2);
    if(chaseLock)
    {
        ROS_INFO("Can in Place ChaseCan");
        msg.linear.x = -0.5;
        msg.angular.z = 0.0;
        return false;
    }

    pushesCan = false;

    if(minFront < 0.3 || (minFront < 1.0 && minFront+0.5 < minFrontWall))// && minFront+0.5 < minLeftFront && minFront+0.5 < minRightFront))
    {
        direction = 0.0;
        ROS_INFO("Pushes Can %d", r);
        msg.linear.x = 0.4;
        //pushesCan = true;
    }
    else if(minLeftFront < 1.0 && minLeftFrontWall > minLeftFront+0.5 && minLeftFront < minFront+0.1)
    {
        if(minLeftFront < 0.2)
        {
            direction = 0.0;
            msg.linear.x = -0.4;
        }
        else
        {
            direction = 0.3;
            msg.linear.x = 0.0;
            ROS_INFO("Targeting Can LeftFront %d", r);
        }
    }
    else if(minRightFront < 1.0 && minRightFrontWall > minRightFront+0.5 && minRightFront < minFront+0.1)
    {
        if(minRightFront < 0.2)
        {
            direction = 0.0;
            msg.linear.x = -0.4;
        }
        else
        {
            direction = -0.3;
            msg.linear.x = 0.0;
            ROS_INFO("Targeting Can RightFront %d", r);
        }
    }
    else if(minLeft < 1.0 && minLeftWall > minLeft+0.5)
    {
        if(minLeft < 0.2)
        {
            direction = 0.0;
            msg.linear.x = -0.4;
        }
        else
        {
            direction = 0.5;
            msg.linear.x = 0.0;
            ROS_INFO("Targeting Can Left %d", r);
        }
    }
    else if(minRight < 1.0 && minRightWall > minRight+0.5)
    {
        if(minRight < 0.2)
        {
            direction = 0.0;
            msg.linear.x = -0.4;
        }
        else
        {
            direction = -0.5;
            msg.linear.x = 0.0;
            ROS_INFO("Targeting Can Right %d", r);
        }
    }
    else
        direction = msg.angular.z;

    msg.angular.z = direction;
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
                                          3);

  movement_pub_rob1 = n.advertise<geometry_msgs::Twist>("/robot1/cmd_vel", 3);

  movement_pub_rob2 = n.advertise<geometry_msgs::Twist>("/robot2/cmd_vel", 3);

  // Start the ROS main loop
  ros::Rate loop_rate(100);
  while (ros::ok()) {

        gazebo_msgs::ModelState vel, vel2;
        geometry_msgs::Twist rob1, rob2;
        vel.model_name = "turtlebot3_burger";
        vel.reference_frame = "turtlebot3_burger";
        //vel.twist.linear.x = 0.5;
        //vel.twist.angular.z = 0.4;

        rob1.linear.x = 0.4;
        rob1.angular.z = 0.3;
        chaseCan(rob1,2);
        if(!backing1)
            avoid(rob1,0);

        vel2.model_name = "turtlebot3_burger1";
        vel2.reference_frame = "turtlebot3_burger1";
        rob2.linear.x = 0.4;
        rob2.angular.z = 0.3;
        chaseCan(rob2,3);
        if(!backing2)
            avoid(rob2,1);
    /**
     * Calls poseCallback once for each iteration.
     */
    ROS_INFO("Linear: %f", rob1.linear.x);
    ROS_INFO("Angular: %f", rob1.angular.z);
    movement_pub_rob1.publish(rob1);
    movement_pub_rob2.publish(rob2);
    ros::spinOnce();
    loop_rate.sleep();

    //pub.publish(vel);
    //pub.publish(vel2);

  }
  return 0;
}


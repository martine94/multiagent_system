#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "gazebo_msgs/ModelState.h"
#include "sensor_msgs/LaserScan.h"

#define M_PI 3.14159265358979323846
#define SensorWall 0
#define SensorBox 1

class agent
{
public:
    agent() {}
    void init(std::string topicNameSub, std::string topicNamePub);
    void run();

protected:
    void sensorCallbackWall(const sensor_msgs::LaserScan::ConstPtr& msg);
    void sensorCallbackBox(const sensor_msgs::LaserScan::ConstPtr& msg);
    float regionDistance(const float &start, const float &stop, int r);
    void checkDirection(std::vector<float>& sensorValues, int r);
    bool avoid(geometry_msgs::Twist &msg, int r);
    bool sortCan(int r);
    bool chaseCan(geometry_msgs::Twist &msg);


    ros::Publisher velPub;
    ros::Subscriber wallLaserSub, boxLaserSub;
    ros::NodeHandle n;

    bool canInPlace = false;
    bool looked = false;
    bool backing = false;

    std::vector<std::vector<float>> distances;
    std::vector<std::vector<float>> angles;

    geometry_msgs::Twist robotVel;
};



void agent::init(std::string topicNameSub, std::string topicNamePub)
{
    distances.resize(2);
    angles.resize(2);
    std::string boxSub = topicNameSub+std::to_string(2);

    wallLaserSub = n.subscribe(topicNameSub, 2, &agent::sensorCallbackWall,this);
    boxLaserSub = n.subscribe(boxSub, 2, &agent::sensorCallbackBox,this);
    velPub = n.advertise<geometry_msgs::Twist>(topicNamePub, 3);

}



void agent::sensorCallbackWall(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // Clear data from previous scan reading.
    this->distances[SensorWall].clear();
    this->angles[SensorWall].clear();

    // Store distance readings in class-scope vector.
    for( int i = 0; i < msg->ranges.size(); i++)
      this->distances[SensorWall].push_back(msg->ranges[i]);


    // Store scan angularities in class-scope vector.
    float angle = msg->angle_min;

    for( ; angle < msg->angle_max; angle += msg->angle_increment)
      this->angles[SensorWall].push_back(angle);

    for(float &d : this->distances[SensorWall])
        d = d < 0.08 ? std::numeric_limits<float>::infinity() : d;

}



void agent::sensorCallbackBox(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // Clear data from previous scan reading.
    this->distances[SensorBox].clear();
    this->angles[SensorBox].clear();

    // Store distance readings in class-scope vector.
    for( int i = 0; i < msg->ranges.size(); i++)
      this->distances[SensorBox].push_back(msg->ranges[i]);

    // Store scan angularities in class-scope vector.
    float angle = msg->angle_min;

    for( ; angle < msg->angle_max; angle += msg->angle_increment)
      this->angles[SensorBox].push_back(angle);

    for(float &d : this->distances[SensorBox])
        d = d < 0.08 ? std::numeric_limits<float>::infinity() : d;

}



float agent::regionDistance(const float &start, const float &stop, int r)
{
    int i = 0, j = 0;

    if(this->angles[r].empty())
        return 8.0;

    for(auto &angle : this->angles[r]) {
      if( angle < start ) i++;
      if( angle < stop  ) j++;
    }

    if( i > j )
      return *std::min_element( this->distances[r].begin() + j, this->distances[r].begin() + i);
    return *std::min_element( this->distances[r].begin() + i, this->distances[r].begin() + j);

}



void agent::checkDirection(std::vector<float>& sensorValues, int r)
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
}



bool agent::avoid(geometry_msgs::Twist &msg, int r)
{
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

        this->looked = true;
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
        this->looked = true;
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
        this->looked = true;
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
        this->looked = true;
        //ROS_INFO("WALL LEFT");
        msg.linear.x = 0.1;
        msg.angular.z = -0.3;
        //ROS_INFO("TURN");
        //msg.angular.z = msg.angular.z*10;
        return false;
    }
    if(minRight < 0.1) {
        this->looked = true;
        //ROS_INFO("WALL RIGHT");
        msg.linear.x = 0.1;
        msg.angular.z = 0.3;
        //ROS_INFO("TURN");
        //msg.angular.z = msg.angular.z*10;
        return false;
    }
    this->looked = false;
    //msg.angular.z = msg.angular.z*10;
    return false;
}



bool agent::sortCan(int r)
{
    std::vector<float> sensorWall(6);
    checkDirection(sensorWall, SensorWall);
    float minLeftWall = sensorWall[0], minLeftFrontWall = sensorWall[1], minFrontWall = sensorWall[2],
            minRightFrontWall = sensorWall[3], minRightWall = sensorWall[4], minBackWall = sensorWall[5];
    ROS_INFO("SRTCAN");
    ROS_INFO("FrontWall: %f", minFrontWall);
    ROS_INFO("minLeftFrontWall: %f", minLeftFrontWall);
    ROS_INFO("minRightFrontWall: %f", minRightFrontWall);

    if(minFrontWall < 0.5 || minLeftFrontWall < 0.5 || minRightFrontWall < 0.5)
    {
        ROS_INFO("Can In Place");
        this->canInPlace = true;
    }

    if(canInPlace && (minFrontWall < 2.0 || minLeftFrontWall < 2.0 || minRightFrontWall < 2.0 ) && minBackWall > 0.3)
    {
        ROS_INFO("Backing Away From Can");
        this->backing = true;
        return true;
    }

    this->canInPlace = false;
    this->backing = false;
    return false;
}



bool agent::chaseCan(geometry_msgs::Twist &msg)
{
    float direction = 0.0;
    std::vector<float> sensorWall(6), sensorBox(6);
    checkDirection(sensorBox, SensorBox);
    checkDirection(sensorWall, SensorWall);
    float minLeft = sensorBox[0], minLeftFront = sensorBox[1], minFront = sensorBox[2],
            minRightFront = sensorBox[3], minRight = sensorBox[4];
    float minLeftWall = sensorWall[0], minLeftFrontWall = sensorWall[1], minFrontWall = sensorWall[2],
            minRightFrontWall = sensorWall[3], minRightWall = sensorWall[4];

    bool chaseLock;

    chaseLock = sortCan(SensorWall);
    if(chaseLock)
    {
        ROS_INFO("Can in Place ChaseCan");
        msg.linear.x = -0.5;
        msg.angular.z = 0.0;
        return false;
    }

    if(minFront < 0.3 || (minFront < 1.0 && minFront+0.5 < minFrontWall))// && minFront+0.5 < minLeftFront && minFront+0.5 < minRightFront))
    {
        direction = 0.0;
        ROS_INFO("Pushes Can");
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
            ROS_INFO("Targeting Can LeftFront");
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
            ROS_INFO("Targeting Can RightFront");
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
            ROS_INFO("Targeting Can Left");
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
            ROS_INFO("Targeting Can Right");
        }
    }
    else
        direction = msg.angular.z;

    msg.angular.z = direction;
    return false;
}



void agent::run()
{
    this->robotVel.linear.x =0.4;
    this->robotVel.angular.z = 0.3;
    this->chaseCan(this->robotVel);
    if(!this->backing)
        this->avoid(this->robotVel,0);

    this->velPub.publish(this->robotVel);
}


int main(int argc, char **argv) {

  // Init the connection with the ROS system
  ros::init(argc, argv, "lab1_node");
  agent Robot1, Robot2, Robot3;
  Robot1.init("/robot1/scan", "/robot1/cmd_vel");
  Robot2.init("/robot2/scan", "/robot2/cmd_vel");
  Robot3.init("/robot3/scan", "/robot3/cmd_vel");

  // Start the ROS main loop
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    Robot1.run();
    Robot2.run();
    Robot3.run();



    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

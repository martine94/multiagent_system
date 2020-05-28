#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "gazebo_msgs/ModelState.h"
#include "sensor_msgs/LaserScan.h"
#include "multiagent_system/boxesList.h"
#include "multiagent_system/auction.h"
#include "multiagent_system/jobStatus.h"
#include<random>
#include<ctime>
#include<time.h>

#include <math.h>
#include <algorithm>

#define M_PI 3.14159265358979323846
#define SensorWall 0
#define SensorBox 1
#define angleIncrement 0.0175019223243
#define DoNotDriveValue 1000.0
#define MAXSPEED 5.0f
#define MINSPEED 0.1f
#define BoxTakenSize 0.17f

class Agent
{
public:
    Agent() {}
    void init(std::string topicName);
    void run();


    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void sensorCallbackWall(const sensor_msgs::LaserScan::ConstPtr& msg);
    void sensorCallbackBox(const sensor_msgs::LaserScan::ConstPtr& msg);
    float regionDistance(const float &start, const float &stop, int r);
    void checkDirection(std::vector<float>& sensorValues, int r);
    bool avoid(geometry_msgs::Twist &msg, int r);
    bool sortCan(int r);
    bool chaseCan(geometry_msgs::Twist &msg);
    void locateBox();
    geometry_msgs::Pose getPose(){return this->robotPosition;}


    ros::Publisher velPub;
    ros::Subscriber wallLaserSub, boxLaserSub, robotPose;
    ros::NodeHandle n;

    bool canInPlace = false;
    bool looked = false;
    bool backing = false;

    std::vector<std::vector<float>> distances;
    std::vector<std::vector<float>> angles;

    geometry_msgs::Twist robotVel;
    geometry_msgs::Pose robotPosition;
    geometry_msgs::Point boxPosition;
};

class Piccolos : Agent
{
public:
    void legolasCallbackLocation(const multiagent_system::boxesList::ConstPtr& msg);
    void reciveCommandCallback(const multiagent_system::auction::ConstPtr& msg);               //reciveMsg
    void initPiccolos(std::string topicNameSub, std::string topicNamePub, int rID);
    void springPiccolo();
    void work9To5();
    void reset();
    void executeCommandCallback();
    // Node handler publisher and subscriber
    ros::Publisher bidPub, jobPub;                                                              //bidMsg
    ros::Subscriber boxLocationSub, reciveCommandedSub;
    //ros::ServiceServer service;

    multiagent_system::jobStatus jobReport;

    int robotID;
    bool pushingBox = false;
    bool working = false;
    bool breakTime = false;
    bool noTurning = false;

    time_t now, end;
    //struct tm end;
};

class Legolas : Agent
{
public:
    void locateBox();
    void initLegolas(Piccolos slave1, Piccolos slave2, Piccolos slave3);
    void dealer(multiagent_system::auction &Picco1, multiagent_system::auction &Picco2);              //dealerMsg
    void giveCommand(const multiagent_system::auction& Picco);                                  //reciveMsg
    void springLegolas();
    void bidCallback(const multiagent_system::auction::ConstPtr& robotmsg);                              //bidMsg
    void jobStatusCallback(const multiagent_system::jobStatus::ConstPtr& msg);

    // Node handler publisher and subscriber
    ros::Publisher boxLocationPub, currPicc, auctionBoxesPub;                                            //reciveMsg
    ros::Subscriber bidSub, jobSub;

    multiagent_system::auction Picco1, Picco2, Picco3;

    geometry_msgs::Point P[4];

    struct PiccoloChord
    {
        int succesful = 0;
        int failed = 0;
    } PiccoloRobot1, PiccoloRobot2, PiccoloRobot3;
};



void Agent::init(std::string topicName)
{
    distances.resize(2);
    angles.resize(2);
    std::string velocityPubName = topicName+"/cmd_vel";
    std::string odomSub = topicName+"/odom";
    std::string wallSub = topicName+"/scan";
    std::string boxSub = wallSub+std::to_string(2);

    robotPose = n.subscribe(odomSub, 2, &Agent::poseCallback,this);
    wallLaserSub = n.subscribe(wallSub, 2, &Agent::sensorCallbackWall,this);
    boxLaserSub = n.subscribe(boxSub, 2, &Agent::sensorCallbackBox,this);
    velPub = n.advertise<geometry_msgs::Twist>(velocityPubName, 3);

}



void Agent::poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
// Robot pose (odometry)
        geometry_msgs::Pose robot_pose = msg->pose.pose;
        this->robotPosition = robot_pose;
}



void Agent::sensorCallbackWall(const sensor_msgs::LaserScan::ConstPtr& msg)
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



void Agent::sensorCallbackBox(const sensor_msgs::LaserScan::ConstPtr& msg)
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



float Agent::regionDistance(const float &start, const float &stop, int r)
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



void Agent::checkDirection(std::vector<float>& sensorValues, int r)
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



bool Agent::avoid(geometry_msgs::Twist &msg, int r)
{
    std::vector<float> sensorBox(6);
    checkDirection(sensorBox, r);
    float minLeft = sensorBox[0], minLeftFront = sensorBox[1], minFront = sensorBox[2],
            minRightFront = sensorBox[3], minRight = sensorBox[4];
    float direction = 0.0;

    if(minFront < 0.5) {
        if(minLeftFront < minRightFront)
            direction = -0.6;
        else
            direction = 0.6;

        this->looked = true;
        ////ROS_INFO("WALL FRONT");
        ////ROS_INFO("%f", minFront);
        if(minFront < 0.3) {
            if(minFront < 0.2) {
                msg.linear.x = 0.0;
                msg.angular.z = direction;
                ////ROS_INFO("TURN");
            }
            else {
                msg.linear.x = 0.1;
                msg.angular.z = direction;
                ////ROS_INFO("Move slow while Turn");
            }
        }
        else
        {
            msg.linear.x = 0.1;
            msg.angular.z = direction;
            ////ROS_INFO("Start Truning");
        }
        //msg.angular.z = msg.angular.z*10;
        return false;
    }
    if(minLeftFront < 0.4) {
        this->looked = true;
        ////ROS_INFO("WALL LEFTFRONT");
        if(minLeftFront < 0.2) {
            if(minLeftFront < 0.25) {
                msg.linear.x = 0.0;
                msg.angular.z = -0.6;
                ////ROS_INFO("TURN");
            }
            else {
                msg.linear.x = 0.1;
                msg.angular.z = -0.6;
                ////ROS_INFO("Move slow while Turn");
            }
        }
        else
        {
            msg.linear.x = 0.1;
            msg.angular.z = -0.6;
            ////ROS_INFO("Start Truning");
        }
        //msg.angular.z = msg.angular.z*10;
        return false;
    }
    if(minRightFront < 0.4) {
        this->looked = true;
        ////ROS_INFO("WALL RIGHTFRONT");
        if(minRightFront < 0.2) {
            if(minRightFront < 0.5) {
                msg.linear.x = 0.0;
                msg.angular.z = 0.6;
                ////ROS_INFO("TURN");
            }
            else {
                msg.linear.x = 0.1;
                msg.angular.z = 0.6;
                ////ROS_INFO("Move slow while Turn");
            }
        }
        else
        {
            msg.linear.x = 0.1;
            msg.angular.z = 0.6;
            ////ROS_INFO("Start Truning");
        }
        //msg.angular.z = msg.angular.z*10;
        return false;
    }
    if(minLeft < 0.2) {
        this->looked = true;
        ////ROS_INFO("WALL LEFT");
        msg.linear.x = 0.1;
        msg.angular.z = -0.6;
        ////ROS_INFO("TURN");
        //msg.angular.z = msg.angular.z*10;
        return false;
    }
    if(minRight < 0.2) {
        this->looked = true;
        ////ROS_INFO("WALL RIGHT");
        msg.linear.x = 0.1;
        msg.angular.z = 0.6;
        ////ROS_INFO("TURN");
        //msg.angular.z = msg.angular.z*10;
        return false;
    }
    this->looked = false;

    msg.linear.x = 0.3;
    msg.angular.z = 0.0;
    //msg.angular.z = msg.angular.z*10;
    return false;
}



bool Agent::sortCan(int r)
{
    std::vector<float> sensorWall(6);
    checkDirection(sensorWall, SensorWall);
    float minLeftWall = sensorWall[0], minLeftFrontWall = sensorWall[1], minFrontWall = sensorWall[2],
            minRightFrontWall = sensorWall[3], minRightWall = sensorWall[4], minBackWall = sensorWall[5];
    //ROS_INFO("SRTCAN");
    //ROS_INFO("FrontWall: %f", minFrontWall);
    //ROS_INFO("minLeftFrontWall: %f", minLeftFrontWall);
    //ROS_INFO("minRightFrontWall: %f", minRightFrontWall);

    if(minFrontWall < 0.5 || minLeftFrontWall < 0.5 || minRightFrontWall < 0.5)
    {
        //ROS_INFO("Can In Place");
        this->canInPlace = true;
    }

    if(canInPlace && (minFrontWall < 2.0 || minLeftFrontWall < 2.0 || minRightFrontWall < 2.0 ) && minBackWall > 0.3)
    {
        //ROS_INFO("Backing Away From Can");
        this->backing = true;
        return true;
    }

    this->canInPlace = false;
    this->backing = false;
    return false;
}



bool Agent::chaseCan(geometry_msgs::Twist &msg)
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
        //ROS_INFO("Can in Place ChaseCan");
        msg.linear.x = -0.3;
        msg.angular.z = 0.0;
        return false;
    }

    if(minFront < 0.3 || (minFront < 1.0 && minFront+0.5 < minFrontWall))// && minFront+0.5 < minLeftFront && minFront+0.5 < minRightFront))
    {
        direction = 0.0;
        //ROS_INFO("Pushes Can");
        msg.linear.x = 0.2;
        //pushesCan = true;
    }
    else if(minLeftFront < 1.0 && minLeftFrontWall > minLeftFront+0.5 && minLeftFront < minFront+0.1)
    {
        if(minLeftFront < 0.2)
        {
            direction = 0.0;
            msg.linear.x = -0.2;
        }
        else
        {
            direction = 0.3;
            msg.linear.x = 0.0;
            //ROS_INFO("Targeting Can LeftFront");
        }
    }
    else if(minRightFront < 1.0 && minRightFrontWall > minRightFront+0.5 && minRightFront < minFront+0.1)
    {
        if(minRightFront < 0.2)
        {
            direction = 0.0;
            msg.linear.x = -0.2;
        }
        else
        {
            direction = -0.1;
            msg.linear.x = 0.0;
            //ROS_INFO("Targeting Can RightFront");
        }
    }
    else if(minLeft < 1.0 && minLeftWall > minLeft+0.5)
    {
        if(minLeft < 0.2)
        {
            direction = 0.0;
            msg.linear.x = -0.2;
        }
        else
        {
            direction = 0.3;
            msg.linear.x = 0.0;
            //ROS_INFO("Targeting Can Left");
        }
    }
    else if(minRight < 1.0 && minRightWall > minRight+0.5)
    {
        if(minRight < 0.2)
        {
            direction = 0.0;
            msg.linear.x = -0.2;
        }
        else
        {
            direction = -0.3;
            msg.linear.x = 0.0;
            //ROS_INFO("Targeting Can Right");
        }
    }
    else
        direction = msg.angular.z;

    msg.angular.z = direction;
    return false;
}



void Legolas::initLegolas(Piccolos slave1, Piccolos slave2, Piccolos slave3)
{
    this->init("/Legolas");
    bidSub = n.subscribe("/Piccolo/bids/", 0, &Legolas::bidCallback,this);
    boxLocationPub = n.advertise<multiagent_system::boxesList>("/Legolas/BoxLocation/", 3);
    jobSub = n.subscribe("/PiccoloStatus/", 0, &Legolas::jobStatusCallback,this);
    /*this->Picco1 = slave1.getPose();
    this->Picco2 = slave1.getPose();
    this->Picco3 = slave1.getPose();*/
}



void Legolas::bidCallback(const multiagent_system::auction::ConstPtr &robotmsg)  //bidMsg
{
    ////ROS_INFO("bidCallback 1");
    if(robotmsg->rID == 1)
        this->Picco1 = *robotmsg;
    else if(robotmsg->rID == 2)
        this->Picco2 = *robotmsg;
    else if(robotmsg->rID == 3)
        this->Picco3 = *robotmsg;
    ////ROS_INFO("bidCallback 2");
}



void Legolas::jobStatusCallback(const multiagent_system::jobStatus::ConstPtr& msg)
{
    if(msg->rID == 1)
        if(msg->success == true)
            this->PiccoloRobot1.succesful += 1;
        else
            this->PiccoloRobot1.failed += 1;
    else if(msg->rID == 2)
        if(msg->success == true)
            this->PiccoloRobot2.succesful += 1;
        else
            this->PiccoloRobot2.failed += 1;
    else if(msg->rID == 3)
        if(msg->success == true)
            this->PiccoloRobot3.succesful += 1;
        else
            this->PiccoloRobot3.failed += 1;
}



void Legolas::locateBox()
{
    //ROS_INFO("locateBox 1");
    int j = 0;
    float t = tf::getYaw(this->robotPosition.orientation);

    if(t != 0.0)
    {
    std::vector<float> items;
    multiagent_system::boxesList foundBoxes;

    for(float distance:distances[SensorBox])
    {
        if (distance < 0.7)
         {

            if(distances[SensorWall].size() == 360 && fabs(distances[SensorWall][j] - distance) > 0.3)
            {
                float x, y, xPrim, yPrim;
                xPrim = (cos(j*angleIncrement))*distance;
                yPrim = (sin(j*angleIncrement))*distance;
                x = (float)((xPrim*cos(t)) + (-yPrim*sin(t)) + (this->robotPosition.position.x));
                y = (float)((xPrim*sin(t)) + (yPrim*cos(t)) + (this->robotPosition.position.y));
                if(items.size() > 0 && fabs(items[items.size()-2] - x) < BoxTakenSize
                                    && fabs(items[items.size()-1] - y) < BoxTakenSize)
                                    //&& x-xPrim < 0.05 && y-yPrim < 0.05)
                {

                }
                else
                {
                    items.push_back(x);
                    items.push_back(y);
                    //ROS_INFO("Found Box at: %f, %f", x, y );
                }
            }
         }
        j++;
    }
    foundBoxes.data = items;
    //for(int i = 0; i < items.size(); i+=2)
    //    ROS_INFO("Element%d: %.2f, %.2f",i, items[i], items[i+1]);
    //ROS_INFO("-----------------------");
    if(!items.empty())
        if(fabs(items[0]) > 0.01 && fabs(items[1]) > 0.01)
            this->boxLocationPub.publish(foundBoxes);
    }
    //ROS_INFO("locateBox 2");
}

void Legolas::dealer(multiagent_system::auction &Picco1, multiagent_system::auction &Picco2)  //dealerMsg
{
    //ROS_INFO("dealer 1");
    if(fabs(Picco1.Position.position.x - Picco2.Position.position.x) < BoxTakenSize && fabs(Picco1.Position.position.y - Picco2.Position.position.y) < BoxTakenSize)
        //if(Picco1.Score == std::min(Picco1.Score, Picco2.Score))
            //Picco2.Score = DoNotDriveValue;
        if(Picco2.Score == std::min(Picco1.Score,Picco2.Score))
            Picco1.Score = DoNotDriveValue;
    if(Picco1.Score == DoNotDriveValue)
        Picco1.DoNotDrive = DoNotDriveValue;
    //ROS_INFO("RID: %d, Score: %.2f, DND: %.2f", Picco1.rID, Picco1.Score, Picco1.DoNotDrive);
    //ROS_INFO("dealer 2");
}



void Legolas::giveCommand(const multiagent_system::auction &Picco)              //reciveMsg
{
    //ROS_INFO("giveCommand %d", Picco.rID);
    if(Picco.DoNotDrive != DoNotDriveValue && (int)Picco.rID >= 1 && (int)Picco.rID <= 3)
    {
        //ROS_INFO("weGotThIsMa8Te");
        std::string topicName = "/orderToPiccolo"+std::to_string((int)Picco.rID);
        this->currPicc = this->n.advertise<multiagent_system::auction>(topicName,3);
        multiagent_system::auction destination;
        //destination.boxItem
        //
        //
        destination.rID = Picco.rID;
        destination.Position = Picco.Position;
        bool alreadyTaken = false;
        for(int i = 1; i < 4; i++)
        {
            float dist = sqrt(pow(P[i].x - destination.Position.position.x, 2) + pow(P[i].y - destination.Position.position.y, 2));
            if(dist < BoxTakenSize)
            {
                alreadyTaken = true;
                break;
            }
        }
        if(!alreadyTaken)
        {
            P[Picco.rID].x = destination.Position.position.x;
            P[Picco.rID].y = destination.Position.position.y;
            this->currPicc.publish(destination);
        }
    }
}



void Legolas::springLegolas()
{
    //ROS_INFO("springLegolas 1");
    this->locateBox();
    this->dealer(this->Picco1, this->Picco2);
    this->dealer(this->Picco2, this->Picco3);
    this->dealer(this->Picco3, this->Picco1);
    this->giveCommand(this->Picco1);
    this->giveCommand(this->Picco2);
    this->giveCommand(this->Picco3);
    this->avoid(this->robotVel, SensorBox);
    this->velPub.publish(this->robotVel);

    Picco1.Score = 0.0;
    Picco2.Score = 0.0;
    Picco3.Score = 0.0;
    Picco1.DoNotDrive = 0.0;
    Picco2.DoNotDrive = 0.0;
    Picco3.DoNotDrive = 0.0;

    //ROS_INFO("1: S:%d , F:%d", this->PiccoloRobot1.succesful, this->PiccoloRobot1.failed);
    //ROS_INFO("2: S:%d , F:%d", this->PiccoloRobot2.succesful, this->PiccoloRobot2.failed);
    //ROS_INFO("3: S:%d , F:%d", this->PiccoloRobot3.succesful, this->PiccoloRobot3.failed);
    //ROS_INFO("--------------------------");

    //ROS_INFO("springLegolas 2");
}



void Piccolos::springPiccolo()
{
    this->executeCommandCallback();
    this->work9To5();
    this->velPub.publish(this->robotVel);
}



void Piccolos::initPiccolos(std::string topicNameSub, std::string topicNamePub, int rID)
{
    std::string subName, commandName, jobName;
    subName = "/Piccolo"+std::to_string(rID);
    commandName = "/orderToPiccolo"+std::to_string(rID);
    jobName = "/PiccoloStatus";

    this->init(subName);
    this->robotID = rID;
    this->boxLocationSub = this->n.subscribe(topicNameSub, 0,&Piccolos::legolasCallbackLocation,this);
    this->reciveCommandedSub = this->n.subscribe(commandName,0,&Piccolos::reciveCommandCallback,this);
    this->bidPub = this->n.advertise<multiagent_system::auction>(topicNamePub, 3);
    this->jobPub = this->n.advertise<multiagent_system::jobStatus>(jobName, 1);
    this->robotVel.linear.x = 0.0;
    this->robotVel.angular.z = 0.0;
}

void Piccolos::legolasCallbackLocation(const multiagent_system::boxesList::ConstPtr& msg)
{
    //ROS_INFO("legolasCallbackLocation 1");
    multiagent_system::auction bid;
    float minDist = 10000.0;
    int minIndex;
    float currDist;
    bid.rID = 0;
    bid.Score = 0.0;
    bid.boxItem = 0;
    bid.Position.position.x = 0.0;
    bid.Position.position.y = 0.0;
    //ROS_INFO("size:%d",msg->data.size());
    for(int i = 0; i < msg->data.size();i+=2)
    {
        currDist = std::sqrt(std::pow(msg->data[i] - this->robotPosition.position.x, 2)+std::pow(msg->data[i+1] - this->robotPosition.position.y, 2));
        //ROS_INFO("rID:%d, currDist:%.4f", this->robotID, currDist);
        if(currDist < minDist)
        {
            minDist = currDist;
            minIndex = i;
        }

    }

    bid.rID = this->robotID;
    if(this->working || this->breakTime)
    {
        bid.DoNotDrive = DoNotDriveValue;
        bid.Score = DoNotDriveValue;
    }
    else
    {
        bid.DoNotDrive = 0.0;
        bid.Score = currDist;
    }

    bid.boxItem = minIndex;
    bid.Position.position.x = msg->data[minIndex];
    bid.Position.position.y = msg->data[minIndex+1];

    //ROS_INFO("rID:%d, bid.Score:%.4f", this->robotID, bid.Score);

    this->bidPub.publish(bid);

    //ROS_INFO("legolasCallbackLocation 2");
}



void Piccolos::reciveCommandCallback(const multiagent_system::auction::ConstPtr& msg)  //reciveMsg
{
    //ROS_INFO("reciveCommandCallback 1");
    //ROS_INFO("RID: %d, bid Value: %f", this->robotID, this->robotPosition.orientation.y);
    if(!this->working && !this->pushingBox)// && msg->DoNotDrive != DoNotDriveValue)
    {
        this->boxPosition = msg->Position.position;
        this->working = true;
    }


    ////ROS_INFO("reciveCommandCallback 2");
}


void Piccolos::executeCommandCallback()
{
    //ROS_INFO("executeCommandCallback 1");
    if(fabs(this->boxPosition.x) < 0.01 && fabs(this->boxPosition.y) < 0.01)
    {
        //ROS_INFO("RESET executeCommandCallback");
        this->reset();
        return;
    }
    float rotationAngle=0.0;

    //ROS_INFO("test %d: , rID: %d", test, this->robotID);
    if(!this->pushingBox && this->working)
    {
        if((this->boxPosition.y - this->robotPosition.position.y) != 0.0)
            rotationAngle = (float)atan2((this->boxPosition.y - this->robotPosition.position.y), (this->boxPosition.x - this->robotPosition.position.x));
        else
            rotationAngle = (float)atan2(0.0, 0.0);

        //rotationAngle = rotationAngle * M_PI;
        //ROS_INFO("Box: %.2f , %.2f", this->boxPosition.x, this->boxPosition.y);
        //ROS_INFO("Robot: %.2f , %.2f", this->robotPosition.position.x, this->robotPosition.position.y);
        //ROS_INFO("Box-Robot-: %.2f , %.2f", (this->boxPosition.x - this->robotPosition.position.x), (this->boxPosition.y - this->robotPosition.position.y));
        //ROS_INFO("RID %d, rotationAngle %f, YAW %f", this->robotID, rotationAngle, tf::getYaw(this->robotPosition.orientation));
        //ROS_INFO("RID %d, rotationAngle - Yaw: %f", this->robotID, rotationAngle - tf::getYaw(this->robotPosition.orientation));
        //ROS_INFO("rotation %f", rotationAngle);
        /*ROS_INFO("%d, angle: %f , rotationAngle: %f: ", (int)this->robotID,
                 (float)fabs(rotationAngle - (float)robotPosition.orientation.z),
                 rotationAngle);*/
        if(((rotationAngle - tf::getYaw(this->robotPosition.orientation)) > (0.2)) || ((rotationAngle - tf::getYaw(this->robotPosition.orientation)) < (-0.2)))
            this->noTurning = false;

        float turnSpeed = (rotationAngle - tf::getYaw(this->robotPosition.orientation));
        if (turnSpeed > 0.0)
        {
            turnSpeed = std::max(turnSpeed, MINSPEED);
            turnSpeed = std::min(turnSpeed, MAXSPEED);
        }
        else if(turnSpeed < 0.0)
        {
            turnSpeed = std::min(turnSpeed, -MINSPEED);
            turnSpeed = std::max(turnSpeed, -MAXSPEED);
        }

        if(!this->noTurning && ((rotationAngle - tf::getYaw(this->robotPosition.orientation)) > (0.01)) )//|| ((rotationAngle - tf::getYaw(this->robotPosition.orientation)) < (-0.02)))
        {
            //ROS_INFO("1");
            this->robotVel.angular.z = turnSpeed;
            this->robotVel.linear.x = 0.0;
        }
        else if(!this->noTurning && (rotationAngle - tf::getYaw(this->robotPosition.orientation)) < (-0.01))
        {
            //ROS_INFO("2");
            this->robotVel.angular.z = turnSpeed;
            this->robotVel.linear.x = 0.0;
        }
        else
        {
            //ROS_INFO("YIPPI");
            this->robotVel.angular.z = 0.0;
            this->robotVel.linear.x = 0.3;
            this->noTurning = true;
        }
    }
    else if(!this->pushingBox && !this->working)
    {
        //ROS_INFO("NO ACTION: RID %d", this->robotID);
        this->robotVel.angular.z = 0.0;
        this->robotVel.linear.x = 0.0;
    }
    //ROS_INFO("executeCommandCallback 2");
}


void Piccolos::work9To5()
{
    std::vector<float> sensorWall(6), sensorBox(6);
    checkDirection(sensorBox, SensorBox);
    checkDirection(sensorWall, SensorWall);
    float minLeftBox = sensorBox[0], minLeftFrontBox = sensorBox[1], minFrontBox = sensorBox[2],
            minRightFrontBox = sensorBox[3], minRightBox = sensorBox[4];
    float minLeftWall = sensorWall[0], minLeftFrontWall = sensorWall[1], minFrontWall = sensorWall[2],
            minRightFrontWall = sensorWall[3], minRightWall = sensorWall[4];

    float x, y, dist;
    x = this->robotPosition.position.x - this->boxPosition.x;
    y = this->robotPosition.position.y - this->boxPosition.y;
    dist = pow(x,2) + pow(y,2);
    dist = sqrt(dist);
    if(dist < 0.1 && minFrontBox > 0.2)
    {
        //ROS_INFO("RESET work9to5");
        this->reset();
        return;
    }

    if(minFrontBox < 0.2)
        this->pushingBox = true;

    //else
        //this->pushingBox = false;

    if(this->pushingBox && !this->breakTime)
    {
        /*if(minFrontWall < 0.5)
        {
            this->robotVel.angular.z = 0.0;
            this->robotVel.linear.x = 0.0;
            this->velPub.publish(this->robotVel);
            time(&this->now);
            this->end = this->now + 5;
            while(now < end)
                time(&this->now);
        }*/

        if(minFrontWall < 0.5)
        {  
            this->breakTime = true;

            if(minFrontBox+0.025 < minFrontWall)
            {
                ROS_INFO("JOB SUCCESSFULLY DONE");
                this->jobReport.success = true;
                this->jobReport.rID = this->robotID;
            }
            else
            {
                ROS_INFO("JOB FAILED");
                this->jobReport.success = false;
                this->jobReport.rID = this->robotID;
            }
            jobPub.publish(this->jobReport);
        }

        this->robotVel.angular.z = 0.0;
        this->robotVel.linear.x = 0.3;
    }
    else if(this->breakTime)
    {
        if(minFrontWall < 1.5)
        {
            this->robotVel.angular.z = 0.0;
            this->robotVel.linear.x = -0.3;
        }
        else
        {
            //ROS_INFO("RESET Backing clear");
            this->reset();
            return;
        }
    }
    else
    {
        //this->pushingBox = false;
        //this->breakTime = false;
    }
}



void Piccolos::reset()
{
    this->pushingBox = false;
    this->working = false;
    this->breakTime = false;
    this->noTurning = false;
    this->robotVel.angular.z = 0.0;
    this->robotVel.linear.x = 0.0;
    this->boxPosition.x = 0.0;
    this->boxPosition.y = 0.0;
}



void Agent::run()
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
  //Agent Robot1, Robot2, Robot3, Robot4;
  //Robot1.init("/robot1/scan", "/robot1/cmd_vel");
  //Robot2.init("/robot2/scan", "/robot2/cmd_vel");
  //Robot3.init("/robot3/scan", "/robot3/cmd_vel");
  //Robot4.init("/robot4/scan", "/robot4/cmd_vel");


  Legolas scout;
  Piccolos slave1, slave2, slave3;

  slave1.initPiccolos("/Legolas/BoxLocation/", "/Piccolo/bids/",1);
  slave2.initPiccolos("/Legolas/BoxLocation/", "/Piccolo/bids/",2);
  slave3.initPiccolos("/Legolas/BoxLocation/", "/Piccolo/bids/",3);
  scout.initLegolas(slave1,slave2,slave3);
  // Start the ROS main loop
  ros::Rate loop_rate(10);
  loop_rate.sleep();
  while (ros::ok()) {
    //Robot1.run();
    //Robot2.run();
    //Robot3.run();
    //Robot4.run();
    scout.springLegolas();
    slave1.springPiccolo();
    slave2.springPiccolo();
    slave3.springPiccolo();


    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

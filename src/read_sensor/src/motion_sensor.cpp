//ros
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"

//c++
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "std_msgs/String.h"

//spdlog
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"

//map
//#include <contour.h>
#include <path.h>

// #include <mapping.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

//Synchronize subscription messages
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include  "navigationFunction.cpp"

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
                                                        geometry_msgs::PoseStamped> sync_policy_classifiction;

//get the current date //date -s "20180929 08:00:00"
std::string getDate();

//point file
std::string point_path = "/home/rock/catkin_ws/src/read_sensor/data/";
// std::string point_path = "/home/yat/catkin_ws/src/read_sensor/data/";
std::string file_status = point_path + getDate() + "sensor.txt";
std::ofstream fout_point(file_status.c_str());

// typedef struct
// {
// 	double  x; // coordinate in X axis
// 	double  y; // coordinate in Y axis
// 	double  z; // coordinate in Z axis
// }POINT_XYZ;

typedef struct
{
    double dt;

    ros::Time uwbTime;
    int uwbFlag; //
	double uwb[4]; //
    
    ros::Time imuTime;
	POINT_XYZ gyro; // 
    POINT_XYZ acc;
    POINT_XYZ euler;

    ros::Time odomTime;
    POINT_XYZ odomMil;
    POINT_XYZ odomVel;

    ros::Time gnssTime;
    POINT_XYZ rtkBLH;
    POINT_XYZ rtkENU;
    double rtkYaw;
    double rtkVel;

    POINT_XYZ heading;

    bool baseType;
    POINT_XYZ baseBLH;
    POINT_XYZ baseECEF;
    POINT_XYZ baiseGyro;

    ros::Time magTime;
    double magYaw;
    POINT_XYZ magXYZ;
}SensorType;

/******************* ros publisher ****************************/
ros::Publisher pose_pub;
ros::Publisher vel_pub;

bool motionFinish = false;
bool refreshPID = false;
bool updateIMU = false;
bool updateOdom = true;
bool updateGNSS = false;
bool gotPointRTK = false;

geometry_msgs::Twist targetStates;
POINT_XYZ robotPose;
POINT_XYZ startPose;
SensorType sensorData;

#define pathNum 20

void dataRecord()
{
    static ros::Time lastTime = ros::Time::now();
    ros::Time currentTime = ros::Time::now();

    double dt = currentTime.toSec() - lastTime.toSec();
    double dt1 = sensorData.uwbTime.toSec() - lastTime.toSec();
    double dt2 = sensorData.imuTime.toSec() - lastTime.toSec();
    double dt3 = sensorData.odomTime.toSec() - lastTime.toSec();
    double dt4 = sensorData.gnssTime.toSec() - lastTime.toSec();
    double dt5 = sensorData.magTime.toSec() - lastTime.toSec();

    //log the map
    fout_point.setf(std::ios_base::showpoint);
    fout_point.precision(6);
    fout_point << dt  << " " << sensorData.uwbFlag << " " <<  sensorData.uwb[0]  << " " ;
    fout_point << sensorData.uwb[1] << " " <<  sensorData.uwb[2] << " " <<  sensorData.uwb[3]  << " " ;
    fout_point << sensorData.gyro.x << " " <<  sensorData.gyro.y << " " << sensorData.gyro.z << " ";
    fout_point << sensorData.acc.x << " " <<  sensorData.acc.y << " " << sensorData.acc.z << " ";
    fout_point << sensorData.euler.x << " " <<  sensorData.euler.y << " " << sensorData.euler.z << " ";
    fout_point << sensorData.odomMil.x << " " <<  sensorData.odomMil.y << " " << sensorData.odomMil.z << " ";
    fout_point << sensorData.odomVel.x << " " <<  sensorData.odomVel.y << " " << sensorData.odomVel.z << " ";
    fout_point << sensorData.rtkBLH.x << " " <<  sensorData.rtkBLH.y << " " << sensorData.rtkBLH.z << " ";
    fout_point << sensorData.rtkENU.x << " " <<  sensorData.rtkENU.y << " " << sensorData.rtkENU.z << " ";
    fout_point << sensorData.rtkYaw << " " <<  sensorData.rtkVel << " " << sensorData.magYaw << " ";
    fout_point << sensorData.magXYZ.x << " " <<  sensorData.magXYZ.y << " " << sensorData.magXYZ.z << " " ;
    fout_point << sensorData.heading.x << " " <<  sensorData.heading.y << " " << sensorData.heading.z << " " ;
    fout_point << gotPointRTK << " " << dt1 << " " <<  dt2 << " " <<  dt3 << " " << dt4 << " "  << dt5 << " "<< std::endl;

    lastTime = currentTime;
}

/*******************************************************************/

void locationDR()
{
    double currentYaw = sensorData.euler.z * PI/180;
    double currentMileage = sensorData.odomMil.z;

    static double yaw_prev = currentYaw; 
    static double mileage_prev = currentMileage; 
    static Eigen::Vector3d PositionDR(0,0,PI/2);

    double deltaMileage = currentMileage - mileage_prev; // delta yaw measured by imu
    double deltaYaw = deltaAngle(currentYaw,yaw_prev); // current mileage measured by odom

    robotPose.x = robotPose.x + deltaMileage * cos(robotPose.z + deltaYaw/2);
    robotPose.y = robotPose.y + deltaMileage * sin(robotPose.z + deltaYaw/2);
    robotPose.z = constrainAngle(robotPose.z + deltaYaw);

    // ROS_INFO(" robot position is: x = %f , y = %f, theta = %f .",robotPose.x,robotPose.y,robotPose.z * DegreeToRad); 

    //update the prev data
    yaw_prev = currentYaw; 
    mileage_prev = currentMileage;

    /***********************************************************************************************************************************/
    geometry_msgs::PointStamped posePub;

    posePub.header.stamp = ros::Time::now();
    posePub.header.frame_id = "motionPose";

    posePub.point.x = robotPose.x;
    posePub.point.y = robotPose.y;
    posePub.point.z = robotPose.z;

    pose_pub.publish(posePub);
}

YAT_POINTF getPathWayPoint(int num)
{
    vector<YAT_POINTF> pathWayPoint(pathNum);

    float moveDistance = 6;
    float cutWidth = 0.2;

    for(int i = 0; i < pathNum; i++)
    {
        switch (i%4)
        {
            case 0:
                pathWayPoint[i].x = startPose.x + 2 * cutWidth * (i/4) * cos(startPose.z - PI/2);
                pathWayPoint[i].y = startPose.y + 2 * cutWidth * (i/4) * sin(startPose.z - PI/2);
                break;
            case 1:
                pathWayPoint[i].x = pathWayPoint[i-1].x + moveDistance * cos(startPose.z);
                pathWayPoint[i].y = pathWayPoint[i-1].y + moveDistance * sin(startPose.z);
                break;
            case 2:
                pathWayPoint[i].x = pathWayPoint[i-1].x + cutWidth * cos(startPose.z - PI/2);
                pathWayPoint[i].y = pathWayPoint[i-1].y + cutWidth * sin(startPose.z - PI/2);
                break;
            case 3:
                pathWayPoint[i].x = pathWayPoint[i-1].x - moveDistance * cos(startPose.z);
                pathWayPoint[i].y = pathWayPoint[i-1].y - moveDistance * sin(startPose.z);
                break;
            default:
                ROS_ERROR(" error point number");
                break;
        }
    }

    return pathWayPoint[num % pathNum];
}

int sign(float data)
{
    if(data>0)
    {
        return 1;
    }
    else if(data<0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

float constrainAngle(float angle)
{
    float currentAngle = angle;
    
    while (currentAngle < -PI)
    {
        currentAngle += 2*PI;
    }
    
    while (currentAngle > PI)
    {
        currentAngle -= 2*PI;
    }

    return currentAngle;
}

float deltaDistance(YAT_POINTF pointA,YAT_POINTF pointB)
{
    float delta_X = pointA.x - pointB.x;
    float delta_Y = pointA.y - pointB.y;

    return sqrt(delta_X*delta_X + delta_Y*delta_Y);
}

float distanceToAB(YAT_POINTF pointA, YAT_POINTF pointB)
{
    YAT_POINTF currentPoint;
    currentPoint.x = robotPose.x;
    currentPoint.y = robotPose.y;

    // angle control
    float pathDistance = deltaDistance(pointA , pointB); 

    YAT_POINTF pathVector,normalVector,targetVector;
    pathVector.x = (pointA.x - pointB.x)/pathDistance;
    pathVector.y = (pointA.y - pointB.y)/pathDistance;

    // clockwise vector
    normalVector.x = pathVector.y;
    normalVector.y = - pathVector.x;

    targetVector.x = pointA.x - currentPoint.x;
    targetVector.y = pointA.y - currentPoint.y;

    return (targetVector.x*normalVector.x + targetVector.y*normalVector.y);
}

float controlPID(float error, float integral_error, float last_error, float Kp, float Ki, float Kd)
{
    float controller;
    controller = Kp * error + Ki * integral_error + Kd * (error - last_error);
    return controller;
}

bool move_forward(YAT_POINTF targetPoint, YAT_POINTF lastPoint, YAT_POINTF &commandVelocity)
{   
    // angle control
    float pathDistance = deltaDistance(targetPoint , lastPoint); 

    Eigen::Vector2d pathVector( (targetPoint.x - lastPoint.x)/pathDistance, (targetPoint.y - lastPoint.y)/pathDistance );
    Eigen::Vector2d normalVector(pathVector(1), -pathVector(0)); // clockwise vector
    Eigen::Vector2d targetVector(targetPoint.x - robotPose.x, targetPoint.y - robotPose.y);

    float trajectoryDistance = distanceToAB(targetPoint , lastPoint);  // left is positive; right is negative;
    float PathAngle = atan2( targetPoint.y - lastPoint.y, targetPoint.x - lastPoint.x);

    float aimLength = 0.5;
 
    float controlAngle = - 0.5 * PI * sign( trajectoryDistance );
    if( fabs( trajectoryDistance ) < aimLength)
    {
        controlAngle = - asin( trajectoryDistance / aimLength );
    }

    float targetRobotPose = constrainAngle(controlAngle + PathAngle);
    float  controlObject = deltaAngle(targetRobotPose,robotPose.z);  // - odomData.pose.orientation.z;

    commandVelocity.y = controlObject * 2.4 / PI;
    if (fabs(commandVelocity.y) > 0.4) commandVelocity.y = 0.4 * sign( commandVelocity.y );

    // if boundry mode, limit velocity
    double deadLineDistance = targetVector.transpose() * pathVector;
    commandVelocity.x = 0.5 * deadLineDistance * cos(controlObject);

    // horizontal moving, limit velocity
    if( fabs(commandVelocity.x) > 0.25)
    {
         commandVelocity.x = 0.25 * sign(commandVelocity.x);
    }
    else if( fabs(commandVelocity.x) < 0.1)
    {
         commandVelocity.x = 0.1 * sign(commandVelocity.x);
    }

    if(deadLineDistance < 0.05)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool turning(YAT_POINTF targetPoint, YAT_POINTF lastPoint, YAT_POINTF &commandVelocity)
{
    float targetAngle = atan2(targetPoint.y - lastPoint.y , targetPoint.x - lastPoint.x);
    float  deltaTheta = deltaAngle(targetAngle, robotPose.z);

    commandVelocity.x = 0;
    commandVelocity.y = deltaTheta * 1.8 / PI;

    if(fabs(commandVelocity.y) < 0.2)
    {
        commandVelocity.y = 0.2 * sign(deltaTheta);
    }
    else if(fabs(commandVelocity.y) > 0.4)
    {
        commandVelocity.y = 0.4 * sign(deltaTheta);
    }

    if(fabs(deltaTheta) < 5 * DegreeToRad)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool waitTimes(int waitNum)
{
    static int num = 0;
    
    num++;
    
    if(num >= waitNum)
    {
        num=0;
        return true;
    }
    else
    {
        return false;
    } 
}

bool turnedTarget(YAT_POINTF targetPoint, YAT_POINTF lastPoint,float turnDoneAngle)
{
    float targetAngle = atan2( targetPoint.y - lastPoint.y , targetPoint.x - lastPoint.x );
    bool turnedFlag = ( fabs( deltaAngle( targetAngle , robotPose.z ) ) < turnDoneAngle * DegreeToRad ) ?true:false;

    return turnedFlag;
}

bool arrivedTarget(YAT_POINTF targetPoint, YAT_POINTF lastPoint,float moveDoneLength)
{
    Eigen::Vector2d pathVector(0,0);
    Eigen::Vector2d targetVector(0,0);
    
    float pathDistance = deltaDistance( targetPoint, lastPoint );

    pathVector(0) = (targetPoint.x - lastPoint.x) / pathDistance;
    pathVector(1) = (targetPoint.y - lastPoint.y) / pathDistance;
    
    targetVector(0) = targetPoint.x - robotPose.x;
    targetVector(1) = targetPoint.y - robotPose.y;

    float deadDistance = ( pathVector.transpose() )*targetVector;

    bool arrivedFlag = ( deadDistance < moveDoneLength )?true:false;

    return arrivedFlag;
}

bool turningTargeAngle(YAT_POINTF targetPoint,YAT_POINTF lastPoint, YAT_POINTF &commandVelocity)
{
    static int motionStep = 0;
    bool motionFinish = false;


    if( motionStep == 0)
    {
        refreshPID = true;
        motionStep = 1;
    }

    if( motionStep == 1 )
    {
       if(turning(targetPoint,lastPoint,commandVelocity))
        {
            motionStep = 2;
        }
    }

    if( motionStep == 2 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(waitTimes(10))
        {
            motionStep = 0;
            motionFinish = true;
        }
    }

    return motionFinish;
}

bool movingTargePoint(YAT_POINTF targetPoint,YAT_POINTF lastPoint,YAT_POINTF &commandVelocity)
{
    static int motionStep = 0;
    bool motionFinish = false;

    if( motionStep == 0)
    {
        refreshPID = true;
        motionStep = 1;
    }

    if( motionStep == 1 )
    {
        if(move_forward(targetPoint,lastPoint,commandVelocity))
        {
            motionStep = 2;
        }
    }

    if( motionStep == 2 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(waitTimes(10))
        {
            motionStep = 0;
            motionFinish = true;
        }
    }

    spdlog::get("robotStatus")->info("motion forward: motionStep={}.",motionStep);

    return motionFinish;
}

bool testMotion()
{

    static int motionStep = 0;
    bool motionFinish = false;

    static int pointNum = 0;
    static YAT_POINTF lastPoint = getPathWayPoint(0);
    static YAT_POINTF nextPoint = getPathWayPoint(1);

    ROS_INFO("error flag 12");

    YAT_POINTF commandVelocity;

    if( motionStep == 0)
    {
        lastPoint = getPathWayPoint(pointNum);
        pointNum++;
        nextPoint = getPathWayPoint(pointNum);
        motionStep = 1;
    }

    if( motionStep == 1 )
    {
        if(turningTargeAngle(nextPoint,lastPoint,commandVelocity))
        {
            motionStep = 2;
        }
    }

    if( motionStep == 2 )
    {

        if(movingTargePoint(nextPoint,lastPoint,commandVelocity))
        {
            if(pointNum >= pathNum)
            {
                motionStep = 3;
            }
            else
            {
                motionStep = 0;
            }
        }
    }

    targetStates.linear.x = commandVelocity.x; // velocity control
    targetStates.angular.z = commandVelocity.y;    // angle control

    vel_pub.publish(targetStates);


    ROS_INFO("motion step is %d",motionStep);

    return (motionStep == 3)?true:false;
}

POINT_XYZ normalizationVector(POINT_XYZ data)
{
    double length = sqrt(data.x*data.x + data.y*data.y + data.z*data.z);
    POINT_XYZ result;
    result.x = data.x/length;
    result.y = data.y/length;
    result.z = data.z/length;
    return result;
}

bool initialization()
{
    static int pointNum = 0;

    int initialNum = 50;

    static vector<POINT_XYZ> gyroData(initialNum);
    static vector<POINT_XYZ> positionData(initialNum);

    bool leftLowSpeed = (fabs(sensorData.odomVel.x)<0.0001)?true:false;
    bool rightLowSpeed = (fabs(sensorData.odomVel.y)<0.0001)?true:false;

    ROS_INFO("error flag 1");

    if(leftLowSpeed && rightLowSpeed)
    {
        gyroData[pointNum].x = sensorData.gyro.x;
        gyroData[pointNum].y = sensorData.gyro.y;
        gyroData[pointNum].z = sensorData.gyro.z;

        pointNum++;
    }

    ROS_INFO("error flag 2");

    if(pointNum >= initialNum)
    {
        sensorData.baiseGyro = averagePoint(gyroData,pointNum);
        return true;
    }

    ROS_INFO("error flag 3");

    targetStates.linear.x = 0;
    targetStates.angular.z = 0;
    vel_pub.publish(targetStates);

    ROS_INFO("error flag 4");

    return false;
}

void magCallback(const geometry_msgs::PoseStampedConstPtr& msg_mag)
{
   sensorData.magTime = ros::Time::now();

   sensorData.magXYZ.x = msg_mag->pose.position.x;
   sensorData.magXYZ.y = msg_mag->pose.position.y;
   sensorData.magXYZ.z = msg_mag->pose.position.z;

   sensorData.magYaw = msg_mag->pose.orientation.w;
}

void uwbCallback(const geometry_msgs::PoseStampedConstPtr& msg_uwb)
{
    sensorData.uwbTime = ros::Time::now();
    sensorData.uwbFlag = msg_uwb->pose.position.z;

    sensorData.uwb[0] = msg_uwb->pose.orientation.x;
    sensorData.uwb[1] = msg_uwb->pose.orientation.y;
    sensorData.uwb[2] = msg_uwb->pose.orientation.z;
    sensorData.uwb[3] = msg_uwb->pose.orientation.w;
}

void imuCallback(const geometry_msgs::PoseArrayConstPtr& msg_imu)
{
    sensorData.imuTime = ros::Time::now();

    sensorData.acc.x = msg_imu->poses[1].position.x; // acceleration data
    sensorData.acc.y = msg_imu->poses[1].position.y;
    sensorData.acc.z = msg_imu->poses[1].position.z;

    sensorData.gyro.x = msg_imu->poses[1].orientation.x; // gyro data
    sensorData.gyro.y = msg_imu->poses[1].orientation.y;
    sensorData.gyro.z = msg_imu->poses[1].orientation.z;

    sensorData.euler.x =  msg_imu->poses[0].position.x; //euler angle
    sensorData.euler.y =  msg_imu->poses[0].position.y;
    sensorData.euler.z =  msg_imu->poses[0].position.z;

    updateIMU = true;
}

void odomCallback(geometry_msgs::Twist msg_Odom)
{
    static ros::Time prev_time = ros::Time::now();
    double dt = ros::Time::now().toSec() - prev_time.toSec();

    static bool firstTime = true;
    if( firstTime )
    {
        dt = 0.1;
        firstTime = false;
    }

    sensorData.odomTime = ros::Time::now();

    sensorData.odomMil.x = msg_Odom.angular.x; // left wheel mileage
    sensorData.odomMil.y = msg_Odom.angular.y; // right wheel mileage
    sensorData.odomMil.z = 0.5*(sensorData.odomMil.x + sensorData.odomMil.y); // mileage

    static double lastMileageL = sensorData.odomMil.x;
    static double lastMileageR = sensorData.odomMil.y;

    sensorData.odomVel.x = (sensorData.odomMil.x - lastMileageL)/dt; // left wheel velocity
    sensorData.odomVel.y = (sensorData.odomMil.y - lastMileageR)/dt; // right wheel velocity
    sensorData.odomVel.z =  0.5*(sensorData.odomVel.x + sensorData.odomVel.y); // velocity

    updateOdom = true;
}

void baseStationCallback(const geometry_msgs::PointStampedConstPtr& msg_station)
{
    if(!sensorData.baseType)
    {
        sensorData.baseECEF.x = msg_station->point.x;
        sensorData.baseECEF.y = msg_station->point.y;
        sensorData.baseECEF.z = msg_station->point.z;
        sensorData.baseBLH = ECEFtoBLH(sensorData.baseECEF);

        sensorData.baseType = true; // true is available, false is invalid

        fout_point.setf(std::ios_base::showpoint);
        fout_point.precision(15);

        // line 1
        fout_point << sensorData.baseECEF.x << "   " << sensorData.baseECEF.y << "   " << sensorData.baseECEF.z << "   ";
        fout_point << sensorData.baseBLH.x << "   " << sensorData.baseBLH.y << "   " << sensorData.baseBLH.z << "   "; 
        fout_point << sensorData.baiseGyro.x << "   " << sensorData.baiseGyro.y  << "   " <<  sensorData.baiseGyro.z  << "   " <<  " 1 " << std::endl;
    }
}

void gnssMessageCallback(const geometry_msgs::PoseStampedConstPtr& msg_position,
                         const geometry_msgs::PoseStampedConstPtr& msg_state)
{
    sensorData.gnssTime = ros::Time::now();

    sensorData.rtkBLH.x = DegreeToRad * msg_position->pose.orientation.x;
    sensorData.rtkBLH.y = DegreeToRad * msg_position->pose.orientation.y;
    sensorData.rtkBLH.z = msg_position->pose.orientation.z + msg_position->pose.orientation.w;

    POINT_XYZ pointECEF = BLHtoECEF(sensorData.rtkBLH);
    
    if( sensorData.baseType ) sensorData.rtkENU = getCoordinatesENU(pointECEF,sensorData.baseECEF,sensorData.baseBLH);

    gotPointRTK = (fabs(msg_position->pose.position.x - '4' ) < 0.01)?true:false;
    updateGNSS = ((int)(msg_state->pose.orientation.y) == 'A')?true:false;  // true is available, false is invalid

    sensorData.rtkYaw = constrainAngle( PI/2 - msg_state->pose.orientation.z * DegreeToRad);
    sensorData.rtkVel = msg_state->pose.orientation.w * 0.514444;
}

void rtkHeadingCallback(geometry_msgs::PointStamped msg_heading)
{
	sensorData.heading.x = msg_heading.point.x;
	sensorData.heading.y = msg_heading.point.y;
	sensorData.heading.z = msg_heading.point.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "initial_boundary");
    ROS_INFO_STREAM("initial the boundary ...");

    ros::NodeHandle nh;

    //Publisher
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pose_pub = nh.advertise<geometry_msgs::PointStamped>("/initial_pose", 1);

    //Subcriber
    ros::Subscriber pose_sub = nh.subscribe("/Odom", 1, odomCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu_data", 1, imuCallback);
    ros::Subscriber uwb_sub = nh.subscribe("/uwb_data", 1, uwbCallback);
    ros::Subscriber mag_sub = nh.subscribe("/mag_data", 1, magCallback);

    ros::Subscriber baseStation_sub = nh.subscribe("ecef_station", 1, baseStationCallback);
    ros::Subscriber heading_sub = nh.subscribe("rtk_yaw", 1, rtkHeadingCallback);

    message_filters::Subscriber<geometry_msgs::PoseStamped> rtkPosition_sub(nh,"gnss_position",1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> rtkState_sub(nh,"gnss_state",1);
    message_filters::Synchronizer<sync_policy_classifiction> sync(sync_policy_classifiction(10), rtkPosition_sub,rtkState_sub);
    sync.registerCallback(boost::bind(&gnssMessageCallback, _1, _2));

    robotPose.x = 0;
    robotPose.y = 0;
    robotPose.z = PI/2;

    ros::Rate loop_rate(10);
    ros::Time prev_whileloop = ros::Time::now();

    ROS_INFO("error flag 5");

    bool initialMission = false;

    while(ros::ok())
    {
        if(updateOdom || updateIMU) locationDR();
        
        ROS_INFO("error flag 6");

        if(!motionFinish)
        {

            ROS_INFO("error flag 7");

            if(!initialMission)
            {

                ROS_INFO("error flag 8");

                if(gotPointRTK && sensorData.baseType)
                {

                    ROS_INFO("error flag 9");

                    startPose = robotPose;
                    if(initialization())  initialMission = true;
                    ROS_INFO("error flag 10");

                }
            }
            else
            {
                dataRecord();


                // if(updateIMU && updateOdom)
                // {

                //     ROS_INFO("error flag 11");

                //     if(testMotion())
                //     {
                //         motionFinish = true;
                //         ROS_INFO("mission is finish");
                //     }

                //     if(!updateGNSS) ROS_INFO("initialization no GNSS data");
                // }
                // else
                // {
                //     if(!updateIMU) ROS_INFO("initialization no IMU data");
                //     if(!updateOdom) ROS_INFO("initialization no Odom data");
                    
                //     targetStates.linear.x = 0;
                //     targetStates.angular.z = 0;
                //     vel_pub.publish(targetStates);
                // }
            }

            updateOdom = false;
            updateIMU = false;
            updateGNSS = false;
        }



        ros::spinOnce();
        loop_rate.sleep();
        // ros::Duration(0.1).sleep(); // sleep for 0.1 second
    }

    fout_point.close();
    return 0;
}

std::string getDate()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d-%H-%M-%S",localtime(&timep) );
    return tmp;
}

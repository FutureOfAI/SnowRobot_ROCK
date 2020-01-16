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
#include "std_msgs/UInt8.h"

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

#include  "navigateFunction.cpp"

using namespace std;

typedef  struct
{
	double x; 
	double y;
    double z;
} PonitXYZ; 

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
                                                        geometry_msgs::PoseStamped> sync_policy_classifiction;

//get the current date //date -s "20180929 08:00:00"
std::string getDate();

//point file
std::string point_path = "/home/rock/catkin_ws/src/snow_rtk/data/";
std::string file_status = point_path + getDate() + "initialization.txt";
std::ofstream fout_point(file_status.c_str());

/******************* ros publisher ****************************/
ros::Publisher vel_pub;
ros::Publisher initial_pub;

bool stationPoint = false;
bool gotPointRTK = false;
bool updateRTK = false;
bool motionFinish = false;
bool autoMode = false;

Eigen::Vector3d stationBLH(0,0,0);
Eigen::Vector3d stationECEF(0,0,0);
Eigen::Vector3d stationENU(0,0,0);

Eigen::Vector3d robotENU(0,0,0);
Eigen::Vector3d robotECEF(0,0,0);

bool refreshPID = false;
bool refeshIMU = false;

geometry_msgs::PoseStamped odomData;
geometry_msgs::PoseStamped imuData;
geometry_msgs::PointStamped eulerAngle;

geometry_msgs::Twist targetStates;

geometry_msgs::Point robotPose;
geometry_msgs::Point robotVel;
POINT_XYZ robotAttitude;
geometry_msgs::Point rtkENU;
POINT_XYZ gyroBaise;
/*******************************************************************/

void locationDR()
{
    double currentYaw = eulerAngle.point.z*PI/180;
    double currentMileage = odomData.pose.position.x;

    static double yaw_prev = currentYaw; 
    static double mileage_prev = currentMileage; 
    static Eigen::Vector3d PositionDR(0,0,PI/2);

    double deltaMileage = currentMileage - mileage_prev; // delta yaw measured by imu
    double deltaYaw = deltaAngle(currentYaw,yaw_prev); // current mileage measured by odom

    /***********************************************  evaluate PositionDR by IMU and mileage *************************************************/ 
    Eigen::Vector3d deltaPosition(deltaMileage*cos(PositionDR(2)+deltaYaw/2),deltaMileage*sin(PositionDR(2)+deltaYaw/2),deltaYaw);
    PositionDR = PositionDR + deltaPosition;
    /***********************************************************************************************************************************/ 
    PositionDR(2) = constrainAngle(PositionDR(2));

    // ROS_INFO(" robot position is: x = %f , y = %f, theta = %f .",PositionDR(0),PositionDR(1),PositionDR(2)*180/PI);    

    robotPose.x = PositionDR(0);
    robotPose.y = PositionDR(1);
    robotPose.z = PositionDR(2);

    //update the prev data
    yaw_prev = currentYaw; 
    mileage_prev = currentMileage;
}

double controlPID(double delta,double delta_integral,double delta_last,double Kp,double Ki,double Kd)
{
    double DeltaP,DeltaI,DeltaD,DeltaControler;
    
    DeltaP=Kp*delta;
    DeltaI=Ki*delta_integral;
    DeltaD=Kd*(delta-delta_last);
    
    DeltaControler=DeltaP+DeltaI+DeltaD;

    return DeltaControler;
}

double deltaDistance(geometry_msgs::Point pointA, geometry_msgs::Point pointB)
{
    double delta_X = pointA.x - pointB.x;
    double delta_Y = pointA.y - pointB.y;

    return sqrt(delta_X*delta_X + delta_Y*delta_Y);
}

double distanceToAB(geometry_msgs::Point pointA, geometry_msgs::Point pointB,geometry_msgs::Point currentPoint)
{
    // angle control
    double pathDistance = deltaDistance(pointA , pointB); 

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

int sign(double data)
{
    if(data>0)
    {
        return 1;
    }
    else if (data<0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

bool straightMotion(geometry_msgs::Point nextPoint,geometry_msgs::Point lastPoint)
{
    // angle control
    double pathDistance = deltaDistance(nextPoint , lastPoint); 

    Eigen::Vector2d pathVector( (nextPoint.x - lastPoint.x)/pathDistance, (nextPoint.y - lastPoint.y)/pathDistance );
    Eigen::Vector2d normalVector(pathVector(1), -pathVector(0)); // clockwise vector
    Eigen::Vector2d targetVector(nextPoint.x - robotPose.x, nextPoint.y - robotPose.y);

    double trajectoryDistance = distanceToAB(nextPoint , lastPoint, robotPose);  // left is positive; right is negative;
    double PathAngle = atan2( nextPoint.y - lastPoint.y, nextPoint.x - lastPoint.x);

    double aimLength = 0.5;
 
    double controlAngle = - 0.5 * PI * sign( trajectoryDistance );
    if( fabs( trajectoryDistance ) < aimLength)
    {
        controlAngle = - asin( trajectoryDistance / aimLength );
    }

    double targetRobotPose = constrainAngle(controlAngle + PathAngle);
    double  controlObject = deltaAngle(targetRobotPose,robotPose.z);  // - odomData.pose.orientation.z;

    static double last_controlObject = controlObject;
    static double conObjectInt = 0;
    static double control_V_start = 0;

    POINT_XYZ commandVelocity;

    if(refreshPID)
    {
        last_controlObject = controlObject;
        conObjectInt = 0;
        control_V_start = 0;

        refreshPID = false;
    }
    
    /************************************************************************************************************************/
    if( fabs(controlObject) < 5 * PI / 180 )
    {
        conObjectInt = conObjectInt + controlObject;
    }
    commandVelocity.y = controlPID(controlObject, conObjectInt, last_controlObject, 0.5, 0.01, 0);
    last_controlObject = controlObject;

    if (fabs(commandVelocity.y) > 0.5) commandVelocity.y = 0.5 * sign( commandVelocity.y );

    // if boundry mode, limit velocity
    double deadLineDistance = targetVector.transpose() * pathVector;
    ROS_INFO("deadLine = %f, toLine = %f, target = %f.",deadLineDistance, trajectoryDistance, targetRobotPose); 

    commandVelocity.x = 0.5*deadLineDistance*cos(controlObject);

    // horizontal moving, limit velocity
    if( fabs(commandVelocity.x) > 0.3)
    {
         commandVelocity.x = 0.3 * sign(commandVelocity.x);
    }
    else if( fabs(commandVelocity.x) < 0.1)
    {
         commandVelocity.x = 0.1 * sign(commandVelocity.x);
    }

    if(deadLineDistance > 0.1)
    {
        targetStates.linear.x = commandVelocity.x;
        targetStates.angular.z = commandVelocity.y;
        return false;
    }
    else
    {
        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
        return true;
    }
}

bool zeroTurning(geometry_msgs::Point nextPoint,geometry_msgs::Point lastPoint)
{
    double targetAngle = atan2(nextPoint.y - lastPoint.y, nextPoint.x - lastPoint.x);

    double deltaTheta = deltaAngle(targetAngle,robotPose.z);

    ROS_INFO("target angle is %f ",targetAngle*180/PI);

    POINT_XYZ commandVelocity;
    commandVelocity.x = 0;
    commandVelocity.y = deltaTheta * 2 / PI;

    if(fabs(commandVelocity.y)<0.5)
    {
        commandVelocity.y = 0.5*sign(deltaTheta);
    }

    if(fabs(deltaTheta) > 20*PI/180)
    {
        targetStates.linear.x = commandVelocity.x;
        targetStates.angular.z = commandVelocity.y;
        return false;
    }
    else
    {
        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
        return true;
    }

    // spdlog::get("robot_status")->info("motion control states ----v:{}, w:{}",targetStates.linear.x,targetStates.angular.z); 
}

bool motionControl(geometry_msgs::Point pathPointA, geometry_msgs::Point pathPointB)
{
    static int motionStep = 0;

    static geometry_msgs::Point nextPoint;
    static geometry_msgs::Point lastPoint;

    static geometry_msgs::Point imuPointA;
    static geometry_msgs::Point imuPointB;
    static geometry_msgs::Point rtkPointA;
    static geometry_msgs::Point rtkPointB;

    static geometry_msgs::Point initailECEF;
    
    static int startCounter = 0;

    int initialNum = 10;

    if(motionStep == 0)
    {
        nextPoint.x = pathPointB.x;
        nextPoint.y = pathPointB.y;
        nextPoint.z = pathPointB.z;

        lastPoint.x = pathPointA.x;
        lastPoint.y = pathPointA.y;
        lastPoint.z = pathPointA.z;

        imuPointA.x = 0;
        imuPointA.y = 0;

        rtkPointA.x = 0;
        rtkPointA.y = 0;

        imuPointB.x = 0;
        imuPointB.y = 0;

        rtkPointB.x = 0;
        rtkPointB.y = 0;

        initailECEF.x = 0;
        initailECEF.y = 0;
        initailECEF.z = 0;

        motionStep = 1;
    }

    if(motionStep == 1)
    {
        if(zeroTurning(nextPoint,lastPoint))
        {
            motionStep = 2;
        }
    }

    if(motionStep == 2)
    {
        if(gotPointRTK && fabs(odomData.pose.position.y)<0.05)
        {
            startCounter++;

            imuPointA.x = imuPointA.x + robotPose.x/initialNum;
            imuPointA.y = imuPointA.y + robotPose.y/initialNum;

            rtkPointA.x = rtkPointA.x + robotENU(0)/initialNum;
            rtkPointA.y = rtkPointA.y + robotENU(1)/initialNum;
        }

        if(startCounter >= initialNum)
        {
            ROS_INFO(" point A : x = %f , y = %f.",rtkPointA.x,rtkPointA.y);
            motionStep = 3;
            startCounter = 0;
        }
    }

    if(motionStep == 3)
    {
        if(straightMotion(nextPoint,lastPoint))
        {
            motionStep = 4;
        }
    }

    if(motionStep == 4)
    {
        if(gotPointRTK && fabs(odomData.pose.position.y)<0.05)
        {
            startCounter++;

            imuPointB.x = imuPointB.x + robotPose.x/initialNum;
            imuPointB.y = imuPointB.y + robotPose.y/initialNum;

            rtkPointB.x = rtkPointB.x + robotENU(0)/initialNum;
            rtkPointB.y = rtkPointB.y + robotENU(1)/initialNum;
            rtkPointB.z = rtkPointB.z + robotENU(2)/initialNum;

            rtkENU = rtkPointB;
        }

        if(startCounter >= initialNum)
        {
            ROS_INFO(" point B : x = %f , y = %f.",rtkPointB.x,rtkPointB.y);
            motionStep = 5;
            startCounter = 0;
        }
    }

    if(motionStep == 5)
    {
        double imuAngle = atan2(imuPointB.y - imuPointA.y,imuPointB.x - imuPointA.x);
        double rtkAngle = atan2(rtkPointB.y - rtkPointA.y,rtkPointB.x - rtkPointA.x);
        double initialOffsetIMU = deltaAngle(rtkAngle,imuAngle);

        robotAttitude.z = constrainAngle( robotPose.z + initialOffsetIMU - PI/2);
        
        ROS_INFO("Now IMU angle = %f, angle offset = %f",robotPose.z*180/PI,initialOffsetIMU*180/PI);


        return true;
    }

    ROS_INFO("motion step is %d",motionStep);

    return false;
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

bool initialRillPitch()
{
    static int pointNum = 0;

    int initialNum = 50;

    static vector<POINT_XYZ> accData(initialNum);
    static vector<POINT_XYZ> gyroData(initialNum);

    bool leftLowSpeed = (fabs(odomData.pose.orientation.z)<0.0001)?true:false;
    bool rightLowSpeed = (fabs(odomData.pose.orientation.w)<0.0001)?true:false;

    if(leftLowSpeed && rightLowSpeed)
    {
        accData[pointNum].x = imuData.pose.position.x;
        accData[pointNum].y = imuData.pose.position.y;
        accData[pointNum].z = imuData.pose.position.z;

        accData[pointNum] = normalizationVector(accData[pointNum]);

        gyroData[pointNum].x = imuData.pose.orientation.x;
        gyroData[pointNum].y = imuData.pose.orientation.y;
        gyroData[pointNum].z = imuData.pose.orientation.z;

        pointNum++;
    }

    if(pointNum>= initialNum)
    {
        POINT_XYZ accAverage = averagePoint(accData,pointNum);
        robotAttitude.x = atan2(accAverage.y,accAverage.z);
        robotAttitude.y = asin(-accAverage.x);

        gyroBaise = averagePoint(gyroData,pointNum);

        ROS_INFO("roll is %f, pitch is %f,Z baise = %f",robotAttitude.x,robotAttitude.y,gyroBaise.z);

        return true;
    }

    targetStates.linear.x = 0;
    targetStates.angular.z = 0;

    return false;
}

bool initialRobotPose()
{
    static int motionStep = 0;
    static int waitTimes = 0;
    bool taskFinish = false;

    geometry_msgs::Point pathPointA,pathPointB;
    if(motionStep == 0)
    {
        pathPointA.x = robotPose.x;
        pathPointA.y = robotPose.y;
        pathPointA.z = robotPose.z;

        pathPointB.x = pathPointA.x + 2.0*cos(robotPose.z);
        pathPointB.y = pathPointA.y + 2.0*sin(robotPose.z);
        pathPointB.z = constrainAngle(robotPose.z);

        motionStep = 1;

        ROS_INFO(" startPoint: x = %f , y = %f , theta = %f.",robotPose.x,robotPose.y,robotPose.z);
    }

    if(motionStep == 1)
    {
        if(motionControl(pathPointA,pathPointB))
        {

            motionStep = 2; // finish initial yaw and position(x,y)
        }
    }

    if(motionStep == 2)
    {
        waitTimes ++;
        if(waitTimes > 10)
        {
            motionStep = 3;
        }
    }

    if(motionStep == 3)
    {

        if(initialRillPitch())
        {
            motionStep = 4; // finish initial roll and pitch 
        }
    }

    if(motionStep == 4)
    {
        geometry_msgs::PoseArray initialPose;
        initialPose.poses.resize(2);
        initialPose.header.stamp = ros::Time::now();
        initialPose.header.frame_id = "InitialState";

        initialPose.poses[0].position.x = rtkENU.x;
        initialPose.poses[0].position.y = rtkENU.y; 
        initialPose.poses[0].position.z = rtkENU.z;

        Eigen::Quaterniond Qbn = angleToQuaternion(robotAttitude);

        initialPose.poses[0].orientation.w = Qbn.w();
        initialPose.poses[0].orientation.x = Qbn.x();
        initialPose.poses[0].orientation.y = Qbn.y();
        initialPose.poses[0].orientation.z = Qbn.z();


        initialPose.poses[1].position.x = stationBLH(0);
        initialPose.poses[1].position.y = stationBLH(1); 
        initialPose.poses[1].position.z = stationBLH(2);

        initialPose.poses[1].orientation.x = gyroBaise.x;
        initialPose.poses[1].orientation.y = gyroBaise.y;
        initialPose.poses[1].orientation.z = gyroBaise.z;
        initialPose.poses[1].orientation.w = imuData.pose.orientation.w;

        initial_pub.publish(initialPose);

        ROS_INFO("initialization position(ENU): X = %f, Y = % f, Z = %f",rtkENU.x,rtkENU.y, rtkENU.z);
        ROS_INFO("initial robotAttitude: roll = %f , pitch = %f, yaw = %f.",robotAttitude.x*180/PI, robotAttitude.y*180/PI, robotAttitude.z*180/PI);

        motionFinish = true;

        taskFinish = true;
        motionStep = 0;
    }

    vel_pub.publish(targetStates);

    ROS_INFO("initial programa step is : %d",motionStep);

    return taskFinish;
}

void odomCallback(geometry_msgs::Twist msg_odom)
{
    static ros::Time prev_time = ros::Time::now();
    double dt = ros::Time::now().toSec() - prev_time.toSec();
    odomData.header.stamp = ros::Time::now();

    static bool firstTime = true;
    if( firstTime )
    {
        dt = 0.1;
        firstTime = false;
    }

    odomData.pose.orientation.x = msg_odom.angular.x;; // left wheel mileage
    odomData.pose.orientation.y = msg_odom.angular.y; // right wheel mileage

    static double lastMileageL = odomData.pose.orientation.x;
    static double lastMileageR = odomData.pose.orientation.y;

    odomData.pose.orientation.z = (odomData.pose.orientation.x - lastMileageL)/dt; // left wheel velocity
    odomData.pose.orientation.w = (odomData.pose.orientation.y - lastMileageR)/dt; // right wheel velocity

    odomData.pose.position.x = 0.5*(odomData.pose.orientation.x + odomData.pose.orientation.y); // mileage
    odomData.pose.position.y = 0.5*(odomData.pose.orientation.z + odomData.pose.orientation.w); // velocity
    odomData.pose.position.z = (odomData.pose.position.x - odomData.pose.position.y)/0.32;

    locationDR();

    refeshIMU = true;

    fout_point.setf(std::ios_base::showpoint);
    fout_point.precision(15);

    // line 2
    fout_point << msg_odom.linear.x << "    " << msg_odom.linear.y  << "   " << msg_odom.linear.z  << "   ";
    fout_point << msg_odom.angular.x << "   " << msg_odom.angular.y << "   " << msg_odom.angular.z << "   ";
    fout_point << " 0 " << "    " << " 0 "  << "   " <<  " 2 " << std::endl;
}

void imuCallback(const geometry_msgs::PoseArrayConstPtr& msg_imu)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    lastTime = ros::Time::now();

    imuData.header.stamp = ros::Time::now();

    imuData.pose.position.x = msg_imu->poses[1].position.x; // acceleration data
    imuData.pose.position.y = msg_imu->poses[1].position.y;
    imuData.pose.position.z = msg_imu->poses[1].position.z;

    imuData.pose.orientation.x = msg_imu->poses[1].orientation.x; // gyro data
    imuData.pose.orientation.y = msg_imu->poses[1].orientation.y;
    imuData.pose.orientation.z = msg_imu->poses[1].orientation.z;
    imuData.pose.orientation.w = msg_imu->poses[1].orientation.w;  // temperature

    eulerAngle.header.stamp = ros::Time::now();
    eulerAngle.point.x =  msg_imu->poses[0].position.x; //euler angle
    eulerAngle.point.y =  msg_imu->poses[0].position.y;
    eulerAngle.point.z =  msg_imu->poses[0].position.z;

    fout_point.setf(std::ios_base::showpoint);
    fout_point.precision(15);

    // line 11
    fout_point << msg_imu->poses[0].position.x << "    " << msg_imu->poses[0].position.y  << "   " <<  msg_imu->poses[0].position.z  << "   ";
    fout_point << msg_imu->poses[0].orientation.x << "   " << msg_imu->poses[0].orientation.y << "   " << msg_imu->poses[0].orientation.z << "   ";
    fout_point << msg_imu->poses[0].orientation.w << "    " << dt  << "   " <<  " 6 " << std::endl;

    // line 12
    fout_point << msg_imu->poses[1].position.x << "    " << msg_imu->poses[1].position.y  << "   " <<  msg_imu->poses[1].position.z  << "   ";
    fout_point << msg_imu->poses[1].orientation.x << "   " << msg_imu->poses[1].orientation.y << "   " << msg_imu->poses[1].orientation.z << "   ";
    fout_point << msg_imu->poses[1].orientation.w << "    " << dt  << "   " <<  " 7 " << std::endl;
}

void baseStationCallback(const geometry_msgs::PointStampedConstPtr& msg_station)
{
    stationECEF(0) = msg_station->point.x;
    stationECEF(1) = msg_station->point.y;
    stationECEF(2) = msg_station->point.z;

    stationBLH = ECEFtoBLH(stationECEF);

    stationPoint = true;

    fout_point.setf(std::ios_base::showpoint);
    fout_point.precision(15);

    // line 1
    fout_point << stationECEF(0) << "   " << stationECEF(1) << "   " << stationECEF(2) << "   ";
    fout_point << stationBLH(0) << "   " << stationBLH(1) << "   " << stationBLH(2) << "   "; 
    fout_point << " 0 " << "   " << " 0 "  << "   " <<  " 1 "  << std::endl;
}

void gnssMessageCallback(const geometry_msgs::PoseStampedConstPtr& msg_position,
                         const geometry_msgs::PoseStampedConstPtr& msg_state)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    lastTime = ros::Time::now();

    POINT_XYZ pointBLH;
    pointBLH.x = DegreeToRad * msg_position->pose.orientation.x;
    pointBLH.y = DegreeToRad * msg_position->pose.orientation.y;
    pointBLH.z = msg_position->pose.orientation.z + msg_position->pose.orientation.w;

    POINT_XYZ pointECEF = BLHtoECEF(pointBLH);

    robotECEF(0) = pointECEF.x;
    robotECEF(1) = pointECEF.y;
    robotECEF(2) = pointECEF.z; 

    robotENU = getCoordinatesENU(robotECEF,stationECEF,stationBLH);

    gotPointRTK = (fabs(msg_position->pose.position.x - '4' ) < 0.01) ? true:false;
    updateRTK = true;

    fout_point.setf(std::ios_base::showpoint);
    fout_point.precision(15);

    // line 4
    fout_point << msg_position->pose.position.x << "   " << msg_position->pose.position.y << "   " << msg_position->pose.position.z <<"   ";
    fout_point << msg_position->pose.orientation.x << "   " << msg_position->pose.orientation.y << "   " << msg_position->pose.orientation.z <<"   ";
    fout_point << msg_position->pose.orientation.w << "   " << dt << "   " << " 4 " << std::endl;

    // line 5
    fout_point << msg_state->pose.position.x << "   " << msg_state->pose.position.y << "   " << msg_state->pose.position.z <<"   ";
    fout_point << msg_state->pose.orientation.x << "   " << msg_state->pose.orientation.y << "   " << msg_state->pose.orientation.z <<"   ";
    fout_point << msg_state->pose.orientation.w << "   " << dt << "   " << " 5 " << std::endl;
}

void joyModeCallback(std_msgs::UInt8 msg_record)
{
    if(msg_record.data == 1)
    {
        autoMode = true;
    }
    else
    {
        autoMode = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "initial_loose");
    ROS_INFO_STREAM("Loose Integrated location and Navigation program is initialing...");

    ros::NodeHandle nh;

    //Publisher
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    initial_pub = nh.advertise<geometry_msgs::PoseArray>("/init_sensor", 1);

    //Subcriber
    ros::Subscriber pose_sub  = nh.subscribe("/Odom", 1,odomCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu_data", 1, imuCallback);

    ros::Subscriber baseStation_sub = nh.subscribe("/ecef_station", 1, baseStationCallback);
    ros::Subscriber record_pub = nh.subscribe("/mode", 1, joyModeCallback);

    message_filters::Subscriber<geometry_msgs::PoseStamped> rtkPosition_sub(nh,"/gnss_position",1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> rtkState_sub(nh,"/gnss_state",1);
    message_filters::Synchronizer<sync_policy_classifiction> sync(sync_policy_classifiction(10), rtkPosition_sub,rtkState_sub);
    sync.registerCallback(boost::bind(&gnssMessageCallback, _1, _2));

    ros::Rate loop_rate(10);
    ros::Time prev_whileloop = ros::Time::now();

    autoMode = false;

    while(ros::ok())
    {
        if((!motionFinish) && refeshIMU && updateRTK && stationPoint)
        {
            if(initialRobotPose())
            {
                motionFinish = true;
            }
            refeshIMU = false;
            updateRTK = false;
            gotPointRTK = false;
        }

        ros::spinOnce();
        // loop_rate.sleep();
        ros::Duration(0.1).sleep(); // sleep for 0.1 second
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

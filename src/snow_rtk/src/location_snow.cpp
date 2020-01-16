// ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include <string>
#include <iostream>
#include <cstdio>

//c++
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>

//Synchronize subscription messages
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include  "navigateFunction.cpp"

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
                                                        geometry_msgs::PoseStamped> sync_policy_classifiction;

// information for base station
bool updateIMU = true;

typedef struct
{
    int num;
    int enable[10];
    double gyro[10];
    double dt[10];
    POINT_XYZ point[10];
    double yaw;
}POINT_ARRAY;

// information for Loose Integrated Navigation
CONFIG_PARAMETER configData;
SOLUTION_PARAMETER solutionData;

POINT_ARRAY motionData; // use the trajectory to get the heading

ros::Publisher ekf_pub;

std::string getDate()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d-%H-%M-%S",localtime(&timep) );
    return tmp;
}

std::string point_path = "/home/rock/catkin_ws/src/snow_rtk/data/";
// std::string point_path = "/home/yat/catkin_ws/src/rtk_zigzag/data/";
std::string file_location = point_path + getDate() + "location.txt";
std::ofstream fout_point(file_location.c_str());

double deltaDistance2D(POINT_XYZ dataA,POINT_XYZ dataB)
{
    double deltaX = dataB.x - dataA.x;
    double deltaY = dataB.y - dataA.y;
    return sqrt(deltaX * deltaX + deltaY * deltaY);
}

void initialParameter(CONFIG_PARAMETER &configMessage,SOLUTION_PARAMETER &solutionMessage)
{
    POINT_XYZ zeroPoint;
    zeroPoint.x = 0;
    zeroPoint.y = 0;
    zeroPoint.z = 0;
    
    configMessage.baseENU = zeroPoint; // origin point , coordinates is (0,0,0)

    configData.armPos.x = 0; 
    configData.armPos.y = 0;
    configData.armPos.z = 0;

    configMessage.enableBase= false;
    solutionMessage.initialState = false;
}

void attitudeFilterSolution(CONFIG_PARAMETER configMessage,SOLUTION_PARAMETER &solutionMessage)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    lastTime = ros::Time::now();

    // ROS_INFO("attitude kalman filter delta time is: %5.15f",dt);

    // update state transition matrix
    Eigen::Matrix<double,2,2> PHI;
    PHI.setZero(2,2);
    PHI(0,0) = 1;
    PHI(0,1) = - dt;
    PHI(1,1) = 1;

    // update priori estimate yaw states
    solutionMessage.robotEuler.x = configMessage.euler.x;
    solutionMessage.robotEuler.y = configMessage.euler.y;
    solutionMessage.robotEuler.z = solutionMessage.robotEuler.z + dt * (configMessage.gyro.z - solutionMessage.gyroBias.z); //deltaYaw; // dt * configMessage.gyro.z; //

    // configure initial Prior estimate covariance
    Eigen::Matrix<double,2,2> P0;
    P0.setZero(2,2);
    P0(0,0) = pow(DegreeToRad*0.2,2); // initial gyro angle error
    P0(1,1) = pow(DegreeToRad/64/50,2); // initial gyro angle rate error

    static Eigen::Matrix<double,2,2> P_p = P0;

    // configure system state Noise covariance Q
    Eigen::Matrix<double,2,2> Q;
    Q.setZero(2,2);
    Q(0,0) = pow(DegreeToRad * 0.01,2); // gyro angle random noise
    Q(1,1) = pow(DegreeToRad * 0.08/3600,2);  // gyro angle rate random noise

    // configure Observation matrix
    Eigen::Matrix<double,1,2> H;
    H.setZero(1,2);
    H(0,0) = 1;

    // configure Observation noise covariance R
    Eigen::Matrix<double,1,1> R;
    R.setZero(1,1);
    R(0,0) = pow(15*DegreeToRad,2); // GPS angle random noise

    Eigen::Matrix<double,2,1> K;
    K.setZero(2,1);

    /**************************************************************/
    bool disableGPS = (sqrtValue(configMessage.gnssVel)<0.25)?true:false;
    bool disableOdom = (fabs(configData.odomVel.z)<0.25)?true:false;

    // posteriori evaluation
    double yawGPS = constrainAngle(atan2(configMessage.gnssVel.y,configMessage.gnssVel.x) - PI / 2);
    if(configMessage.odomVel.z < 0)
    {
        motionData.yaw = constrainAngle(motionData.yaw + PI);
    }

    double deltaTheta = deltaAngle(motionData.yaw,solutionMessage.robotEuler.z); 
    if(disableOdom || (!configData.enableGNSS) )
    {
        motionData.yaw = solutionMessage.robotEuler.z;
    }
    else
    {
        // update Prior estimate covariance
        P_p = PHI * P_p * PHI.transpose() + Q;

        // update kalman gain
        K = P_p*H.transpose()*( H*P_p*H.transpose() + R).inverse();

        // configure unit matrix
        Eigen::Matrix<double,2,2> I;
        I.setIdentity(2,2);

        // update posteriori estimate covariance
        Eigen::Matrix<double,2,2> P_f = (I - K*H)*P_p;
        P_p = P_f;
    }
    /**************************************************************/

    Eigen::Matrix<double,1,1> Y;
    Y(0,0) = deltaTheta;

    // update posteriori evaluation states
    Eigen::Matrix<double,2,1> X_f = K * Y;

    solutionMessage.robotEuler.z = constrainAngle(solutionMessage.robotEuler.z + X_f(0,0));
    solutionMessage.gyroBias.z = solutionMessage.gyroBias.z + X_f(1,0);
}

void positionFilterSolution(CONFIG_PARAMETER configMessage,SOLUTION_PARAMETER &solutionMessage)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    lastTime = ros::Time::now();

    // configure unit matrix
    Eigen::Matrix<double,4,4> I;
    I.setIdentity(4,4);

    // update state transition matrix
    Eigen::Matrix<double,4,4> PHI;
    PHI.setZero(4,4);
    PHI(0,2) = dt;
    PHI(1,3) = dt;
    PHI += I;

    Eigen::Matrix<double,2,2> Cbn;
    Cbn(0,0) = cos(solutionMessage.robotEuler.z);
    Cbn(0,1) = - sin(solutionMessage.robotEuler.z);
    Cbn(1,0) = sin(solutionMessage.robotEuler.z);
    Cbn(1,1) = cos(solutionMessage.robotEuler.z);

    double currentMileage = 0.5*configMessage.odomVel.z;  // current mileage measured by odom
    static double mileage_prev = currentMileage; 
    double deltaMileage = currentMileage - mileage_prev;
    mileage_prev = currentMileage;

    Eigen::Vector2d positionBody(0,deltaMileage);
    Eigen::Vector2d velocityBody(0,configData.odomVel.z);

    Eigen::Vector2d positionNav = Cbn * positionBody;
    Eigen::Vector2d velocityNav = Cbn * velocityBody;

    solutionData.robotEKF.x = solutionData.robotEKF.x + positionNav(0);
    solutionData.robotEKF.y = solutionData.robotEKF.y + positionNav(1);
    solutionData.robotVel.x = velocityNav(0);
    solutionData.robotVel.y = velocityNav(1);

    // configure initial Prior estimate covariance
    Eigen::Matrix<double,4,4> P0;
    P0.setZero(4,4);
    P0(0,0) = pow( 0.5 , 2 ); // initial X axis position noise
    P0(1,1) = pow( 0.5 , 2 ); // initial Y axis position noise
    P0(2,2) = pow( 0.1 , 2 ); // initial X axis velocity noise
    P0(3,3) = pow( 0.1 , 2 ); // initial Y axis velocity noise

    // configure system state Noise covariance Q
    Eigen::Matrix<double,4,4> Q;
    Q.setZero(4,4);
    Q(0,0) = pow( 0.0025 , 2 );  // X axis position noise of Odom
    Q(1,1) = pow( 0.0025 , 2 );  // Y axis position noise of Odom
    Q(2,2) = pow( 0.03125 , 2 );  // X axis velocity noise of Odom
    Q(3,3) = pow( 0.03125 , 2 );  // Y axis velocity noise of Odom

    // update Prior estimate covariance
    static Eigen::Matrix<double,4,4> P_p = P0;

    // configure Observation matrix
    Eigen::Matrix<double,4,4> H;
    H.setZero(4,4);
    H = I;

    // configure Observation noise covariance R
    Eigen::Matrix<double,4,4> R;
    R.setZero(4,4);
    R(0,0) = pow( 0.1 , 2 );  // X axis position noise of GNSS
    R(1,1) = pow( 0.1 , 2 );  // Y axis position noise of GNSS
    R(2,2) = pow( 0.1 , 2 );  // X axis velocity noise of GNSS
    R(3,3) = pow( 0.1 , 2 );  // Y axis velocity noise of GNSS

    Eigen::Matrix<double,4,4> K;
    K.setZero(4,4);

    if(configMessage.updateGNSS)
    {
        P_p = PHI * P_p * PHI.transpose() + Q;

        Eigen::Matrix<double,4,4> S = H*P_p*H.transpose() + R;

        // update kalman gain
        K = P_p*H.transpose()*(H*P_p*H.transpose() + R).inverse();

    }

    Eigen::Matrix<double,4,1> Y;
    Y.setZero(4,1);
    Y(0,0) = configMessage.gnssENU.x - solutionData.robotEKF.x;
    Y(1,0) = configMessage.gnssENU.y - solutionData.robotEKF.y;
    Y(2,0) = sqrtValue(configMessage.gnssVel) * cos(solutionMessage.robotEuler.z + PI/2) - solutionData.robotVel.x; // 
    Y(3,0) = sqrtValue(configMessage.gnssVel) * sin(solutionMessage.robotEuler.z + PI/2) - solutionData.robotVel.y; // 

    // update posteriori evaluation states
    Eigen::Matrix<double,4,1> X_f = K * Y;

    // update posteriori estimate covariance
    Eigen::Matrix<double,4,4> P_f = (I - K*H)*P_p;
    P_p = P_f;

    solutionData.robotEKF.x = solutionData.robotEKF.x + X_f(0,0);
    solutionData.robotEKF.y = solutionData.robotEKF.y + X_f(1,0);
    solutionData.robotVel.x = solutionData.robotVel.x + X_f(2,0);
    solutionData.robotVel.y = solutionData.robotVel.y + X_f(3,0);
}

void deadReckoningSolution(CONFIG_PARAMETER configMessage,SOLUTION_PARAMETER &solutionMessage)
{
    double currentYaw = solutionMessage.robotEuler.z;
    double currentMileage = 0.5*(configMessage.odomMil.x + configMessage.odomMil.y);  // current mileage measured by odom

    static double yaw_prev = currentYaw; 
    static double mileage_prev = currentMileage; 

    POINT_XYZ deltaTheta;
    deltaTheta.x = deltaAngle(configMessage.euler.x,solutionMessage.robotPose.x);
    deltaTheta.y = deltaAngle(configMessage.euler.y,solutionMessage.robotPose.y);
    deltaTheta.z = deltaAngle(currentYaw,yaw_prev);  // delta yaw measured by imu

    POINT_XYZ midTheta;
    midTheta.x = solutionMessage.robotPose.x + 0.5 * deltaTheta.x;
    midTheta.y = solutionMessage.robotPose.y + 0.5 * deltaTheta.y;
    midTheta.z = solutionMessage.robotPose.z + 0.5 * deltaTheta.z;

    POINT_XYZ deltaIndex;
    deltaIndex.x = cos(midTheta.x) * cos(midTheta.z) + sin(midTheta.x) * sin(midTheta.y) * sin(midTheta.z);
    deltaIndex.y = cos(midTheta.x) * sin(midTheta.z) - sin(midTheta.x) * sin(midTheta.y) * cos(midTheta.z);
    deltaIndex.z = sin(midTheta.x) * cos(midTheta.y);

    /***********************************************  evaluate robotDR by IMU and mileage *******************************************/ 
    double deltaMileage = currentMileage - mileage_prev;

    solutionMessage.robotDR.x = solutionMessage.robotDR.x + deltaMileage * deltaIndex.x;
    solutionMessage.robotDR.y = solutionMessage.robotDR.y + deltaMileage * deltaIndex.y;
    solutionMessage.robotDR.z = solutionMessage.robotDR.z + deltaMileage * deltaIndex.z;

    solutionMessage.robotPose.x = constrainAngle(solutionMessage.robotPose.x + deltaTheta.x);
    solutionMessage.robotPose.y = constrainAngle(solutionMessage.robotPose.y + deltaTheta.y);
    solutionMessage.robotPose.z = constrainAngle(solutionMessage.robotPose.z + deltaTheta.z);
    /***********************************************************************************************************************************/ 

    //update the prev data
    yaw_prev = currentYaw; 
    mileage_prev = currentMileage;
}

void updateTrajectory(CONFIG_PARAMETER configMessage,SOLUTION_PARAMETER solutionMessage)
{
    int n = motionData.num;
    double dt = configMessage.dtGNSS;
    if(n < 10)
    {
        motionData.point[n] = configMessage.gnssENU;
        motionData.enable[n] = configMessage.enableGNSS;
        motionData.gyro[n] = configMessage.gyro.z;
        motionData.dt[n] = configMessage.dtGNSS;
        motionData.yaw = solutionMessage.robotEuler.z;
    }
    else
    {
        double deltaTime = 0;

        for(int i = 0; i < 9; i++)
        {
            motionData.point[i] = motionData.point[i + 1];
            motionData.enable[i] = motionData.enable[i + 1];
            motionData.gyro[i] = motionData.gyro[i + 1];
            motionData.dt[i] = motionData.dt[i + 1];
            
            deltaTime += motionData.dt[i];
        }

        motionData.point[9] = configMessage.gnssENU;
        motionData.enable[9] = configMessage.enableGNSS;
        motionData.gyro[9] = configMessage.gyro.z;
        motionData.dt[9] = configMessage.dtGNSS;

        deltaTime += motionData.dt[9];

        double averageVelocity = deltaDistance2D(motionData.point[0],motionData.point[9]) / deltaTime;
        if(averageVelocity > 0.25)
        {
            motionData.yaw = atan2(motionData.point[9].y - motionData.point[0].y,motionData.point[9].x - motionData.point[0].x);
            for(int j = 0; j < 5; j++)
            {
                motionData.yaw += motionData.gyro[5 + j] * motionData.dt[5 + j];
            }
            motionData.yaw = constrainAngle(motionData.yaw - PI/2);

            ROS_INFO("NO.%d point averageVelocity = %f , moyion yaw = %f", motionData.num, averageVelocity, motionData.yaw * RadToDegree);
        }
        else
        {
            motionData.yaw = solutionMessage.robotEuler.z + motionData.gyro[9] * motionData.dt[9];;
        }
        
    }

    motionData.num ++;
}

void combinationNavigation(CONFIG_PARAMETER configMessage,SOLUTION_PARAMETER &solutionMessage)
{
    if( configMessage.enableGNSS && configData.enableBase && configData.updateIMU)   // if rtk is available
    {
        // update direction measured from rtk
        updateTrajectory(configMessage,solutionMessage);

        // update the attitude
        attitudeFilterSolution(configMessage,solutionMessage);

        // update the position
        positionFilterSolution(configMessage,solutionMessage);

        // update the loose navigation result
        deadReckoningSolution(configMessage,solutionMessage);
    }

    // line 8
    fout_point << solutionMessage.robotEuler.x << "   " << solutionMessage.robotEuler.y << "   " << solutionMessage.robotEuler.z  << "   ";
    fout_point << solutionMessage.robotVel.x << "   " << solutionMessage.robotVel.y << "   " << solutionMessage.robotVel.z << "   ";
    fout_point << configMessage.enableGNSS << "    " << configMessage.dtIMU  << "   " <<  " 10 " << std::endl;

    // line 9
    fout_point << solutionMessage.robotENU.x << "    " << solutionMessage.robotENU.y  << "   " <<  solutionMessage.robotENU.z  << "   ";
    fout_point << solutionMessage.robotEKF.x << "   " << solutionMessage.robotEKF.y << "   " << solutionMessage.robotEKF.z << "   ";
    fout_point << configMessage.enableBase << "    " << configMessage.dtGNSS  << "   " <<  " 11 " << std::endl;

    fout_point<< "\n";
}

void ekfPositionPubish()
{
    geometry_msgs::PoseStamped PositionPub;

    static ros::Time current_time;
    current_time=ros::Time::now();

    PositionPub.header.stamp = current_time;
    PositionPub.header.frame_id = "ekfPose";

    PositionPub.pose.position.x = solutionData.robotEKF.x;
    PositionPub.pose.position.y = solutionData.robotEKF.y;
    PositionPub.pose.position.z = solutionData.robotEKF.z;

    PositionPub.pose.orientation.x = solutionData.robotEuler.x;
    PositionPub.pose.orientation.y = solutionData.robotEuler.y;
    PositionPub.pose.orientation.z = solutionData.robotPose.z;
    PositionPub.pose.orientation.w = configData.enableGNSS;

    ekf_pub.publish(PositionPub);
}

void initialCallback(geometry_msgs::PoseArray msg_initial)
{
    configData.gnssENU.x =  msg_initial.poses[0].position.x; // initial GNSS position
    configData.gnssENU.y =  msg_initial.poses[0].position.y;
    configData.gnssENU.z =  msg_initial.poses[0].position.z;

    solutionData.Qbn[0] = msg_initial.poses[0].orientation.w;
    solutionData.Qbn[1] = msg_initial.poses[0].orientation.x;
    solutionData.Qbn[2] = msg_initial.poses[0].orientation.y;
    solutionData.Qbn[3] = msg_initial.poses[0].orientation.z;

    Eigen::Matrix3d Cbn = quaternionToMatrix(solutionData.Qbn);
    Eigen::Quaterniond Qbn = Eigen::Quaterniond(solutionData.Qbn[0], solutionData.Qbn[1], solutionData.Qbn[2], solutionData.Qbn[3]);//(w,x,y,z)
    solutionData.robotEuler = matrixToAngle(Qbn.toRotationMatrix());

    ROS_INFO("location get initial rtk ENU: x = %f, y = %f , z = %f",configData.gnssENU.x,configData.gnssENU.y,configData.gnssENU.z);
    ROS_INFO("location get initial angle: x = %f, y = %f , z = %f",solutionData.robotEuler.x * RadToDegree, solutionData.robotEuler.y * RadToDegree, solutionData.robotEuler.z * RadToDegree);

    /**********************************************************************************************/
    configData.baseBLH.x = msg_initial.poses[1].position.x;
    configData.baseBLH.y = msg_initial.poses[1].position.y;
    configData.baseBLH.z = msg_initial.poses[1].position.z;
    configData.baseECEF = BLHtoECEF(configData.baseBLH);
    configData.enableBase = true;
    // ROS_INFO("location get station BLH: x = %f, y = %f , z = %f",configData.baseBLH.x*180/PI, configData.baseBLH.y*180/PI, configData.baseBLH.z);

    solutionData.gyroBias.x = DegreeToRad * msg_initial.poses[1].orientation.x;
    solutionData.gyroBias.y = DegreeToRad * msg_initial.poses[1].orientation.y;
    solutionData.gyroBias.z = DegreeToRad * msg_initial.poses[1].orientation.z;

    /*********************************************************************************************/

    // double deltaLength = modulusValue(configData.armPos);
    Eigen::Vector3d deltaArmPos(configData.armPos.x,configData.armPos.y,configData.armPos.z);
    Eigen::Vector3d deltaLength = Cbn * deltaArmPos;

    solutionData.robotEKF.x =  configData.gnssENU.x + deltaLength(0);
    solutionData.robotEKF.y =  configData.gnssENU.y + deltaLength(1);
    solutionData.robotEKF.z =  configData.gnssENU.z + deltaLength(2);

    ROS_INFO("location get initial ENU: x = %f, y = %f , z = %f",solutionData.robotENU.x, solutionData.robotENU.y, solutionData.robotENU.z);

    solutionData.robotECEF = ENUtoECEF(solutionData.robotENU,configData.baseBLH,configData.baseECEF);// base station is (0,0,0)

    Eigen::Vector3d velocityB(0, 0.5*(configData.odomMil.x + configData.odomMil.y), 0);
    Eigen::Vector3d velocityN = Cbn * velocityB;

    solutionData.robotVel.x = velocityN(0);
    solutionData.robotVel.y = velocityN(1);
    solutionData.robotVel.z = velocityN(2);

    ROS_INFO("location get initial Vel: x = %f, y = %f , z = %f",solutionData.robotVel.x, solutionData.robotVel.y, solutionData.robotVel.z);

    solutionData.robotDR = solutionData.robotEKF;

    solutionData.robotPose.x = solutionData.robotEuler.x;
    solutionData.robotPose.y = solutionData.robotEuler.y;
    solutionData.robotPose.z = constrainAngle(solutionData.robotEuler.z + PI/2);

    solutionData.initialState = true;
}

void baseStationCallback(const geometry_msgs::PointStampedConstPtr& msg_station)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    lastTime = ros::Time::now();

    if(!configData.enableBase)
    {
        configData.baseECEF.x = msg_station->point.x;
        configData.baseECEF.y = msg_station->point.y;
        configData.baseECEF.z = msg_station->point.z;

        configData.baseBLH = ECEFtoBLH(configData.baseECEF);

        configData.enableBase = true; // true is available, false is invalid

        fout_point.setf(std::ios_base::showpoint);
        fout_point.precision(15);

        // line 1
        fout_point << configData.baseECEF.x << "   " << configData.baseECEF.y << "   " << configData.baseECEF.z << "   ";
        fout_point << configData.baseBLH.x << "   " << configData.baseBLH.y << "   " << configData.baseBLH.z << "   "; 
        fout_point << " 0 " << "   " << dt  << "   " <<  " 1 "  << std::endl;
    }
}

void odomCallback(geometry_msgs::Twist msg_odom)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    lastTime = ros::Time::now();

    static bool firstTime = true;
    if( firstTime )
    {
        dt = 0.1;
        firstTime = false;
    }

    configData.dtOdom = dt;

    configData.odomMil.x = msg_odom.angular.x; // left wheel mileage
    configData.odomMil.y = msg_odom.angular.y; // right wheel mileage
    configData.odomMil.z = 0.5*(configData.odomMil.x + configData.odomMil.y); // mileage

    static double lastMileageL = configData.odomMil.x;
    static double lastMileageR = configData.odomMil.y;

    configData.odomVel.x = (configData.odomMil.x - lastMileageL)/dt; // left wheel velocity
    configData.odomVel.y = (configData.odomMil.y - lastMileageR)/dt; // right wheel velocity
    configData.odomVel.z =  0.5*(configData.odomVel.x + configData.odomVel.y); // velocity

    configData.updateOdom = true;

    fout_point.setf(std::ios_base::showpoint);
    fout_point.precision(15);

    // line 2
    fout_point << msg_odom.linear.x << "    " << msg_odom.linear.y  << "   " << msg_odom.linear.z  << "   ";
    fout_point << msg_odom.angular.x << "   " << msg_odom.angular.y << "   " << msg_odom.angular.z << "   ";
    fout_point << " 0 " << "    " << " 0 "  << "   " <<  " 2 " << std::endl;
}

void gnssMessageCallback(const geometry_msgs::PoseStampedConstPtr& msg_position,
                         const geometry_msgs::PoseStampedConstPtr& msg_state)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    lastTime = ros::Time::now();

    configData.dtGNSS = dt;

    configData.gnssBLH.x = DegreeToRad * msg_position->pose.orientation.x;
    configData.gnssBLH.y = DegreeToRad * msg_position->pose.orientation.y;
    configData.gnssBLH.z = msg_position->pose.orientation.z + msg_position->pose.orientation.w;

    configData.gnssECEF = BLHtoECEF(configData.gnssBLH);
    
    if( configData.enableBase ) configData.gnssENU = getCoordinatesENU(configData.gnssECEF,configData.baseECEF,configData.baseBLH);
                           else configData.gnssENU = configData.baseENU;


    configData.enableGNSS = (fabs(msg_position->pose.position.x - '4' ) < 0.01)?true:false;
    configData.updateGNSS = ((int)(msg_state->pose.orientation.y) == 'A')?true:false;  // true is available, false is invalid

    double speedGround = 0.5144444 * msg_state->pose.orientation.w;
    double directionGround = constrainAngle(PI/2 - msg_state->pose.orientation.z * DegreeToRad);

    configData.gnssVel.x = speedGround * cos(directionGround);
    configData.gnssVel.y = speedGround * sin(directionGround);
    configData.gnssVel.z = 0;

    /**********************************************************************************************/

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
    /**********************************************************************************************/
}

void imuCallback(const geometry_msgs::PoseArrayConstPtr& msg_imu)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    lastTime = ros::Time::now();

    /**********************************************************************************************/
    configData.dtIMU = dt;

    configData.euler.x = DegreeToRad * msg_imu->poses[0].position.x; // euler angle measured by IMU module
    configData.euler.y = DegreeToRad * msg_imu->poses[0].position.y; 
    configData.euler.z = DegreeToRad * msg_imu->poses[0].position.z;

    configData.accel.x = earth_g * msg_imu->poses[1].position.x;  // acceleration data in body coordinate frame
    configData.accel.y = earth_g * msg_imu->poses[1].position.y;
    configData.accel.z = earth_g * msg_imu->poses[1].position.z;

    configData.gyro.x = DegreeToRad * msg_imu->poses[1].orientation.x; // gyro data velocity in body coordinate frame
    configData.gyro.y = DegreeToRad * msg_imu->poses[1].orientation.y;
    configData.gyro.z = DegreeToRad * msg_imu->poses[1].orientation.z;

    configData.imuTemperature = msg_imu->poses[1].orientation.w;

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

void headingCallback(const geometry_msgs::PointStampedConstPtr& msg_heading)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    lastTime = ros::Time::now();

    // line 8
    fout_point << msg_heading->point.x << "    " << msg_heading->point.y  << "   " <<  msg_heading->point.z  << "   ";
    fout_point << " 0 " << "   " << " 0 " << "   " << " 0 " << "   ";
    fout_point << " 0 " << "    " << dt  << "   " <<  " 8 " << std::endl;
}

void magCallback(const geometry_msgs::PoseStampedConstPtr& msg_mag)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    lastTime = ros::Time::now();

    // line 9
    fout_point << msg_mag->pose.position.x << "    " << msg_mag->pose.position.x  << "   " <<  msg_mag->pose.position.x  << "   ";
    fout_point << msg_mag->pose.orientation.w << "   " << " 0 " << "   " << " 0 " << "   ";
    fout_point << " 0 " << "    " << dt  << "   " <<  " 9 " << std::endl;
}

void uwbCallback(const geometry_msgs::PoseStampedConstPtr& msg_uwb)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    lastTime = ros::Time::now();

    // line 9
    fout_point << msg_uwb->pose.position.x << "    " << msg_uwb->pose.position.x  << "   " <<  msg_uwb->pose.position.x  << "   ";
    fout_point << msg_uwb->pose.orientation.x << "   " << msg_uwb->pose.orientation.y << "   " << msg_uwb->pose.orientation.z << "   ";
    fout_point << msg_uwb->pose.orientation.w << "    " << dt  << "   " <<  " 9 " << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "location_loose");
    ros::Time::init();

    ROS_INFO_STREAM("Loose Integrated location and Navigation program is starting...");

    ros::NodeHandle nh;

    ekf_pub = nh.advertise<geometry_msgs::PoseStamped>("/ekf_pose", 10);

    ros::Subscriber initial_sub  = nh.subscribe("/init_sensor", 1,initialCallback);
    ros::Subscriber baseStation_sub = nh.subscribe("ecef_station", 1, baseStationCallback);
    ros::Subscriber odom_sub = nh.subscribe("Odom", 1, odomCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu_data", 1, imuCallback);

    ros::Subscriber heading_sub = nh.subscribe("/rtk_yaw", 1, headingCallback);
    ros::Subscriber mag_sub = nh.subscribe("/mag_data", 1, magCallback);
    ros::Subscriber uwb_sub = nh.subscribe("/uwb_data", 1, uwbCallback);

    message_filters::Subscriber<geometry_msgs::PoseStamped> rtkPosition_sub(nh,"gnss_position",1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> rtkState_sub(nh,"gnss_state",1);
    message_filters::Synchronizer<sync_policy_classifiction> sync(sync_policy_classifiction(10),rtkPosition_sub,rtkState_sub);
    sync.registerCallback(boost::bind(&gnssMessageCallback, _1, _2));

    ros::Time prev_time = ros::Time::now();

    initialParameter(configData,solutionData);

    // ros::Duration(10.0).sleep(); // sleep for 10.0 second
    // ROS_INFO("the start waiting time is %f.",ros::Time::now().toSec() - prev_time.toSec());

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        double dt = ros::Time::now().toSec() - prev_time.toSec();
        //ROS_INFO("main time duration is %f.",dt);
        prev_time = ros::Time::now();

        ros::Time startTime = ros::Time::now();

        // if(solutionData.initialState)
        if(solutionData.initialState) 
        {
            combinationNavigation(configData,solutionData);

            ekfPositionPubish();
            configData.enableBase = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    // fout_point.close();
    return 0;

}

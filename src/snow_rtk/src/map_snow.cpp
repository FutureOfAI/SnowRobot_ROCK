//ros
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
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

//spdlog
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"

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

using namespace std;

#include  "navigateFunction.cpp"

//typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray, geometry_msgs::PoseStamped> sync_policy_classifiction;
//get the current date //date -s "20180929 08:00:00"

typedef struct
{
    int num;

    bool rtkType;
    POINT_XYZ rtkBLH;
    POINT_XYZ rtkENU;
    POINT_XYZ rtkECEF;

    bool baseType;
    POINT_XYZ baseBLH;
    POINT_XYZ baseENU;
    POINT_XYZ baseECEF;

    bool recordData;
    bool stopRecord;
    bool initialRecord;
}SensorType;

SensorType sensorData;

std::string getTime()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d-%H-%M-%S",localtime(&timep) );
    return tmp;
}

//filerock
std::string getTime();
string map_path = "/home/rock/catkin_ws/src/snow_rtk/data/boundary/";
string filename_map_save = map_path + getTime() + "boundary.txt";
ofstream fout_save(filename_map_save.c_str()); //save map
string filename_map = map_path + "boundary.txt";
ofstream fout_map(filename_map.c_str()); //save map

void sensorDataInitialization()
{
    sensorData.num = 0;

    sensorData.rtkType = false;
    sensorData.baseType = false;

    sensorData.recordData = false;
    sensorData.stopRecord = false;
    sensorData.initialRecord = false;
    
    sensorData.baseENU.x = 0;
    sensorData.baseENU.y = 0;
    sensorData.baseENU.z = 0;
}

void positionCallback(const geometry_msgs::PoseStampedConstPtr& msg_position)
{
    sensorData.rtkType = ((int)(msg_position->pose.position.x) == '4')?true:false;  // 4 is available, others is invalid

    sensorData.rtkBLH.x =  DegreeToRad * msg_position->pose.orientation.x;
    sensorData.rtkBLH.y =  DegreeToRad * msg_position->pose.orientation.y;
    sensorData.rtkBLH.z =  msg_position->pose.orientation.z + msg_position->pose.orientation.w;

    sensorData.rtkECEF = BLHtoECEF(sensorData.rtkBLH);

    if( sensorData.baseType ) sensorData.rtkENU = getCoordinatesENU(sensorData.rtkECEF,sensorData.baseECEF,sensorData.baseBLH);
                         else sensorData.rtkENU = sensorData.baseENU;

    if(sensorData.baseType && sensorData.rtkType)
    {
        if(sensorData.rtkType) sensorData.num ++;

        //record map
        fout_map.setf(std::ios_base::showpoint);
        fout_map.precision(15);
        fout_map << sensorData.rtkENU.x << "   " << sensorData.rtkENU.y << "   " <<  sensorData.rtkENU.z << "   ";
        fout_map << sensorData.rtkBLH.x << "   " << sensorData.rtkBLH.y << "   " <<  sensorData.rtkBLH.z << "   ";
        fout_map << sensorData.rtkType  << "   " << sensorData.num << std::endl;

        // save the map
        fout_save.setf(std::ios_base::showpoint);
        fout_save.precision(15);
        fout_save << sensorData.rtkENU.x << "   " << sensorData.rtkENU.y << "   " <<  sensorData.rtkENU.z << "   ";
        fout_save << sensorData.rtkBLH.x << "   " << sensorData.rtkBLH.y << "   " <<  sensorData.rtkBLH.z << "   ";
        fout_save << sensorData.rtkType  << "   " << sensorData.num << std::endl;
    }
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
    }
}

void joyModeCallback(std_msgs::UInt8 msg_record)
{
    ROS_INFO("snow map: get the record enable flag : %d.",msg_record.data);

    if(msg_record.data == 3)
    {
        sensorData.stopRecord = false;
        sensorData.recordData = true;
        sensorData.initialRecord = true;

        ROS_INFO("snow map: get the record enable flag, start recording.");
    }
    else
    {
        sensorData.recordData = false;
        sensorData.stopRecord = true;
        ROS_INFO("snow map: get the record disable flag, stop recording.");
    }

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "mapping");
    ROS_INFO_STREAM("snow mapping program is starting...");

    ros::NodeHandle nh;
    ros::Subscriber baseStation_sub = nh.subscribe("/ecef_station", 1, baseStationCallback);
    ros::Subscriber position_sub  = nh.subscribe("/gnss_position", 1, positionCallback);
    ros::Subscriber record_pub = nh.subscribe("/mode", 1, joyModeCallback);

    ros::Publisher  joy_pub = nh.advertise<std_msgs::Bool>("/Joy_Enable", 100);
    ros::Publisher  map_sub = nh.advertise<std_msgs::Bool>("/maping_finish", 100);

    sensorDataInitialization();
    
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        if(sensorData.initialRecord)
        {
            if(sensorData.stopRecord)
            {
                fout_map.close();
                fout_save.close();
                ROS_INFO("snow map : fencing done.");
                break;
            }
            else
            {
                if(!sensorData.baseType) ROS_INFO("there is no base station message.");
                if(!sensorData.rtkType) ROS_INFO("there is no rtk message.");
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}




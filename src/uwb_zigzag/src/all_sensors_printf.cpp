#include "ros/ros.h"
#include <signal.h> // SIGINT
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <stdlib.h>
#include <stdio.h>

// #define PI 3.14159265358979323846
typedef struct Data
{
    float acc[3];
    float gro[3];
    float mag[3];
    float YawM;
    float euler[3];
    float quat[4];
    float temprature;
    uint8_t headUWB[3];
    float distance[4];
    float rtkPos[3];
    float dt;
}RawData;

// global variable define
float RawAcc[3], RawGro[3], RawMag[3], MagYaw, Euler[3], Quat[4], Tempra, HeaderUWB[3], DistUWB[4], PosRTK[3];
RawData DataBuffer[18000]; //save 30min data when 10hz sample rate
uint16_t BufCnt = 0; // data buffer counter

std::string getDate();

void Save_Data(RawData *db, uint16_t cnt)
{
    // create raw data base text
    std::string data_path = "/home/rock/catkin_ws/src/uwb_zigzag/data/";
    std::string current_file = data_path + getDate() + "_UWBandIMURawData.txt";
    // ROS_INFO_STREAM(current_file); // cout stream        
    FILE *fp;
    fp = fopen(current_file.c_str(), "w"); // create data.txt file

    // write data to text file
    for (int i = 0; i < cnt; ++i)
    {
        fprintf (fp, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %d %d %f %f %f %f %f \n", \
            db[i].acc[0], db[i].acc[1], db[i].acc[2], \
            db[i].gro[0], db[i].gro[1], db[i].gro[2], \
            db[i].mag[0], db[i].mag[1], db[i].mag[2], db[i].YawM, \
            db[i].euler[0], db[i].euler[1], db[i].euler[2], \
            db[i].quat[0], db[i].quat[1], db[i].quat[2], db[i].quat[3], \
            db[i].temprature, \
            db[i].headUWB[0], db[i].headUWB[1], db[i].headUWB[2], \
            db[i].distance[0], db[i].distance[1], db[i].distance[2], db[i].distance[3], \
            db[i].rtkPos[0], db[i].rtkPos[1], db[i].rtkPos[2], \
            db[i].dt); 
    }
    fclose(fp); // close the file
}

// imu data subscribe request
void poseCallback(geometry_msgs::PoseArray msg_alf)
{
    RawAcc[0] = msg_alf.poses[1].position.x; // accx
    RawAcc[1] = msg_alf.poses[1].position.y; // accy
    RawAcc[2] = msg_alf.poses[1].position.z; // accz

    RawGro[0] = msg_alf.poses[1].orientation.x; // grox
    RawGro[1] = msg_alf.poses[1].orientation.y; // groy
    RawGro[2] = msg_alf.poses[1].orientation.z; // groz
    Tempra = msg_alf.poses[1].orientation.w; // temprature
    
    Quat[0] = msg_alf.poses[0].orientation.x; // quaternion[0]
    Quat[1] = msg_alf.poses[0].orientation.y; // [1]
    Quat[2] = msg_alf.poses[0].orientation.z; // [2]
    Quat[3] = msg_alf.poses[0].orientation.w; // [3]

    Euler[0] = msg_alf.poses[0].position.x; // roll
    Euler[1] = msg_alf.poses[0].position.y; // pitch
    Euler[2] = msg_alf.poses[0].position.z; // yaw

    // ROS_INFO("%f %f %f\n", RawGro[0], RawGro[1], RawGro[2]);
    
    // odomData.pose.position.x = 0.5*(msg_alf.poses[1].orientation.x + msg_alf.poses[1].orientation.y);
    // odomData.pose.position.y = 0.5*(msg_alf.poses[1].orientation.z + msg_alf.poses[1].orientation.w);
    // odomData.pose.position.z = 180/PI*(msg_alf.poses[1].orientation.y - msg_alf.poses[1].orientation.x)/0.32;

    // odomData.pose.orientation.x = msg_alf.poses[1].orientation.x;
    // odomData.pose.orientation.y = msg_alf.poses[1].orientation.y;
    // odomData.pose.orientation.z = msg_alf.poses[1].orientation.z;
    // odomData.pose.orientation.w = msg_alf.poses[1].orientation.w;
}

// uwb data subscribe request
void uwbCallback(geometry_msgs::PoseStamped msg_uwb)
{
    HeaderUWB[0] = msg_uwb.pose.position.x;
    HeaderUWB[1] = msg_uwb.pose.position.y;
    HeaderUWB[2] = msg_uwb.pose.position.z;

    DistUWB[0] = msg_uwb.pose.orientation.x; // anchor[0]
    DistUWB[1] = msg_uwb.pose.orientation.y; // anchor[1]
    DistUWB[2] = msg_uwb.pose.orientation.z; // anchor[2]
    DistUWB[3] = msg_uwb.pose.orientation.w; // anchor[3]

    // ROS_INFO("%f %f %f\n", HeaderUWB[0], HeaderUWB[1], HeaderUWB[2]);
    // ROS_INFO("%f %f %f %f\n", DistUWB[0], DistUWB[1], DistUWB[2], DistUWB[3]);
}

// magnetic sensor subscribe request
void magCallback(geometry_msgs::PoseStamped msg_mag)
{
    RawMag[0] = msg_mag.pose.position.x;
    RawMag[1] = msg_mag.pose.position.y;
    RawMag[2] = msg_mag.pose.position.z;

    MagYaw = msg_mag.pose.orientation.w;
}

// rtk gnss subscribe request
void gnssCallback(geometry_msgs::PoseStamped msg_gnss)
{
    PosRTK[0] = msg_gnss.pose.orientation.x;
    PosRTK[1] = msg_gnss.pose.orientation.y;
    PosRTK[2] = msg_gnss.pose.orientation.z;
}

void MySigintHandler(int sig)
{
    // save data buffer to txt
    ROS_INFO("Saving data and shutdown ROS!");
    Save_Data(DataBuffer, BufCnt);
    ros::shutdown(); // shut down ros node
}

// get the current date //date -s "20180929 08:00:00"
std::string getDate()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y%m%d_%H%M%S",localtime(&timep) );
    return tmp;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Sensors_Data");
    ROS_INFO_STREAM("Sensors printf topic is starting...");

    ros::NodeHandle nl;

    ros::Subscriber pose_sub = nl.subscribe("/alf001_dis", 1000, poseCallback);
    // ros::Subscriber uwb_sub = nl.subscribe("/myuwb", 1000, uwbCallback); 
    ros::Subscriber uwb_sub = nl.subscribe("/uwb_raw", 1000, uwbCallback);
    ros::Subscriber mag_sub = nl.subscribe("/mag_data", 1000, magCallback);
    ros::Subscriber gnss_sub = nl.subscribe("/gnss_position", 1000, gnssCallback);

    ros::Rate loop_rate(10); //10hz loop rate
    signal(SIGINT, MySigintHandler); // replace ctrl-c to my own shutdown function

    ros::Time prev_time; // previous time record

    while(ros::ok())
    {
	    // ROS_INFO("%f %f %f\n", RawGro[0], RawGro[1], RawGro[2]);
        // ROS_INFO("%f %f %f %f\n", DistUWB[0], DistUWB[1], DistUWB[2], DistUWB[3]);
        // save date to buffer
        if (BufCnt <= 18000)
        {
            // acc
            DataBuffer[BufCnt].acc[0] = RawAcc[0];
            DataBuffer[BufCnt].acc[1] = RawAcc[1];
            DataBuffer[BufCnt].acc[2] = RawAcc[2];
            // gro
            DataBuffer[BufCnt].gro[0] = RawGro[0];
            DataBuffer[BufCnt].gro[1] = RawGro[1];
            DataBuffer[BufCnt].gro[2] = RawGro[2];
            // mag
            DataBuffer[BufCnt].mag[0] = RawMag[0];
            DataBuffer[BufCnt].mag[1] = RawMag[1];
            DataBuffer[BufCnt].mag[2] = RawMag[2];
            // mag yaw angle
            DataBuffer[BufCnt].YawM = MagYaw;
            // euler
            DataBuffer[BufCnt].euler[0] = Euler[0];
            DataBuffer[BufCnt].euler[1] = Euler[1];
            DataBuffer[BufCnt].euler[2] = Euler[2];
            // quaternion
            DataBuffer[BufCnt].quat[0] = Quat[0];
            DataBuffer[BufCnt].quat[1] = Quat[1];
            DataBuffer[BufCnt].quat[2] = Quat[2];
            DataBuffer[BufCnt].quat[3] = Quat[3];
            // temprature
            DataBuffer[BufCnt].temprature = Tempra;
            // uwb header
            DataBuffer[BufCnt].headUWB[0] = HeaderUWB[0];
            DataBuffer[BufCnt].headUWB[1] = HeaderUWB[1];
            DataBuffer[BufCnt].headUWB[2] = HeaderUWB[2];
            // uwb distance
            DataBuffer[BufCnt].distance[0] = DistUWB[0];
            DataBuffer[BufCnt].distance[1] = DistUWB[1];
            DataBuffer[BufCnt].distance[2] = DistUWB[2];
            DataBuffer[BufCnt].distance[3] = DistUWB[3];
            // rtk position in earth frame
            DataBuffer[BufCnt].rtkPos[0] = PosRTK[0];
            DataBuffer[BufCnt].rtkPos[1] = PosRTK[1];
            DataBuffer[BufCnt].rtkPos[2] = PosRTK[2];

            // dt
            float dt=ros::Time::now().toSec()-prev_time.toSec();            
            DataBuffer[BufCnt].dt = dt;
            ROS_INFO("main time duration is %f.",dt); 
            // counter
            BufCnt++;
        }
        prev_time=ros::Time::now(); // record previous time

	    ros::spinOnce(); 
        loop_rate.sleep();
    }
    return 0;
}

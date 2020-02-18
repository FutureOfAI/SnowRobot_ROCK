#include "ros/ros.h"
#include <signal.h> // SIGINT
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
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
    uint8_t rawMaskUWB;
    float rawDistance[4];
    uint8_t baseMaskUWB;
    float baseDistance[4];
    uint8_t rtkType;
    float rtkPos[3];
    float rtkTrackTrue;
    float rtkSpeed;
    float rtkStation[3];
    float rtkHead;
    float odomShift[2];
    float dt;
}RawData;

// global variable define
float RawAcc[3], RawGro[3], RawMag[3], MagYaw, Euler[3], Quat[4], Tempra, RawDistUWB[4], BaseDistUWB[4], PosRTK[3], TrackTrue, SpeedRTK, stationRTK[3], headRTK, shiftOdom[2];
uint8_t MaskUWBRaw, MaskUWBBase, TypeRTK;
RawData DataBuffer[18000]; //save 30min data when 10hz sample rate
uint16_t BufCnt = 0; // data buffer counter

std::string getDate();

void Save_Data(RawData *db, uint16_t cnt)
{
    // create raw data base text
    std::string data_path = "/home/rock/catkin_ws/src/uwb_zigzag/data/";
    std::string current_file = data_path + getDate() + "_SensorsRawData.txt";
    // ROS_INFO_STREAM(current_file); // cout stream        
    FILE *fp;
    fp = fopen(current_file.c_str(), "w"); // create data.txt file

    // write data to text file
    for (int i = 0; i < cnt; ++i)
    {
        fprintf (fp, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %d %f %f %f %f %d %0.8f %0.8f %0.8f %0.8f %0.8f %0.8f %0.8f %0.8f %0.8f %f %f %f \n", \
            db[i].acc[0], db[i].acc[1], db[i].acc[2], \
            db[i].gro[0], db[i].gro[1], db[i].gro[2], \
            db[i].mag[0], db[i].mag[1], db[i].mag[2], db[i].YawM, \
            db[i].euler[0], db[i].euler[1], db[i].euler[2], \
            db[i].quat[0], db[i].quat[1], db[i].quat[2], db[i].quat[3], \
            db[i].temprature, \
            db[i].rawMaskUWB, db[i].rawDistance[0], db[i].rawDistance[1], db[i].rawDistance[2], db[i].rawDistance[3], \
            db[i].baseMaskUWB, db[i].baseDistance[0], db[i].baseDistance[1], db[i].baseDistance[2], db[i].baseDistance[3], \
            db[i].rtkType, db[i].rtkPos[0], db[i].rtkPos[1], db[i].rtkPos[2], \
            db[i].rtkTrackTrue, db[i].rtkSpeed, \
            db[i].rtkStation[0], db[i].rtkStation[1], db[i].rtkStation[2], \
            db[i].rtkHead, \
            db[i].odomShift[0], db[i].odomShift[1], \
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

// uwb tag to all station data subscribe request
void uwbRawCallback(geometry_msgs::PoseStamped msg_uwb)
{
    MaskUWBRaw = msg_uwb.pose.position.z;
    RawDistUWB[0] = msg_uwb.pose.orientation.x; // anchor[0]
    RawDistUWB[1] = msg_uwb.pose.orientation.y; // anchor[1]
    RawDistUWB[2] = msg_uwb.pose.orientation.z; // anchor[2]
    RawDistUWB[3] = msg_uwb.pose.orientation.w; // anchor[3]

    // ROS_INFO("%f %f %f\n", HeaderUWB[0], HeaderUWB[1], HeaderUWB[2]);
    // ROS_INFO("%f %f %f %f\n", DistUWB[0], DistUWB[1], DistUWB[2], DistUWB[3]);
}

// uwb all station each other suabscibe request
void uwbBaseCallback(geometry_msgs::PoseStamped msg_uwb)
{
    MaskUWBBase = msg_uwb.pose.position.z;
    BaseDistUWB[0] = msg_uwb.pose.orientation.x; // 0
    BaseDistUWB[1] = msg_uwb.pose.orientation.y; // base 0-1
    BaseDistUWB[2] = msg_uwb.pose.orientation.z; // base 0-2
    BaseDistUWB[3] = msg_uwb.pose.orientation.w; // base 1-2
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
    TypeRTK = msg_gnss.poae.position.x;
    PosRTK[0] = msg_gnss.pose.orientation.x;
    PosRTK[1] = msg_gnss.pose.orientation.y;
    PosRTK[2] = msg_gnss.pose.orientation.z;
}

// gps tracktrue and speed subscribe request
void stateCallback(geometry_msgs::PoseStamped msg_state)
{
    TrackTrue = msg_state.pose.orientation.z;
    SpeedRTK = msg_state.pose.orientation.w;
}

// rtk station ECEF-frame
void stationCallback(geometry_msgs::PointStamped msg_station)
{
    stationRTK[0] = msg_station.point.x;
    stationRTK[1] = msg_station.point.y;
    stationRTK[2] = msg_station.point.z;
}

// rtk heading
void headingCallback(geometry_msgs::PointStamped msg_heading)
{
    headRTK = msg_heading.point.z;
}

// odom message
void odomCallback(geometry_msgs::Twist msg_Odom)
{
    shiftOdom[0] = msg_Odom.angular.x;
    shiftOdom[1] = msg_Odom.angular.y;
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

    ros::Subscriber pose_sub = nl.subscribe("/imu_data", 100, poseCallback);
    ros::Subscriber uwb_raw_sub = nl.subscribe("/uwb_raw", 100, uwbRawCallback);
    ros::Subscriber uwb_base_sub = nl.subscribe("/uwb_base", 100, uwbBaseCallback);
    ros::Subscriber mag_sub = nl.subscribe("/mag_data", 100, magCallback);
    ros::Subscriber gnss_sub = nl.subscribe("/gnss_position", 100, gnssCallback);
    ros::Subscriber state_pub = nl.subscribe("/gnss_state", 100, stateCallback);
    ros::Subscriber station_pub = nl.subscribe("/ecef_station", 100, stationCallback);
    ros::Subscriber heading_pub = nl.subscribe("/rtk_yaw", 100, headingCallback);
    ros::Subscriber odom_pub = nl.subscribe("/Odom", 100, odomCallback);

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
            // uwb tag to anchor mask
            DataBuffer[BufCnt].rawMaskUWB = MaskUWBRaw;
            // uwb tag to anchor distance
            DataBuffer[BufCnt].rawDistance[0] = RawDistUWB[0];
            DataBuffer[BufCnt].rawDistance[1] = RawDistUWB[1];
            DataBuffer[BufCnt].rawDistance[2] = RawDistUWB[2];
            DataBuffer[BufCnt].rawDistance[3] = RawDistUWB[3];
            // uwb anchor each other mask
            DataBuffer[BufCnt].baseMaskUWB = MaskUWBBase;
            // uwb anchor each other distance
            DataBuffer[BufCnt].baseDistance[0] = BaseDistUWB[0];
            DataBuffer[BufCnt].baseDistance[1] = BaseDistUWB[1];
            DataBuffer[BufCnt].baseDistance[2] = BaseDistUWB[2];
            DataBuffer[BufCnt].baseDistance[3] = BaseDistUWB[3];
            // rtk position in earth frame
            DataBuffer[BufCnt].rtkType = TypeRTK;
            DataBuffer[BufCnt].rtkPos[0] = PosRTK[0];
            DataBuffer[BufCnt].rtkPos[1] = PosRTK[1];
            DataBuffer[BufCnt].rtkPos[2] = PosRTK[2];
            // gps track true and speed
            DataBuffer[BufCnt].rtkTrackTrue = TrackTrue;
            DataBuffer[BufCnt].rtkSpeed = SpeedRTK;
            // rtk station coordinate in ECEF-frame
            DataBuffer[BufCnt].rtkStation[0] = stationRTK[0];
            DataBuffer[BufCnt].rtkStation[1] = stationRTK[1];
            DataBuffer[BufCnt].rtkStation[2] = stationRTK[2];
            // rtk heading
            DataBuffer[BufCnt].rtkHead = headRTK;
            // odom shift
            DataBuffer[BufCnt].odomShift[0] = shiftOdom[0]; // left wheel
            DataBuffer[BufCnt].odomShift[1] = shiftOdom[1]; // right wheel
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

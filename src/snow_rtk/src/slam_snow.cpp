//ros
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/UInt8.h"
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

// map
#include <nav_msgs/OccupancyGrid.h>

#include  "navigateFunction.cpp"
#include  "slamFunction.cpp"

#define remainWidth 2.0

using namespace std;

_YAT_POINTF_THETA robotPose;

POINT_XYZ baseECEF,baseBLH,pointBLH;
bool getStationPoint = false;
bool getRobotPose = false;
bool finishMaping = false;

vector<YAT_POINTF> boundaryPoint(10000);
int boundaryPointNum = 0;

vector<YAT_POINTF> mapEdgePoint(5000);
int mapEdgeNum = 0;

vector<int> gridPoint(1000000);
vector<int> bumpType(4);

void getBoundaryPoint()
{
    std::string data_path = "/home/rock/catkin_ws/src/snow_rtk/data/boundary/";
    // std::string data_path = "/home/rasp008/mower_ws/src/snow_rtk/data/boundary/";
    string file_path = data_path + "boundary.txt";

    std::ifstream boundaryStream(file_path.c_str(), ios::in);
    std::string line;

    //path data loading
    if(!boundaryStream.is_open()) ROS_INFO("Can not open boundary.txt.");

    vector<double> digitalData(8);
    POINT_XYZ pointXYZ;

    while(!boundaryStream.eof() && getline(boundaryStream, line))
    {
        char *stringData,*startLine;
        startLine = &(line[0]);
        int dataNum = 0;

        while ((stringData = strtok(startLine," ")) != NULL && dataNum < 8)
        {
            if ( strspn(stringData, "0123456789.-") == strlen(stringData) )
            {
                digitalData[dataNum] = atof(stringData);
            }
            else
            {
                break;
            }

            // ROS_INFO("boundary point NO.%d data is %f ,", dataNum, digitalData[dataNum]);

            startLine = NULL;
            dataNum++;
        }

        if(dataNum == 8)
        {
            // POINT_XYZ boundaryBLH;
            // boundaryBLH.x = digitalData[3];
            // boundaryBLH.y = digitalData[4];
            // boundaryBLH.z = digitalData[5];

            // POINT_XYZ boundaryXYZ = BLHtoECEF(boundaryBLH);
            // POINT_XYZ boundaryENU = getCoordinatesENU(boundaryXYZ,baseECEF,baseBLH);

            POINT_XYZ boundaryENU;
            boundaryENU.x = digitalData[0];
            boundaryENU.y = digitalData[1];
            boundaryENU.z = digitalData[2];

            if( fabs(digitalData[6] - 1 ) < 0.001 )
            {
                boundaryPoint[boundaryPointNum].x = boundaryENU.x;
                boundaryPoint[boundaryPointNum].y = boundaryENU.y;
                boundaryPointNum ++;
                ROS_INFO("boundary point ENU: x = %f , y = %f, z = %f", boundaryENU.x, boundaryENU.y, boundaryENU.z);
            }
            else
            {
                ROS_INFO("boundary point enable flag is %d ,", (int)(digitalData[6]) );
            }
        }
    }

    boundaryStream.close();
}

YAT_POINT findFirstEdgePoint(gridMapType parametre,YAT_POINT currentEdge)
{
    YAT_POINT lastEdge;
    YAT_POINT nextEdge;

    vector<YAT_POINT> moveVector(8);

    moveVector[0].x = 0;
    moveVector[0].y = -1;

    moveVector[1].x = 1;
    moveVector[1].y = -1;

    moveVector[2].x = 1;
    moveVector[2].y = 0;

    moveVector[3].x = 1;
    moveVector[3].y = 1;

    moveVector[4].x = 0;
    moveVector[4].y = 1;

    moveVector[5].x = -1;
    moveVector[5].y = 1;

    moveVector[6].x = -1;
    moveVector[6].y = 0;

    moveVector[7].x = -1;
    moveVector[7].y = -1;

    for(int k = 0 ; k < 8; k++)
    {
        lastEdge.x = currentEdge.x + moveVector[k].x;
        lastEdge.y = currentEdge.y + moveVector[k].y;
        
        nextEdge.x = currentEdge.x + moveVector[(k+1) % 8].x;
        nextEdge.y = currentEdge.y + moveVector[(k+1) % 8].y;

        int lastData = gridPoint[lastEdge.x + lastEdge.y * parametre.rangeX];
        int nextData = gridPoint[nextEdge.x + nextEdge.y * parametre.rangeX];

        if((lastData != ID_EDGE)&&(nextData == ID_EDGE))
        {
            break;
        }
    }

    return nextEdge;
}

// detect the out of Bounds
void getOuterRegion(gridMapType parametre)
{
    Eigen::MatrixXi connectedRegion(parametre.rangeX,parametre.rangeY);
    connectedRegion.setZero(parametre.rangeX,parametre.rangeY);

    connectedRegion(0,0) = ID_OUTER; // point (0,0) is in outer

    ROS_ERROR("calculate connected Region.");

    YAT_POINT startEdge;
    YAT_POINT testEdge;
    bool findStartEdge = false;

    for(int j = 0 ; j < parametre.rangeY; j++)
    {
        for(int i = 0 ; i < parametre.rangeX; i++)
        {
            if(!findStartEdge)
            {
                if(gridPoint[i + j * parametre.rangeX] == ID_EDGE)
                {
                    startEdge.x = i;
                    startEdge.y = j;
                    testEdge = findFirstEdgePoint(parametre,startEdge);
                    findStartEdge = true;
                }
                // ROS_INFO("find the first edge point, the grid point (%d,%d) is %d,",i,j,gridPoint[i + j * parametre.rangeX]);
            }
        }
    }

    ROS_ERROR("calculate connected Region 1 .");

    while((startEdge.x != testEdge.x)||(startEdge.y != testEdge.y))
    {
        for(int k = 0 ; k < 4; k++)
        {
            YAT_POINT moveVector;
            switch(k)
            {
                case 0:
                    moveVector.x = -1;
                    moveVector.y = 0;
                    break;
                case 1:
                    moveVector.x = 0;
                    moveVector.y = -1;
                    break;
                case 2:
                    moveVector.x = 1;
                    moveVector.y = 0;
                    break;
                case 3:
                    moveVector.x = 0;
                    moveVector.y = 1;
                    break;                            
                default:
                    ROS_ERROR("error move vector index.");
            }
            
            YAT_POINT detectXY;
            detectXY.x = testEdge.x + moveVector.x;
            detectXY.y = testEdge.y + moveVector.y;
            if(detectPointXY(parametre,detectXY))
            {
                if(gridPoint[detectXY.x + detectXY.y * parametre.rangeX] != ID_EDGE)
                {
                    connectedRegion(detectXY.x,detectXY.y) = ID_OUTER;
                }
            }
        }

        testEdge = findFirstEdgePoint(parametre,testEdge);

        // ROS_INFO("test point: x = %d, y = %d",testEdge.x,testEdge.y);
    }

    ROS_ERROR("calculate connected Region 2 .");

    for(int j = 0 ; j < parametre.rangeY; j++)
    {
        for(int i = 0 ; i < parametre.rangeX; i++)
        {
            if(gridPoint[i + j * parametre.rangeX] != ID_EDGE)
            {
                for(int k = 0 ; k < 4; k++)
                {
                    YAT_POINT moveVector;
                    switch(k)
                    {
                        case 0:
                            moveVector.x = -1;
                            moveVector.y = 0;
                            break;
                        case 1:
                            moveVector.x = 0;
                            moveVector.y = -1;
                            break;
                        case 2:
                            moveVector.x = 1;
                            moveVector.y = 0;
                            break;
                        case 3:
                            moveVector.x = 0;
                            moveVector.y = 1;
                            break;                            
                        default:
                            ROS_ERROR("error move vector index.");
                    }
                    
                    YAT_POINT detectXY;
                    detectXY.x = i + moveVector.x;
                    detectXY.y = j + moveVector.y;
                    if(detectPointXY(parametre,detectXY))
                    {
                        if(connectedRegion(detectXY.x,detectXY.y) == ID_OUTER)
                        {
                            connectedRegion(i,j) = ID_OUTER;
                        }
                    }
                    // ROS_INFO("the detect point (%d,%d) is %d.",i,j,gridMapMatrix(i,j));
                }
            }
            // ROS_INFO("finish detect the (%d,%d) point.",i,j);
        }
    }

    ROS_ERROR("calculate connected Region 2 .");

    for(int j = 0 ; j < parametre.rangeY; j++)
    {
        for(int i = 0 ; i < parametre.rangeX; i++)
        {
            if(connectedRegion(i,j) == ID_OUTER)
            {
                gridPoint[i + j * parametre.rangeX] = ID_OUTER;
            }
        }
    }

    ROS_INFO("finish initialization outer of map.");
}

void getPlannerMap(gridMapType parametre)
{
    Map plannerMap;
	plannerMap.data = NULL;

    plannerMap.height = parametre.rangeY;
    plannerMap.width = parametre.rangeX;
    plannerMap.scale = parametre.scale;
    plannerMap.xoffset = - parametre.offsetX;
    plannerMap.yoffset = - parametre.offsetY;

    plannerMap.data = (unsigned char *)malloc(plannerMap.width * plannerMap.height * sizeof(unsigned char));

	if (!plannerMap.data)
	{
		ROS_ERROR("can not create map data");
	}

	memset(plannerMap.data, -1, plannerMap.width * plannerMap.height * sizeof(unsigned char));

    for(int i = 0; i < plannerMap.height; i++)
    {
        for(int j = 0; j < plannerMap.width; j++)
        {
            int mapData = gridPoint[i * plannerMap.width + j];
            if(mapData == ID_EDGE)
            {
                plannerMap.data[i * plannerMap.width + j] = 100;
            }
            else
            {
                plannerMap.data[i * plannerMap.width + j] = 0;
            }
            // ROS_INFO("planner map: the (%d,%d) data = %d.",j,i,gridPoint[i * plannerMap.width + j]);
        }
    }

    dilateMap(&plannerMap,0.1);

    for(int i = 0; i < plannerMap.height; i++)
    {
        for(int j = 0; j < plannerMap.width; j++)
        {
            if(plannerMap.data[i * plannerMap.width + j] == 100) gridPoint[i * plannerMap.width + j] = ID_EDGE;
            // ROS_INFO("planner map: the (%d,%d) data = %d.",j,i,gridPoint[i * plannerMap.width + j]);
        }
    }
}

void getMapEdge(gridMapType parametre)
{
    double unitWidth = 0.4 / parametre.scale;

    for(int k = 0; k < boundaryPointNum; k++)
    {
        YAT_POINTF pointA = boundaryPoint[k];
        YAT_POINTF pointB = boundaryPoint[(k + 1) % boundaryPointNum];
        int deltaNum = ceilValue( deltaDistance(pointA,pointB) / unitWidth);

        vector<YAT_POINTF> densePoint = densePosition(pointA,pointB,deltaNum);
        for(int i = 0; i < deltaNum; i++)
        {
            mapEdgePoint[mapEdgeNum] = densePoint[i];
            mapEdgeNum ++;
        }
    }
}

void updateBoundary(gridMapType parametre)
{
    for(int k = 0; k < mapEdgeNum; k++)
    {
        YAT_POINT pointXY = worldToRaster(mapEdgePoint[k],parametre);
        int i = pointXY.y;
        int j = pointXY.x;
        if(detectPointXY(parametre,pointXY))
        {
            gridPoint[i * parametre.rangeX + j] = ID_EDGE; // 0 is enable; 100 is obstracle; - 1 is unknow
        }
        else
        {
            ROS_ERROR("invail grid point.");
        }
    }
}

void initialGridMap(gridMapType &parametre)
{
    parametre.scale = 10;

    YAT_POINTF maxPoint = maxValue(boundaryPoint,boundaryPointNum);
    YAT_POINTF minPoint = minValue(boundaryPoint,boundaryPointNum);
 
    ROS_INFO("grid map -> max: x = %f, y = %f.",maxPoint.x,maxPoint.y);
    ROS_INFO("grid map -> min: x = %f, y = %f.",minPoint.x,minPoint.y);

    parametre.rangeX = ceilValue( (maxPoint.x - minPoint.x + 2 * remainWidth) * parametre.scale );
    parametre.rangeY = ceilValue( (maxPoint.y - minPoint.y + 2 * remainWidth) * parametre.scale );

    parametre.offsetX = minPoint.x - remainWidth;
    parametre.offsetY = minPoint.y - remainWidth;

    ROS_INFO("grid map : width = %d, height = %d.",parametre.rangeX,parametre.rangeY);

    // initial the total grid map
    for(int i = 0; i < parametre.rangeY; i++)
    {
        for(int j = 0; j < parametre.rangeX; j++)
        {
            gridPoint[i * parametre.rangeX + j] = ID_REMAIN;
        }
    }
    
    // get map edge point
    getMapEdge(parametre);

    // record the boundary point
    updateBoundary(parametre);

    // get the dilate map
    getPlannerMap(parametre);

    // get the region of outer 
    getOuterRegion(parametre);
}

void updateGridMap(gridMapType parametre)
{
    // calcute the bump point
    vector<YAT_POINTF> robotBody(81);
    for(int i = 2; i <= 6; i++)
    {
        for(int j = -2; j <= 2; j++)
        {
            double deltaXb = 0.05 * j;
            double deltaYb = 0.05 * i;
            robotBody[(i - 2) * 5 + (j + 2)].x = robotPose.x + deltaXb * sin(robotPose.theta) + deltaYb * cos(robotPose.theta);
            robotBody[(i - 2) * 5 + (j + 2)].y = robotPose.y - deltaXb * cos(robotPose.theta) + deltaYb * sin(robotPose.theta);
        }
    }

    // record the robot body point
    for(int k = 0; k < 25; k++)
    {
        YAT_POINT pointXY = worldToRaster(robotBody[k],parametre);
        int i = pointXY.y;
        int j = pointXY.x;
        if(detectPointXY(parametre,pointXY))
        {
            gridPoint[i * parametre.rangeX + j] = 0; // 0 is enable; 100 is obstracle; - 1 is unknow
        }
        else
        {
            ROS_ERROR("invail grid point.");
        }
    }

    // calcute the bump point and record the bump point
    vector<YAT_POINTF> bumpPoint(9);
    for(int k = - 1; k <= 1; k++)
    {
        bumpPoint[k + 1].x = robotPose.x + 0.05 * k * sin(robotPose.theta) + 0.05 * 7 * cos(robotPose.theta);
        bumpPoint[k + 1].y = robotPose.y - 0.05 * k * cos(robotPose.theta) + 0.05 * 7 * sin(robotPose.theta);
        
        YAT_POINT pointXY = worldToRaster(bumpPoint[k + 1],parametre);
        int i = pointXY.y;
        int j = pointXY.x;

        if(detectPointXY(parametre,pointXY))
        {
            // gridPoint[i * parametre.rangeX + j] = (bumpType[0] == 1)?100:0; // 0 is enable; 100 is obstracle; - 1 is unknow
        }
        else
        {
            ROS_ERROR("invail grid point.");
        }
    }

    // record the boundary point
    updateBoundary(parametre);
}

//show map in rviz
nav_msgs::OccupancyGrid postOccupancyMap(gridMapType parametre)
{
    // ROS_INFO("Creating map rviz");
    nav_msgs::OccupancyGrid ocmap;
    ocmap.header.frame_id = "mowerMap";
    ocmap.header.stamp = ros::Time::now();

    ocmap.info.height = parametre.rangeY;
    ocmap.info.width = parametre.rangeX;
    ocmap.info.resolution = 1.0 / parametre.scale;
    ocmap.info.origin.position.x = parametre.offsetX;
    ocmap.info.origin.position.y = parametre.offsetY;

    ocmap.data.resize(ocmap.info.width * ocmap.info.height);

    for(int i = 0; i < ocmap.info.height; i++)
    {
        for(int j = 0; j < ocmap.info.width; j++)
        {
            ocmap.data[i * ocmap.info.width + j] = gridPoint[i * ocmap.info.width + j];
        }
    }

    // ROS_INFO("ocmap data : hight = %d , width = %d .",ocmap.info.height, ocmap.info.width);

    return ocmap;
}

void baseStationCallback(const geometry_msgs::PointStampedConstPtr& msg_station)
{
    static ros::Time prev_time = ros::Time::now();
    double dt = ros::Time::now().toSec() - prev_time.toSec();

    baseECEF.x = msg_station->point.x;
    baseECEF.y = msg_station->point.y;
    baseECEF.z = msg_station->point.z;

    // ROS_INFO("get the base station point: x = %f, y = %f, z = %f",baseECEF.x,baseECEF.y,baseECEF.z);

    baseBLH = ECEFtoBLH(baseECEF);

    ROS_INFO("slam_rtk get the base station point: x = %f, y = %f, z = %f",baseBLH.x,baseBLH.y,baseBLH.z);

    getStationPoint = true;
}

void ekfPositionCallback(geometry_msgs::PoseStamped msg_ekf)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    
    robotPose.x = msg_ekf.pose.position.x;
    robotPose.y = msg_ekf.pose.position.y;
    robotPose.theta = msg_ekf.pose.orientation.z; //(-PI,PI]

    // ROS_INFO("slam_rtk get the ekf position: x = %f, y = %f, z = %f",robotPose.x,robotPose.y,robotPose.theta);

    getRobotPose = true;
}

geometry_msgs::PoseStamped robotPoseTopic()
{
    geometry_msgs::PoseStamped PositionPub;

    PositionPub.header.stamp = ros::Time::now();
    PositionPub.header.frame_id = "robotPose";

    PositionPub.pose.position.x = robotPose.x;
    PositionPub.pose.position.y = robotPose.y;
    PositionPub.pose.position.z = 0;

    Eigen::Vector3d robotEular(0,0,robotPose.theta);
    Eigen::Quaterniond Qbn = euler2Quaternion(robotEular);

    PositionPub.pose.orientation.x = Qbn.x();
    PositionPub.pose.orientation.y = Qbn.y();
    PositionPub.pose.orientation.z = Qbn.z();
    PositionPub.pose.orientation.w = Qbn.w();

    return PositionPub;
}

void finishMapCallback(std_msgs::Bool msg_map)
{
    if(msg_map.data) finishMaping = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_loose");
    ROS_INFO_STREAM("grid map process is starting...");

    ros::NodeHandle nh;

    // Publisher
    ros::Publisher occmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("/grid_map", 1, true);
    ros::Publisher robotPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_pose", 1, true);

    // Subcriber
    // ros::Subscriber map_sub = nh.subscribe("/maping_finish", 1, finishMapCallback);
    ros::Subscriber ekf_sub = nh.subscribe("/ekf_pose", 1, ekfPositionCallback);
    ros::Subscriber baseStation_sub = nh.subscribe("ecef_station", 1, baseStationCallback);

    //Synchronizer
    bool initialMap = false;

    // ROS_INFO("slam_rtk get the base station point: x = %f, y = %f, z = %f",baseBLH.x,baseBLH.y,baseBLH.z);

    gridMapType parametre;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        // inital the grid map
        if(!initialMap)
        {
            if(getStationPoint)
            {
                getBoundaryPoint();

                ROS_INFO("finish read boundary point");

                initialGridMap(parametre);
                
                ROS_INFO("initial the grid map");

                initialMap = true;
            }

            ROS_INFO("waiting for initial rtk_slam");
        }
        else 
        {
            if(getRobotPose)
            {
                updateGridMap(parametre);

                nav_msgs::OccupancyGrid postMap = postOccupancyMap(parametre);
                occmap_pub.publish(postMap);

                geometry_msgs::PoseStamped PositionPub = robotPoseTopic();
                robotPose_pub.publish(PositionPub);

                getRobotPose = false;
            }
            else
            {
                nav_msgs::OccupancyGrid postMap = postOccupancyMap(parametre);
                occmap_pub.publish(postMap);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    // status_pointger->flush();
    spdlog::drop_all();
    return 0;
}


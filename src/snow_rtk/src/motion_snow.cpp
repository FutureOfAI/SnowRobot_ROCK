//ros
#include "ros/ros.h"
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

//Eigen
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>

// map
#include <nav_msgs/OccupancyGrid.h>

// geometry function
#include  "navigateFunction.cpp"
#include  "slamFunction.cpp"

#include "astar.cpp"

using namespace std;

std::string getDate()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d-%H-%M-%S",localtime(&timep) );
    return tmp;
}

/************************************************************************************/
std::string point_path = "/home/rock/catkin_ws/src/snow_rtk/data/";
// std::string point_path = "/home/yat/catkin_ws/src/gps_zigzag/data/";
std::string file_log = point_path + getDate() + "motion2.txt";
std::ofstream fout_point(file_log.c_str());

bool stopTask = false;
bool getBumpTopic = false;
vector<int> bumpType(4);

nav_msgs::OccupancyGrid gridMap;
nav_msgs::OccupancyGrid pathWay; // define parametre of grid map
Eigen::Matrix2d actionZone(2,2); 

bool refreshPID = false;
float W_threshold = 0.5;
float V_threshold = 0.25;
// float kp = 0.5, ki = 0, kd = 0;

poseType robotPose;
poseType startPoint;
YAT_POINT originXY;

geometry_msgs::Twist msg_vel;
ros::Publisher vel_pub;
ros::Publisher target_pub;


ros::Time updateMapTime;
ros::Time updatePoseTime;
bool updateMapFlag = false;
bool updateRobotPose = false;
bool autoMode = false;

enum {WAIT, FORWARD, TURN, BACK};
int controlState = FORWARD;
/********************************  motion control  *******************************/

YAT_POINTF move_forward(poseType targetPoint, poseType lastPoint)
{   
    // angle control
    float pathDistance = deltaDistance(targetPoint , lastPoint); 

    Eigen::Vector2d pathVector( (targetPoint.x - lastPoint.x)/pathDistance, (targetPoint.y - lastPoint.y)/pathDistance );
    Eigen::Vector2d normalVector(pathVector(1), -pathVector(0)); // clockwise vector
    Eigen::Vector2d targetVector(targetPoint.x - robotPose.x, targetPoint.y - robotPose.y);

    float trajectoryDistance = distanceToAB(targetPoint , lastPoint, robotPose);  // left is positive; right is negative;
    float PathAngle = atan2( targetPoint.y - lastPoint.y, targetPoint.x - lastPoint.x);

    // if boundry mode, limit velocity
    double deadLineDistance = targetVector.transpose() * pathVector;
    spdlog::get("robot_state")->info("deadLineDistance:{}, trajectoryDistance:{}.",deadLineDistance, trajectoryDistance);

    float aimLength = 0.5;
 
    float controlAngle = - 0.5 * PI * sign( trajectoryDistance );
    if( fabs( trajectoryDistance ) < aimLength )
    {
        controlAngle = - asin( trajectoryDistance / aimLength );
    }

    float targetAngle = constrainAngle(controlAngle + PathAngle);
    float controlObject = deltaAngle(targetAngle,robotPose.theta);  // - odomData.pose.orientation.z;

    YAT_POINTF commandVelocity;

    // horizontal moving, limit velocity
    commandVelocity.x = 0.5 * deadLineDistance;    
    if( fabs(commandVelocity.x) > V_threshold)
    {
         commandVelocity.x = V_threshold * sign(commandVelocity.x);
    }
    else if( fabs(commandVelocity.x) < 0.1)
    {
         commandVelocity.x = 0.1 * sign(commandVelocity.x);
    }
    commandVelocity.x = commandVelocity.x * cos(controlObject);
    
    /************************************************************************************************************************/

    static double last_controlObject = controlObject;
    static double conObjectInt = 0;
    static double control_V_start = 0;

    if(refreshPID)
    {
        last_controlObject = controlObject;
        conObjectInt = 0;
        control_V_start = 0;

        refreshPID = false;
    }

    if( fabs(controlObject) < 5 * PI / 180 )
    {
        conObjectInt = conObjectInt + controlObject;
    }
    commandVelocity.y = controlPID(controlObject, conObjectInt, last_controlObject,2.4 / PI, 0, 0);
    last_controlObject = controlObject;

    // horizontal moving, limit angle velocity
    // commandVelocity.y = controlObject * 0.9 / PI;
    if (fabs(commandVelocity.y) > W_threshold) commandVelocity.y = W_threshold * sign( commandVelocity.y );

    // constrain the output of linear control, constrain the min turning radius
    // need acceleration from scratch
    if(deadLineDistance < 0.06)
    {
        refreshPID = true;
    }

    return commandVelocity;
}

YAT_POINTF move_back(poseType targetPoint, poseType lastPoint)
{
    Eigen::Vector2d pathVector(0,0);
    Eigen::Vector2d targetVector(0,0);

    float pathDistance = deltaDistance( targetPoint , lastPoint );

    pathVector(0) = ( targetPoint.x - lastPoint.x ) / pathDistance;
    pathVector(1) = ( targetPoint.y - lastPoint.y ) / pathDistance;
    
    targetVector(0) = targetPoint.x - robotPose.x;
    targetVector(1) = targetPoint.y - robotPose.y;

    float deltaDistance = ( pathVector.transpose() )*targetVector;

    // spdlog::get("robot_state")->info("delta distance is :{}.",deltaDistance);

    float pathAngle = atan2(pathVector(1) , pathVector(0));
    float error = deltaAngle(pathAngle, robotPose.theta);

    double control_v = deltaDistance*sign( cos(error) ); 
    if( fabs(control_v) > 0.5*V_threshold )
    {
        control_v = 0.5*sign(control_v)*V_threshold;
    }

    if( fabs(control_v) < 0.15 )
    {
        control_v = sign(control_v)*0.15;
    }

    YAT_POINTF commandVelocity;

    commandVelocity.x = control_v; 
    commandVelocity.y = 0;    

    return commandVelocity;
}

YAT_POINTF turning(poseType targetPoint, poseType lastPoint)
{
    float targetAngle = atan2(targetPoint.y - lastPoint.y , targetPoint.x - lastPoint.x);
    float  deltaTheta = deltaAngle(targetAngle, robotPose.theta);

    YAT_POINTF commandVelocity;

    commandVelocity.x = 0;
    commandVelocity.y = deltaTheta * 1.8 / PI;

    if(fabs(commandVelocity.y)<0.2)
    {
        commandVelocity.y = 0.2*sign(deltaTheta);
    }
    else if(fabs(commandVelocity.y)>0.8)
    {
        commandVelocity.y = 0.8*sign(deltaTheta);
    }

    return commandVelocity;
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

bool turnedTarget(poseType targetPoint, poseType lastPoint,float turnDoneAngle)
{
    float targetAngle = atan2( targetPoint.y - lastPoint.y , targetPoint.x - lastPoint.x );
    bool turnedFlag = ( fabs( deltaAngle( targetAngle , robotPose.theta ) ) < degreeToRad(turnDoneAngle) ) ?true:false;

    return turnedFlag;
}

bool arrivedTarget(poseType targetPoint, poseType lastPoint,float moveDoneLength)
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

/************************************************************************************/
gridMapType getGridParametre(nav_msgs::OccupancyGrid data)
{
    gridMapType parametre;

    parametre.scale = (int)(1/(data.info.resolution));
    parametre.rangeX = data.info.width;
    parametre.rangeY = data.info.height;
    parametre.offsetX = data.info.origin.position.x;
    parametre.offsetY = data.info.origin.position.y;

    return parametre;
}

int getAlongSide(float targetAngle)
{
    if( fabs(targetAngle + PI/2) < PI/4 )
    {
        return 0;
    }
    else if( fabs(targetAngle) <= PI/4 )
    {
        return 1;
    }
    else if( fabs(targetAngle - PI/2) < PI/4 )
    {
        return 2;
    }
    else
    {
        return 3;
    }
}

bool findOriginPoint()
{
    bool findResult = false;

    gridMapType parametre = getGridParametre(gridMap);

    poseType OriginPose = robotPose;
    OriginPose.x += 0.2 * cos( robotPose.theta );
    OriginPose.y += 0.2 * sin( robotPose.theta );
    OriginPose.theta = constrainAngle(getAlongSide(robotPose.theta) * PI/2 - PI/2);

    startPoint = OriginPose;

    int lastData = -1;

    for(int i = 1; i < parametre.rangeY; i++)
    {
        YAT_POINT pointXY;
        pointXY.x = (int)(0.5 + (OriginPose.x - parametre.offsetX) * parametre.scale);
        pointXY.y = i;

        if(detectPointXY(parametre,pointXY))
        {
            int currentData = gridMap.data[i * parametre.rangeX + pointXY.x];
            if(currentData <= ID_REMAIN)
            {
                originXY = pointXY;
                ROS_INFO("find the origin XY: x = %d, y = %d.",originXY.x,originXY.y);
                YAT_POINTF originPoint = rasterToWorld(pointXY,parametre);
                ROS_INFO("find the origin point: x = %f, y = %f.",originPoint.x,originPoint.y);
                findResult = true;
                break;
            }
            lastData = currentData;
        }
        else
        {
            ROS_ERROR("invail origin parametre.");
            break;
        }
    }

    return findResult;
}

Map getPlannerMap()
{
    Map plannerMap;
	plannerMap.data = NULL;

    plannerMap.height = gridMap.info.height;
    plannerMap.width = gridMap.info.width;
    plannerMap.scale = (int)(1.0 / gridMap.info.resolution);
    plannerMap.xoffset = - gridMap.info.origin.position.x;
    plannerMap.yoffset = - gridMap.info.origin.position.y;

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
            int mapData = gridMap.data[i * gridMap.info.width + j];
            plannerMap.data[i * plannerMap.width + j] = (mapData < 0) ? 50 : mapData;
        }
    }

    dilateMap(&plannerMap,0.2);

    return plannerMap;
}

Eigen::MatrixXi getGridMapMatrix(Map plannerMap)
{
    Eigen::MatrixXi mapMatrix(plannerMap.width, plannerMap.height);

    for(int i = 0; i < plannerMap.height; i++)
    {
        for(int j = 0; j < plannerMap.width; j++)
        {
            mapMatrix(j,i) = plannerMap.data[i * plannerMap.width + j];
        }
    }

    return mapMatrix;
}

bool findRemainEdgePoint(int motionType, YAT_POINT &remainPointXY,Map plannerMap)
{
    gridMapType gridParametre = getGridParametre(gridMap);
    Eigen::MatrixXi plannerMapMatrix = getGridMapMatrix(plannerMap);

    // find the point
    bool findRemainPoint = false;
    YAT_POINT searchPoint = poseToRaster(startPoint,gridParametre); // search X and Y in grid map
    searchPoint = constrainGridXY(gridParametre,searchPoint.x,searchPoint.y);

    double minDistance = fabs( (gridParametre.rangeX + gridParametre.rangeY) / gridParametre.scale );
    YAT_POINT searchXY = poseToRaster(startPoint,gridParametre); // search X and Y in grid map

    int searchSide = 0;
    switch ( motionType )
    {
        case 0:
            searchSide = 1;
            break;
        case 1:
            searchSide = 3;
            break;
        case 2:
            searchSide = 0;
            break;
        case 3:
            searchSide = 2;
            break;
        default:
            break;
    }

    switch (searchSide)
    {
        case 0: // search the downside
            for(int i = 0; i <= searchPoint.y; i++)
            {
                for(int j = 0; j < gridParametre.rangeX; j++)
                {
                    YAT_POINT currentPoint = constrainGridXY(gridParametre,j,i);
                    bool remainFlag = (plannerMapMatrix(j,i) == ID_REMAIN)?true:false;
                    double currentDist = deltaDistance(rasterToPose(currentPoint,gridParametre),robotPose);
                    bool isMinDist = (currentDist < minDistance)?true:false;
                    if( remainFlag && isMinDist)
                    {
                        minDistance = currentDist;
                        remainPointXY = currentPoint;
                        findRemainPoint = true;
                    }
                }
            }
            break;
        case 1: // search the rightside
            for(int i = gridParametre.rangeX - 1; i >= searchPoint.x; i--)
            {
                for(int j = 0; j < gridParametre.rangeY; j++)
                {
                    YAT_POINT currentPoint = constrainGridXY(gridParametre,i,j);
                    bool remainFlag = (plannerMapMatrix(i,j) == ID_REMAIN)?true:false;
                    double currentDist = deltaDistance(rasterToPose(currentPoint,gridParametre),robotPose);
                    bool isMinDist = (currentDist < minDistance)?true:false;
                    if( remainFlag && isMinDist)
                    {
                        minDistance = currentDist;
                        remainPointXY = currentPoint;
                        findRemainPoint = true;
                    }
                }
            }
            break;
        case 2: // search the upside
            for(int i = gridParametre.rangeY - 1; i >= searchPoint.y; i--)
            {
                for(int j = gridParametre.rangeX - 1; j >= 0; j--)
                {
                    YAT_POINT currentPoint = constrainGridXY(gridParametre,j,i);
                    bool remainFlag = (plannerMapMatrix(j,i) == ID_REMAIN)?true:false;
                    double currentDist = deltaDistance(rasterToPose(currentPoint,gridParametre),robotPose);
                    bool isMinDist = (currentDist < minDistance)?true:false;
                    if( remainFlag && isMinDist)
                    {
                        minDistance = currentDist;
                        remainPointXY = currentPoint;
                        findRemainPoint = true;
                    }
                }
            }
            break;
        case 3: // search the leftside
            for(int i = 0; i <= searchPoint.x; i++)
            {
                for(int j = gridParametre.rangeY -1; j >= 0; j--)
                {
                    YAT_POINT currentPoint = constrainGridXY(gridParametre,i,j);
                    bool remainFlag = (plannerMapMatrix(i,j) == ID_REMAIN)?true:false;
                    double currentDist = deltaDistance(rasterToPose(currentPoint,gridParametre),robotPose);
                    bool isMinDist = (currentDist < minDistance)?true:false;
                    if( remainFlag && isMinDist)
                    {
                        minDistance = currentDist;
                        remainPointXY = currentPoint;
                        findRemainPoint = true;
                    }
                }
            }
            break;
        default:
            break;
    }

    if(findRemainPoint) ROS_INFO("the remain point XY is : x = %d , y = %d",remainPointXY.x,remainPointXY.y);

    return findRemainPoint;
}

YAT_POINT findNearestPoint(poseType detectPoint,Eigen::MatrixXi plannerMapMatrix)
{
    gridMapType gridParametre = getGridParametre(gridMap);

    float minDistance = deltaDistance(startPoint,detectPoint);
    YAT_POINT findPoint = poseToRaster(startPoint,gridParametre);

    for(int i = 0; i < gridParametre.rangeX; i++)
    {
        for(int j = 0; j < gridParametre.rangeY; j++)
        {
            YAT_POINT currentPoint = constrainGridXY(gridParametre,i,j);
            bool canPassFlag = (plannerMapMatrix(currentPoint.x,currentPoint.y) < 50)?true:false; 
            bool isMinDist = (deltaDistance(rasterToPose(currentPoint,gridParametre),detectPoint) < minDistance)?true:false;
            if( canPassFlag && isMinDist)
            {
                minDistance = deltaDistance(rasterToPose(currentPoint,gridParametre),detectPoint);
                findPoint = currentPoint;
            }
        }
    }

    return findPoint;
}

float getTargetAngle(int motionType, int motionSide)
{
    float targetAngle = 0;

    switch( motionType )
    {
        case 0:
            targetAngle += PI/2;
            break;
        case 1:
            targetAngle += -PI/2;
            break;
        case 2:
            targetAngle += 0;
            break;
        case 3:
            targetAngle += PI;
            break;
        default:
            break;
    }

    switch( motionSide )
    {
        case 0:
            targetAngle = constrainAngle(targetAngle);
            break;
        case 1:
            targetAngle = constrainAngle(targetAngle - PI/2);
            break;
        case 2:
            targetAngle = constrainAngle(targetAngle - PI);
            break;
        case 3:
            targetAngle = constrainAngle(targetAngle - PI/2);
            break;
        default:
            break;
    }

    return targetAngle;
}

int getMotionSide(int motionType, float targetAngle)
{
    int motionSide = 0;
    float initialAngle = 0;

    switch( motionType )
    {
        case 0:
            initialAngle = PI/2;
            break;
        case 1:
            initialAngle = -PI/2;
            break;
        case 2:
            initialAngle = 0;
            break;
        case 3:
            initialAngle = PI;
            break;
        default:
            break;
    }

    float deltaTheta = deltaAngle(constrainAngle(targetAngle), initialAngle);

    if( fabs(deltaTheta) < PI/4)
    {
        motionSide = 0;
    }
    else if( fabs(deltaTheta) > 3*PI/4 )
    {
        motionSide = 2;
    }
    else
    {
        motionSide = 1;
    }

    return motionSide;
}

// get node of the A star planner path
bool getNodePoint(vector<YAT_POINT> pointXY,int num,Map plannerMap)
{
    YAT_POINT maxXY = maxValue(pointXY,2);
    YAT_POINT minXY = minValue(pointXY,2);

    gridMapType gridParametre = getGridParametre(gridMap);
    Eigen::MatrixXi gridMapMatrix = getGridMapMatrix(plannerMap);

    poseType startPoint = rasterToPose(pointXY[0],gridParametre);
    poseType endPoint = rasterToPose(pointXY[num-1],gridParametre);

    bool findNode = false;

    for(int i = minXY.x; i <maxXY.x; i++)
    {
        for(int j = minXY.y; j <maxXY.y; j++)
        {
            YAT_POINT searchXY = constrainGridXY(gridParametre,i,j);
            poseType searchPoint = rasterToPose(searchXY,gridParametre);

            bool smallDistance = (fabs(distanceToAB(startPoint,endPoint,searchPoint)) < 0.1)?true:false;
            bool isEdgePoint = (fabs(gridMapMatrix(searchXY.x,searchXY.y)) > 50)?true:false;

            if( smallDistance && isEdgePoint )
            {
                findNode = true;
            }
        }
    }

    return findNode;
}

int getPlanPath(YAT_POINT pathEnd,YAT_POINT lastPoint,vector<YAT_POINTF> &pathPoint)
{
    gridMapType gridParametre = getGridParametre(gridMap);

    YAT_POINT pathStart = lastPoint;

    poseType targetPoint = rasterToPose(pathEnd,gridParametre);

    // convert type from OccupancyGrid to Map
    Map plannerMap = getPlannerMap();

    // search parametre
    AStar::Params param;
    param.width = plannerMap.width;
    param.height = plannerMap.height;
    param.corner = false;
    param.can_pass = [&](const AStar::Vec2 &pos)->bool
    {
         return plannerMap.data[pos.y * plannerMap.width + pos.x] <= 50;
    };

    // execute the search action
    BlockAllocator allocator;
    AStar algorithm(&allocator);

    param.start = AStar::Vec2(pathStart.x, pathStart.y);
    param.end = AStar::Vec2(pathEnd.x, pathEnd.y);

    std::vector<AStar::Vec2> pathWayPoint = algorithm.find(param); 

    int pathNum  = 0;

    YAT_POINT startNode = lastPoint;
    poseType endPointPose = targetPoint;

    if( pathWayPoint.size() > 0)
    {
        for(int i=0; i<pathWayPoint.size(); i ++)
        {
            vector<YAT_POINT> pointXY(2);
            pointXY[1].x = pathWayPoint[i].x;
            pointXY[1].y = pathWayPoint[i].y;
            pointXY[0] = startNode;

            if( getNodePoint(pointXY,2,plannerMap) && (i >= 1) )
            {
                YAT_POINT nodePoint;
                nodePoint.x = pathWayPoint[i-1].x;
                nodePoint.y = pathWayPoint[i-1].y;

                startNode = nodePoint;

                pathPoint[pathNum] = rasterToWorld(nodePoint,gridParametre);
                endPointPose = rasterToPose(nodePoint,gridParametre);

                pathNum++;
            }
        }
    }

    if( (pathNum < 0.5) || (deltaDistance(targetPoint,endPointPose) > 0.1) )
    {
        // if can arrive there straightly or the end plan point is far from the targetPoint
        pathPoint[pathNum].x = targetPoint.x;
        pathPoint[pathNum].y = targetPoint.y;
        pathNum++;
    }

    return pathNum;
}

bool straightMotionAtoB(poseType targetPoint,poseType lastPoint,YAT_POINTF &commandVelocity)
{
    static int motionStep = 0;
    bool motionFinish = false;

    static poseType begainPose = lastPoint;
    static poseType backPose = targetPoint;

    if( motionStep == 0)
    {
        refreshPID = true;
        motionStep = 1;
    }

    if( motionStep == 1 )
    {
        commandVelocity = turning(targetPoint,lastPoint);

        if(turnedTarget(targetPoint,lastPoint,5.0))
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
            motionStep = 3;
        }
    }

    if( motionStep == 3 )
    {
        commandVelocity = move_forward(targetPoint,lastPoint);

        if( (bumpType[0] > 0.5) )
        {
            motionStep = 4;
            spdlog::get("robot_state")->info("motion A to B: cannot move forward.");
        }

        if( arrivedTarget(targetPoint, lastPoint, 0.05) )
        {
            motionStep = 4;
        }
    }

    if( motionStep == 4 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(waitTimes(10))
        {
            motionStep = 0;
            motionFinish = true;
        }
    }

    spdlog::get("robot_state")->info("motion A to B: next Point -> x={}, y={}, theta={}.",targetPoint.x, targetPoint.y,targetPoint.theta);
    spdlog::get("robot_state")->info("motion A to B: step = {}, forward ={}, backward ={}, left={}, right={}.", motionStep, actionZone(0,0), actionZone(0,1), actionZone(1,0), actionZone(1,1));

    return motionFinish;
}

bool planMotionAtoB(poseType targetPoint,poseType lastPoint,YAT_POINTF &commandVelocity)
{
    static int motionStep = 0;
    bool motionFinish = false;

    static vector<YAT_POINTF> pathPoint(100);
    static int pointNum = 0;
    static int passedNum = 0;

    static poseType pathPointA = lastPoint;
    static poseType pathPointB = targetPoint;

    if( motionStep == 0 )
    {
        ros::Time currentTime = ros::Time::now();

        Map plannerMap = getPlannerMap();
        Eigen::MatrixXi plannerMapMatrix = getGridMapMatrix(plannerMap);

        pointNum = getPlanPath(findNearestPoint(targetPoint,plannerMapMatrix),findNearestPoint(lastPoint,plannerMapMatrix),pathPoint);

        float dt = ros::Time::now().toSec() - currentTime.toSec();

        spdlog::get("robot_state")->info("the A star algorithm cost time is: {}.", dt);

        for(int i = 0; i < pointNum; i++)
        {
            spdlog::get("robot_state")->info("NO.{} plan point is: x= {}, y={}.", i, pathPoint[i].x, pathPoint[i].y);
        }

        if(pointNum == 1)
        {
            pathPointA = lastPoint;
            pathPointB = targetPoint;
            motionStep = 2;
        }
        else
        {
            motionStep = 1;
        }

        passedNum = 0;
    }

    if( motionStep == 1 )
    {
        pathPointA.x = robotPose.x;
        pathPointA.y = robotPose.y;
        pathPointB.x = pathPoint[passedNum].x;
        pathPointB.y = pathPoint[passedNum].y;

        pathPointA.theta = atan2(pathPointB.y - pathPointA.y,pathPointB.x - pathPointA.x);
        pathPointB.theta = atan2(pathPointB.y - pathPointA.y,pathPointB.x - pathPointA.x);

        motionStep = 2;
    }

    if( motionStep == 2 )
    {
        if(straightMotionAtoB(pathPointB,pathPointA,commandVelocity))
        {
            motionStep = 1;

            spdlog::get("robot_state")->info("arrive NO.{} plan point is: x= {}, y={}.", passedNum, pathPoint[passedNum].x, pathPoint[passedNum].y);

            passedNum ++;

            if(passedNum >= pointNum)
            {
                motionFinish = true;

                motionStep = 0;
            }
        }
    }

    spdlog::get("robot_state")->info("plan A to B, the motionStep = {}, pointNum={}, passedNum={}.", motionStep, pointNum, passedNum);

    return motionFinish;
}

bool getRowPoint(YAT_POINT &nextXY,int alongSide,Map plannerMap)
{
    gridMapType parametre = getGridParametre(gridMap);
    Eigen::MatrixXi plannerMapMatrix = getGridMapMatrix(plannerMap);

    bool getNextPoint = false;
    YAT_POINT testPoint = nextXY; // search X and Y in grid map

    switch (alongSide)
    {
        case 0:
            for(int i = nextXY.y - 1; i >= 0; i--)
            {
                testPoint.y = i;
                int mapData = plannerMapMatrix(testPoint.x,testPoint.y);
                if( mapData <= ID_REMAIN )
                {
                    nextXY = testPoint;
                    getNextPoint = true;
                }
                if((mapData == ID_EDGE) && getNextPoint) break;
            }
            break;
        case 1:
            for(int i = nextXY.x + 1; i < parametre.rangeX; i++)
            {
                testPoint.x = i;
                int mapData = plannerMapMatrix(testPoint.x,testPoint.y);
                if( mapData <= ID_REMAIN )
                {
                    nextXY = testPoint;
                    getNextPoint = true;
                }
                if((mapData == ID_EDGE) && getNextPoint) break;
            }
            break;
        case 2:
            for(int i = nextXY.y + 1; i < parametre.rangeY; i++)
            {
                testPoint.y = i;
                int mapData = plannerMapMatrix(testPoint.x,testPoint.y);
                if( mapData <= ID_REMAIN )
                {
                    nextXY = testPoint;
                    getNextPoint = true;
                }
                if((mapData == ID_EDGE) && getNextPoint) break;
            }
            break;
        case 3:
            for(int i = nextXY.x - 1; i >= 0; i--)
            {
                testPoint.x = i;
                int mapData = plannerMapMatrix(testPoint.x,testPoint.y);
                if( mapData <= ID_REMAIN )
                {
                    nextXY = testPoint;
                    getNextPoint = true;
                }
                if((mapData == ID_EDGE) && getNextPoint) break;
            }
            break;
        default:
            break;
    }

    return getNextPoint; // if did not find the next point, return the edge point in the grid map 
}

bool getLinePoint(YAT_POINT &nextXY,int alongSide,Map plannerMap)
{
    gridMapType gridParametre = getGridParametre(gridMap); 
    Eigen::MatrixXi plannerMapMatrix = getGridMapMatrix(plannerMap);

    bool getNextPoint = false;
    YAT_POINT testPoint = nextXY; // search X and Y in grid map

    switch (alongSide)
    {
        case 0:
            for(int i = nextXY.y; i >= 0; i--)
            {
                testPoint.y = i;
                if(plannerMapMatrix(testPoint.x,testPoint.y) == ID_REMAIN)
                {
                    getNextPoint = true;
                    break;
                }
            }
            break;
        case 1:
            for(int i = nextXY.x; i < gridParametre.rangeX; i++)
            {
                testPoint.x = i;
                if(plannerMapMatrix(testPoint.x,testPoint.y) == ID_REMAIN)
                {
                    getNextPoint = true;
                    break;
                }
            }
            break;
        case 2:
            for(int i = nextXY.y; i < gridParametre.rangeY; i++)
            {
                testPoint.y = i;
                if(plannerMapMatrix(testPoint.x,testPoint.y) == ID_REMAIN)
                {
                    getNextPoint = true;
                    break;
                }
            }
            break;
        case 3:
            for(int i = nextXY.x; i >= 0; i--)
            {
                testPoint.x = i;
                if(plannerMapMatrix(testPoint.x,testPoint.y) == ID_REMAIN)
                {
                    getNextPoint = true;
                    break;
                }
            }
            break;
        default:
            break;
    }

    nextXY = testPoint; // if did not find the next point, return the edge point in the grid map

    return getNextPoint;
}

bool motionDetect()
{
    static int motionType = 0;
    static int motionStep = 0;
    static int motionSide = 0;

    static poseType lastPoint = robotPose;
    static poseType nextPoint = robotPose;

    gridMapType gridParametre = getGridParametre(gridMap);

    static YAT_POINT lastXY = poseToRaster(lastPoint,gridParametre);
    static YAT_POINT nextXY = poseToRaster(nextPoint,gridParametre);

    YAT_POINTF commandVelocity;

    Map plannerMap = getPlannerMap();

    // ROS_INFO(" motion Step is %d. ",motionStep);

    if( motionStep == 0 ) 
    {
        motionSide = 0;

        double targetAngle = getTargetAngle( motionType, motionSide );
        int alongSide = getAlongSide( targetAngle );
        // double targetAngle = constrainAngle(getAlongSide(robotPose.theta) * PI/2 - PI/2);

        if(motionType == 0)
        {
            lastXY = poseToRaster(startPoint,gridParametre);
        }
        else
        {
            lastXY = nextXY;
        }
        lastPoint = rasterToPose(lastXY,gridParametre);

        nextXY = poseToRaster(startPoint,gridParametre);
        if(getRowPoint(nextXY,alongSide,plannerMap))
        {
            nextPoint = rasterToPose(nextXY,gridParametre);
            motionStep = 2;
        }
        else
        {
            motionStep = 4;
        }

    }

    if( motionStep == 1 )
    {
        if(straightMotionAtoB(nextPoint,lastPoint,commandVelocity))
        {
            motionStep = 4; 
        }
    }

    if( motionStep == 2 )
    {
        if(planMotionAtoB(nextPoint,lastPoint,commandVelocity))
        {
            motionStep = 4;
        }
    }

    if( motionStep == 3 )
    {
        if(planMotionAtoB(nextPoint,lastPoint,commandVelocity))
        {
            motionType ++;
            motionStep = 0;
        }
    }

    if( motionStep == 4 )
    {
        Eigen::MatrixXi plannerMapMatrix = getGridMapMatrix(plannerMap);

        // get current target angle and move distance
        motionSide = (motionSide + 1) % 4;
        lastXY = nextXY;
        double targetAngle = getTargetAngle(motionType, motionSide);

        YAT_POINT robotXY = poseToRaster(robotPose,gridParametre);
        YAT_POINT nearstXY = findNearestPoint(robotPose,plannerMapMatrix);

        int alongSide = getAlongSide( targetAngle );
        if(alongSide % 2 == 1) // next motion along X, last motion along Y
        {
            lastXY.y = nearstXY.y;
        }
        else if(alongSide % 2 == 0) // next motion along Y, last motion along X
        {
            lastXY.x = nearstXY.x;
        }

        YAT_POINT targetXY = lastXY;
        bool getNextPoint = false;
        if( motionSide % 2 == 0 )
        {
            getNextPoint = getRowPoint(targetXY,alongSide,plannerMap);
        }
        else
        {
            YAT_POINT midXY = lastXY;
            double nextAngle = getTargetAngle(motionType, (motionSide + 1) % 4);
            switch (alongSide)
            {
                case 0:
                    midXY.y = lastXY.y - 2;
                    break;
                case 1:
                    midXY.x = lastXY.x + 2;
                    break;
                case 2:
                    midXY.y = lastXY.y + 2;
                    break;
                case 3:
                    midXY.x = lastXY.x - 2;
                    break;
                default:
                    break;
            }
            int nextAlong = getAlongSide( nextAngle );
            getNextPoint = getLinePoint(midXY,nextAlong,plannerMap);
            targetXY = midXY;
        }

        poseType targetPose = rasterToPose(targetXY,gridParametre);
        double moveDistance = deltaDistance(targetPose,nextPoint);

        spdlog::get("robot_state")->info("along side = {}, target Angle = {}, move Distance = {}.", alongSide, targetAngle, moveDistance);
        spdlog::get("robot_state")->info("next Point: x = {}, y = {}.", targetPose.x, targetPose.y);

        YAT_POINT remainEdgePoint;

        if( findRemainEdgePoint(motionType,remainEdgePoint,plannerMap) )
        {
            if( getNextPoint )
            {
                nextXY = targetXY;
                spdlog::get("robot_state")->info("go to adjacent point.");
                motionStep = 1;  // go to next point
            }
            else
            {
                nextXY = remainEdgePoint;
                lastXY = robotXY;
                spdlog::get("robot_state")->info("go to remain point.");
                motionSide = 3;  // next motionSide is 0 (means start from scratch zigzag)
                motionStep = 2;  // go to remain point
            }
        }
        else
        {
            // nextXY = poseToRaster(startPoint,gridParametre);
            // lastXY = robotXY;
            ROS_INFO("Finish the NO.%d motion type.",motionType);
            spdlog::get("robot_state")->info("go to next motion type.");
            // motionStep = 3;  // go home point

            motionType ++;
            motionStep = 0;
        }

        lastPoint = rasterToPose(lastXY,gridParametre);
        nextPoint = rasterToPose(nextXY,gridParametre);

        spdlog::get("robot_state")->info("get the next Point: x={}, y={}, theta={}.",nextPoint.x, nextPoint.y,nextPoint.theta);
    }

    /************************************************/
    msg_vel.linear.x = commandVelocity.x;     // velocity control
    msg_vel.angular.z = commandVelocity.y;    // angle control

    vel_pub.publish(msg_vel);

    nextPoint.theta = atan2(nextPoint.y - lastPoint.y, nextPoint.x - lastPoint.x);

    spdlog::get("robot_state")->info("robot control: type={}, commmand velocity: v={}, w={}.", motionType, commandVelocity.x, commandVelocity.y);
    spdlog::get("robot_state")->info("target point: x={}, y={}, theta={}.",nextPoint.x, nextPoint.y, nextPoint.theta);
    spdlog::get("robot_state")->info("last point: x={}, y={},motionStep={}.\n",lastPoint.x, lastPoint.y,motionStep);

    return (motionType >= 4)?true:false;
}

void missionStop()
{
    msg_vel.linear.x = 0;     // velocity control
    msg_vel.angular.z = 0;    // angle control

    vel_pub.publish(msg_vel);
}

void ekfPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg_ekf)
{
    updatePoseTime = ros::Time::now();

    // in the NEU navigation frame
    robotPose.x = msg_ekf->pose.position.x;
    robotPose.y = msg_ekf->pose.position.y;
    robotPose.theta = msg_ekf->pose.orientation.z;

    int rtkFlag = (int)msg_ekf->pose.orientation.w;

    spdlog::get("robot_state")->info("the robot data is: x={}, y={}, theta={}, rtkFlag = {}.",robotPose.x, robotPose.y,robotPose.theta,rtkFlag);

    updateRobotPose = (rtkFlag == 1) ? true:false;
}

void gridMapCallback(nav_msgs::OccupancyGrid msg_map)
{
    updateMapTime = ros::Time::now();

    gridMap = msg_map;
    
    updateMapFlag = true;
    // ROS_INFO("get the grid map");
    spdlog::get("robot_state")->info("update the grid map from topic named grid_map.");
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
    ros::init(argc, argv, "motion");
    ROS_INFO_STREAM("motion program is starting...");

    ros::NodeHandle nh;

    auto my_logger = spdlog::rotating_logger_mt("robot_state", file_log, 1048576*20, 5);
    my_logger->info("----------------------- The Vision Motion is starting -----------------------");

    // publish message
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
    target_pub = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);

    // subscrible message
    // ros::Subscriber stop_sub = nh.subscribe("mission_finish", 1, missionFinishCallback);
    ros::Subscriber pose_sub = nh.subscribe("ekf_pose", 1, ekfPoseCallback);
    ros::Subscriber map_sub = nh.subscribe("grid_map", 1, gridMapCallback);
    ros::Subscriber record_pub = nh.subscribe("/mode", 1, joyModeCallback);

    // initial flag
    bool initialMotion = false;
    bool finishTask = false;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        float dt = ros::Time::now().toSec() - updateMapTime.toSec();

        if( initialMotion )
        {
            // Eigen::MatrixXi pathWayMatrix = updatePathWay( pathWay );
            bool mapLost = (ros::Time::now().toSec() - updateMapTime.toSec() > 1.0)?true:false;
            bool poseLost = (ros::Time::now().toSec() - updatePoseTime.toSec() > 0.5)?true:false;

            if( mapLost || poseLost )//|| (!getBumpTopic)
            {
                missionStop();
                if(mapLost || poseLost) ROS_INFO(" %f seconds ago and up to now, has not received the map data. ",dt);
                spdlog::get("robot_state")->info("mapLost={}, poseLost={}, getBumpTopic = {},stop detecting.", mapLost, poseLost,getBumpTopic);
            }
            else
            {
                ros::Time startTime = ros::Time::now();

                double deltaTimeA = ros::Time::now().toSec() - startTime.toSec();

                if(!finishTask)
                {
                    if(motionDetect())
                    {
                        finishTask = true;
                    }
                }

                double deltaTimeB = ros::Time::now().toSec() - startTime.toSec() - deltaTimeA;
                spdlog::get("robot_state")->info("deltaTimeA = {}, deltaTimeB = {}.", deltaTimeA, deltaTimeB);
            }
        }
        else
        {
            if( updateMapFlag && updateRobotPose )
            {
                if(findOriginPoint())
                {
                    initialMotion = true;
                }
                else
                {
                    ROS_INFO("did not find the origin point.");
                }

                // initialMotion = true;
            }
            spdlog::get("robot_state")->info("updateMapFlag = {}, updateRobotPose = {}.", updateMapFlag, updateRobotPose);
        }

        updateMapFlag = false;
        updateRobotPose = false;

        ros::spinOnce();
        loop_rate.sleep();
        // ros::Duration(0.1).sleep();
    }

    fout_point.close();
    return 0;
}
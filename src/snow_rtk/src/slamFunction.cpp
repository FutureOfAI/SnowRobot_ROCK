#include <iostream>
#include <string>
#include <list>
#include <vector>
#include <map>
#include <stack>

#include "core.h"

// #include <opencv2/imgproc/imgproc.hpp> 
// #include <opencv2/highgui/highgui.hpp>

using namespace std;

enum MAP_ID
{
    ID_CUT = 0, ID_PASS = 10, ID_REMAIN = 40, ID_OUTER = 80 , ID_OBSTACLE = 90, ID_EDGE = 100, 
};

typedef struct gridMapParametreType
{
    int scale;

    double offsetX;
	double offsetY;

    int rangeX;
    int rangeY;
}gridMapType;

typedef struct robotPointPoseType
{
	double  x; // coordinate in X axis
	double  y; // coordinate in Y axis
	double  theta; // yaw of robot
}poseType;

YAT_POINT worldToRaster(YAT_POINTF mapPoint,gridMapType parametre)
{
    YAT_POINT gridPoint;
    gridPoint.x = (int)((mapPoint.x - parametre.offsetX) * parametre.scale + 0.5);
    gridPoint.y = (int)((mapPoint.y - parametre.offsetY) * parametre.scale + 0.5); 
    return gridPoint;
}

YAT_POINTF rasterToWorld(YAT_POINT gridPoint,gridMapType parametre)
{
    YAT_POINTF mapPoint;
    mapPoint.x = ((double)gridPoint.x )/parametre.scale + parametre.offsetX;
    mapPoint.y = ((double)gridPoint.y )/parametre.scale + parametre.offsetY;
    return mapPoint;
}

int ceilValue(double value)
{
    return ( (int) value + 1);
}

double minValue(vector<double> array,int num)
{
    double minData = array[0];
    int minNum = 0; 
    for(int i = 0; i < num;i++) 
    {
        if (array[i] < minData)
        {
            minData = array[i];
            minNum = i; 
        }
    }

    return minData;
}

YAT_POINTF minValue(vector<YAT_POINTF> array,int num)
{
    vector<double> arrayX(num);
    vector<double> arrayY(num);

    YAT_POINTF minData;

    for(int i = 0; i < num;i++) //sizeof(array)
    {
        arrayX[i] = array[i].x;
        arrayY[i] = array[i].y;
    }

    minData.x = minValue(arrayX,num);
    minData.y = minValue(arrayY,num);

    return minData;
}

double maxValue(vector<double> array,int num)
{
    double maxData = array[0];
    int maxNum = 0; 
    for(int i = 0; i < num;i++) //sizeof(array)
    {
        if (array[i] > maxData)
        {
            maxData = array[i];
            maxNum = i; 
        }
    }

    return maxData;
}

double degreeToRad(double angle)
{
    return angle * PI / 180;
}

double radToDegree(double angle)
{
    return angle * 180 / PI;
}

double deltaDistance(YAT_POINTF pointA,YAT_POINTF pointB)
{
    double delta_X = pointA.x - pointB.x;
    double delta_Y = pointA.y - pointB.y;

    return sqrt(delta_X*delta_X + delta_Y*delta_Y);
}

double deltaDistance(poseType pointA, poseType pointB)
{
    double delta_X = pointA.x - pointB.x;
    double delta_Y = pointA.y - pointB.y;

    return sqrt(delta_X*delta_X + delta_Y*delta_Y);
}

double distanceToAB(poseType pointA, poseType pointB,poseType currentPoint)
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

int maxValue(vector<int> array,int num)
{
    int maxData = array[0];
    int maxNum = 0; 
    for(int i = 0; i < num;i++) //sizeof(array)
    {
        if (array[i] > maxData)
        {
            maxData = array[i];
            maxNum = i; 
        }
    }

    return maxData;
}

YAT_POINTF maxValue(vector<YAT_POINTF> array,int num)
{
    vector<double> arrayX(num);
    vector<double> arrayY(num);

    YAT_POINTF maxData;

    for(int i = 0; i < num;i++) //sizeof(array)
    {
        arrayX[i] = array[i].x;
        arrayY[i] = array[i].y;
    }

    maxData.x = maxValue(arrayX,num);
    maxData.y = maxValue(arrayY,num);

    return maxData;
}


YAT_POINT maxValue(vector<YAT_POINT> array,int num)
{
    vector<int> arrayX(num);
    vector<int> arrayY(num);

    YAT_POINT maxData;

    for(int i = 0; i < num;i++) //sizeof(array)
    {
        arrayX[i] = array[i].x;
        arrayY[i] = array[i].y;
    }

    maxData.x = maxValue(arrayX,num);
    maxData.y = maxValue(arrayY,num);

    return maxData;
}


int minValue(vector<int> array,int num)
{
    int minData = array[0];
    int minNum = 0; 
    for(int i = 0; i < num;i++) 
    {
        if (array[i] < minData)
        {
            minData = array[i];
            minNum = i; 
        }
    }

    return minData;
}

YAT_POINT minValue(vector<YAT_POINT> array,int num)
{
    vector<int> arrayX(num);
    vector<int> arrayY(num);

    YAT_POINT minData;

    for(int i = 0; i < num;i++) //sizeof(array)
    {
        arrayX[i] = array[i].x;
        arrayY[i] = array[i].y;
    }

    minData.x = minValue(arrayX,num);
    minData.y = minValue(arrayY,num);

    return minData;
}

YAT_POINTF meanValue(vector<YAT_POINTF> array,int num)
{
    YAT_POINTF meanData;
    meanData.x = 0;
    meanData.y = 0;
    for(int i = 0; i < num;i++)
    {
        meanData.x += (array[i].x)/num;
        meanData.y += (array[i].y)/num;
    }

    return meanData;
}

int floorValue(double value)
{
    return (int) value ;
}

vector<int> minRank(vector<int> linePath,int num)
{
    vector<int> rankData = linePath;
    int a;
    for(int i = 0; i < num; i++ )
    {
        for(int j = i+1; j < num; j++)
        {
            if(rankData[j] < rankData[i])
            {
                a = rankData[i];
                rankData[i] = rankData[j];
                rankData[j] = a;
            }
        }
    }

    return rankData;
}

vector<int> maxRank(vector<int> linePath,int num)
{
    vector<int> rankData = linePath;
    int a;
    for(int i = 0; i < num; i++ )
    {
        for(int j = i+1; j < num; j++)
        {
            if(rankData[j] > rankData[i])
            {
                a = rankData[i];
                rankData[i] = rankData[j];
                rankData[j] = a;
            }
        }
    }

    return rankData;
}

vector<YAT_POINTF> densePosition(YAT_POINTF pointA,YAT_POINTF pointB,int num)
{
    vector<YAT_POINTF> densePoint(num);
    double t;
    for(int i =0 ; i < num; i++)
    {
        t = ((double) i)/num;
        densePoint[i].x = (1-t)*pointA.x +t*pointB.x;
        densePoint[i].y = (1-t)*pointA.y +t*pointB.y;
    }

    return densePoint;
}

/************************************************************************************/


YAT_POINT poseToRaster(poseType poseData, gridMapType parametre)
{
    YAT_POINTF robotPoint;
    robotPoint.x = poseData.x;
    robotPoint.y = poseData.y;

    return worldToRaster(robotPoint,parametre);
}

poseType rasterToPose(YAT_POINT gridPoint, gridMapType parametre)
{
    YAT_POINTF robotPoint = rasterToWorld(gridPoint, parametre);
    poseType mapPoint;
    mapPoint.x = robotPoint.x;
    mapPoint.y = robotPoint.y;

    return mapPoint;
}

bool detectPointXY(gridMapType parametre,YAT_POINT detectePoint)
{   
    if( detectePoint.x < 0 )
    {
        return false;
    }

    if( detectePoint.x > parametre.rangeX - 1 )
    {
        return false;
    }

    if( detectePoint.y < 0 )
    {
        return false;
    }

    if( detectePoint.y > parametre.rangeY - 1 )
    {
        return false;
    }

    return true;
}

YAT_POINT constrainGridXY(gridMapType parametre,int x, int y)
{
    YAT_POINT detectePoint;

    detectePoint.x = x;
    detectePoint.y = y;
    
    if( detectePoint.x < 0 )
    {
        detectePoint.x = 0;
    }

    if( detectePoint.x > parametre.rangeX - 1 )
    {
        detectePoint.x = parametre.rangeX - 1;
    }

    if( detectePoint.y < 0 )
    {
        detectePoint.y = 0;
    }

    if( detectePoint.y > parametre.rangeY - 1 )
    {
        detectePoint.y = parametre.rangeY - 1;
    }

    return detectePoint;
}

YAT_POINT constrainGridXY(gridMapType parametre,YAT_POINT detectePoint)
{
    return constrainGridXY(parametre,detectePoint.x, detectePoint.y) ;
}

void initializeGridPoint(Eigen::MatrixXd &mapPoint, gridMapType parametre)
{
    for(int i = 0; i < parametre.rangeX ; i++)
    {
        for(int j = 0; j < parametre.rangeY ; j++)
        {
            mapPoint(i,j) = -1; 
        }
    }
}

void initializeGridPoint(Eigen::MatrixXi &mapPoint, gridMapType parametre)
{
    for(int i = 0; i < parametre.rangeX ; i++)
    {
        for(int j = 0; j < parametre.rangeY ; j++)
        {
            mapPoint(i,j) = 0; 
        }
    }
}


std::vector<int> matrixToArray(Eigen::MatrixXi matrixPoint, gridMapType parametre)
{
    std::vector<int> arrayPoint(parametre.rangeX * parametre.rangeY);
    for(int i = 0; i < parametre.rangeX ; i++)
    {
        for(int j = 0; j < parametre.rangeY ; j++)
        {
            arrayPoint[ i * parametre.rangeY + j] = matrixPoint(i,j);
        }
    }
    return arrayPoint;
}

Eigen::MatrixXi arrayToMatrix(std::vector<int> arrayPoint, gridMapType parametre)
{
    Eigen::MatrixXi matrixPoint(parametre.rangeX, parametre.rangeY);
    for(int i = 0; i < parametre.rangeX ; i++)
    {
        for(int j = 0; j < parametre.rangeY ; j++)
        {
            matrixPoint(i,j) = arrayPoint[ i * parametre.rangeY + j];
        }
    }
    
    return matrixPoint;
}

// function means: detectedPoint is in or not in polygon, detectedZone is point of the polygon 
bool inPolygonZone(YAT_POINTF  detectedPoint, vector<YAT_POINTF>  detectedZone)
{
    Eigen::Vector2d targetVector(0,0);
    Eigen::Vector2d pathVector(0,0);
    Eigen::Vector2d normalVector(0,0);

    bool intZoneFlag = true;

    for(int i = 0; i < 4; i++)
    {
        int startNum = i;
        int endNum = (i + 1) % 4;

        targetVector(0) = detectedZone[ startNum ].x - detectedPoint.x;
        targetVector(1) = detectedZone[ startNum ].y - detectedPoint.y;

        pathVector(0) = detectedZone[ endNum ].x - detectedZone[ startNum ].x;
        pathVector(1) = detectedZone[ endNum ].y - detectedZone[ startNum ].y;

        normalVector(0) = pathVector(1);
        normalVector(1) = - pathVector(0);

        double judgeParametre = normalVector(0) * targetVector(0) + normalVector(1) * targetVector(1);

        if( judgeParametre <= 0)
        {
            intZoneFlag = false;
        }
    }

    return intZoneFlag;
}

bool inPolygonCeil(YAT_POINTF  detectedPoint, vector<YAT_POINTF>  detectedZone, int ceilDepart, int num)
{
    int lineNum = num / ceilDepart;
    int rowNum = num % ceilDepart;

    // vector of ceil polygon
    vector<YAT_POINTF> polygonVector(4);
    for(int i = 0; i < 4; i++)
    {
        int startNum = i;
        int endNum = (i + 1) % 4;

        polygonVector[i].x = ( detectedZone[ endNum ].x - detectedZone[ startNum ].x ) / ceilDepart;
        polygonVector[i].y = ( detectedZone[ endNum ].y - detectedZone[ startNum ].y ) / ceilDepart;
    }

    vector<YAT_POINTF>  detectedCeil(4);
    detectedCeil[0].x = detectedZone[0].x + lineNum * polygonVector[0].x - rowNum * polygonVector[3].x;
    detectedCeil[0].y = detectedZone[0].y + lineNum * polygonVector[0].y - rowNum * polygonVector[3].y;
    for(int j = 0; j < 3; j++)
    {
        detectedCeil[j+1].x = detectedCeil[j].x + polygonVector[j].x;
        detectedCeil[j+1].y = detectedCeil[j].y + polygonVector[j].y;
    }

    return inPolygonZone(detectedPoint, detectedCeil);
}

/*****************************************************/

double controlPID(double error, double integral_error, double last_error, double Kp, double Ki, double Kd)
{
       double controller;
       controller = Kp * error + Ki * integral_error + Kd * (error - last_error);
       return controller;
}

int sign(double data)
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

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
    return q;
}


void dilateMap(Map * map, double dilatewidth)
{
	bool *mask = (bool*)malloc(sizeof(bool)*(map->height * map->width));
	memset(mask, false, map->height * map->width);

	int dilate = dilatewidth * map->scale * 2 + 1;

	if (dilate < 1)
	{
		dilate = 1;
	}

	for (int i = 0; i<map->height; i++)
	{
		for (int j = 0; j<map->width; j++)
		{
			bool flag = false;
			for (int k = -dilate / 2; k <= dilate / 2; k++)
			{
				if ((i + k) < 0 || (i + k) >= map->height)
					continue;
				for (int l = -dilate / 2; l <= dilate / 2; l++)
				{
					if ((j + l) < 0 || (j + l) >= map->width)
						continue;
					if (*(map->data + (i + k) * map->width + j + l) == 100)
						mask[i * map->width + j] = true;
				}
			}
		}
	}

	for (int i = 0; i < map->height; i++)
	{
		for (int j = 0; j < map->width; j++)
		{
			if (mask[i * map->width + j] == true )//&& *(map->data + i * map->width + j) == 0)
				*(map->data + i * map->width + j) = 100;
				//*(map->data + i * map->width + j) = 100;	//warning range
		}
	}

	free(mask);
}

//  Connected Component Analysis/Labeling By Seed-Filling Algorithm 
//  Author:  www.icvpr.com  
//  Blog  :  http://blog.csdn.net/icvpr 
// void icvprCcaBySeedFill(const cv::Mat& _binImg, cv::Mat& _lableImg)
// {
// 	// connected component analysis (4-component)
// 	// use seed filling algorithm
// 	// 1. begin with a foreground pixel and push its foreground neighbors into a stack;
// 	// 2. pop the top pixel on the stack and label it with the same label until the stack is empty
// 	// 
// 	// foreground pixel: _binImg(x,y) = 1
// 	// background pixel: _binImg(x,y) = 0
 
// 	if (_binImg.empty() ||
// 		_binImg.type() != CV_8UC1)
// 	{
// 		return ;
// 	}
 
// 	_lableImg.release();
// 	_binImg.convertTo(_lableImg, CV_32SC1);
 
// 	int label = 1 ;  // start by 2
 
// 	int rows = _binImg.rows - 1 ;
// 	int cols = _binImg.cols - 1 ;
// 	for (int i = 1; i < rows-1; i++)
// 	{
// 		int* data= _lableImg.ptr<int>(i) ;
// 		for (int j = 1; j < cols-1; j++)
// 		{
// 			if (data[j] == 1)
// 			{
// 				std::stack<std::pair<int,int>> neighborPixels ;   
// 				neighborPixels.push(std::pair<int,int>(i,j)) ;     // pixel position: <i,j>
// 				++label ;  // begin with a new label
// 				while (!neighborPixels.empty())
// 				{
// 					// get the top pixel on the stack and label it with the same label
// 					std::pair<int,int> curPixel = neighborPixels.top() ;
// 					int curX = curPixel.first ;
// 					int curY = curPixel.second ;
// 					_lableImg.at<int>(curX, curY) = label ;
 
// 					// pop the top pixel
// 					neighborPixels.pop() ;
 
// 					// push the 4-neighbors (foreground pixels)
// 					if (_lableImg.at<int>(curX, curY-1) == 1)
// 					{// left pixel
// 						neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;
// 					}
// 					if (_lableImg.at<int>(curX, curY+1) == 1)
// 					{// right pixel
// 						neighborPixels.push(std::pair<int,int>(curX, curY+1)) ;
// 					}
// 					if (_lableImg.at<int>(curX-1, curY) == 1)
// 					{// up pixel
// 						neighborPixels.push(std::pair<int,int>(curX-1, curY)) ;
// 					}
// 					if (_lableImg.at<int>(curX+1, curY) == 1)
// 					{// down pixel
// 						neighborPixels.push(std::pair<int,int>(curX+1, curY)) ;
// 					}
// 				}		
// 			}
// 		}
// 	}
// }

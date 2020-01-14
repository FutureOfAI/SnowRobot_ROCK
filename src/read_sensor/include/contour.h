#ifndef _COUNTOUR_H
#define _COUNTOUR_H
#include "core.h"

/************************************************************************/
/* 函数名：CreateROIImage
说明：  重建图像感兴趣区域
输入：  polygon - 多边形顶点
nPolygonNUM—多边形顶点个数
nWidth—图像宽度
nHeight—图像高度
输出：  无
返回：  创建的图像感兴趣区域
/************************************************************************/
Map createGridMap(const char *pointfilename, POLYGON &smoothpolygon);

/************************************************************************/
/* 函数名：ScanFill
说明：多边形填充的主体程序
输入：plolygonNum—多边形顶点个数
polygon—指向多边形顶点的指针数组
输出：pImage—感兴趣区域图像数据*/
/************************************************************************/
void scanFill(YAT_POINT* polygon, int polygonNum, unsigned char *pImage, int nWidth, int nHeight);

/************************************************************************/
/* 函数名：BuildEdgeList
说明：  创建边表的主体函数
输入：  cnt—多边形顶点个数
pts—多边形顶点坐标
输出：  edges[]—指向活性边结点的指针数组
返回：  无*/
/************************************************************************/
void buildEdgeList(int cnt, YAT_POINT* pts, YAT_EDGE* edges[], int yMin);

/************************************************************************/
/*函数名： BuildActiveList
说明：建立活性边表的主体函数，建立第scan条扫描线的活性边表
输入：scan—扫描线行数
edges—活性边列表
输出：active—活性边表
返回：无*/
/************************************************************************/
void buildActiveList(int scan, YAT_EDGE** active, YAT_EDGE* edges[]);

/************************************************************************/
/*函数名：FillScan
说明：  填充一对交点的主体函数，填充扫描线上，且在下一结点到再下一结点之间的点
输入：  scan—扫描线行数
active—活动边表
输出：  pImage—感兴趣区域图像
返回：  无*/
/************************************************************************/
void fillScan(int scan, YAT_EDGE* active, unsigned char* pImage, int nWidth);

/************************************************************************/
/* 函数名： UpdateActiveList
说明：   删除扫描线scan完成交点计算的活性边，同时更新交点x域
输入：   scan—扫描线行数
输出：   active—活动边表
返回：	无*/
/************************************************************************/
void updateActiveList(int scan, YAT_EDGE **active);

/************************************************************************/
/*函数名： ResortActiveList
说明：   对活性边表节点重新排序的主体函数，活性边表active中的节点按照x值从小到大重新排序
输入：
输出：   active—活性边表
返回;	   无*/
/************************************************************************/
void resortActiveList(YAT_EDGE **active);

/************************************************************************/
/* 函数名：PushEdgeList
说明：  把新放入边列表中
输入：  Edge—新边
输出：  List—边列表
返回：  无*/
/************************************************************************/
void pushEdgeList(YAT_EDGE **List, YAT_EDGE* Edge);

void getPolygon(const char *filename, POLYGON *polygon);

void normalizePolygon(POLYGON *polygong, NORMEDPOLYGON *normpolygon, Map map);

//get distance*distance between two points
float getdist2(YAT_POINTF point1, YAT_POINTF point2);

Map createMap(int nWidth, int nHeight, float scale);

void releaseMap(Map map);

void addOnePoint2Polygon(POLYGON *polygon, YAT_POINTF point);

/************************************************************************/
/*函数名： dilateMap
说明：使地图边缘膨胀
输入：map：原始gridmap
dilatewidht：要膨胀的宽度
输出：map：膨胀后的gridmap
返回:无*/
/************************************************************************/
void dilateMap(Map *map, float dilatewidht);

/************************************************************************/
/*函数名： setCome
说明：重置单元格成本值
输入：map：原始gridmap
x,y:重置单元格的坐标
value:重置值
返回:无*/
/************************************************************************/
void setCome(Map *map, float x, float y);

/************************************************************************/
/*函数名： saveMap
说明：保存map
输入：map：gridmap
filename：map文件名
返回:无*/
/************************************************************************/
bool saveMap(Map *map, const char *filename);

/************************************************************************/
/*函数名： loadMap
说明：读取map
输入：filename：map文件名
返回:Map  需要自己释放*/
/************************************************************************/
Map loadMap(const char *filename);

/************************************************************************/
/*函数名： isInMap
说明：判断point位置是否在所画地图内
输入：map：地图
输入：point：位置，YAT_POINTF类型
返回：是否在地图内，bool类型*/
/************************************************************************/
bool isInMap(Map *map, YAT_POINTF point);

/************************************************************************/
/*函数名： isPointInMap
说明：判断point位置是否在所画地图内
输入：map：地图
输入：point：位置，YAT_POINT类型
返回：是否在地图内，bool类型*/
/************************************************************************/
bool isPointInMap(Map *map, YAT_POINT point);

/************************************************************************/
/*函数名： setUsed
说明：设置ｐｏｉｎｔ位置已经在ｐａｔｈ路径上
输入：map：地图
输入：point：位置，YAT_POINT类型
返回：无/
/************************************************************************/
void setUsed(Map *map, YAT_POINT point);


/************************************************************************/
/*函数名： isOutOfRange
说明：判断point位置是否在栅格图上
输入：map：地图
输入：point：位置，YAT_POINT类型
返回：是否在图上，bool类型*/
/************************************************************************/
bool isOutOfRange(Map *map, YAT_POINT point);

/************************************************************************/
/*函数名： isWarnedRange
说明：判断point位置是否在预警范围
输入：map：地图
输入：point：位置，YAT_POINTF类型
返回：是否在地图预警范围内，bool类型*/
/************************************************************************/
bool isWarnedRange(Map *map, YAT_POINTF point);

/************************************************************************/
/*函数名： getEdgePoint
说明：输出离 point位置最近的边的端点
输入：map：地图
输入：point：位置，YAT_POINTF类型
返回：边界向量的角度*/
/************************************************************************/
float getWallHeading(Map *map, YAT_POINTF point);

/************************************************************************/
/*函数名： line_heading
说明：输出计算两点所连成直线与x正半轴的夹角
输入：start:起点，YAT_POINTF类型
输入：end:终点，YAT_POINTF类型
返回：传入两点连成直线与x正半轴的夹角*/
/************************************************************************/
float line_heading(YAT_POINTF start, YAT_POINTF end);

/************************************************************************/
/*函数名： smoth_map
说明：平滑map
输入：map：原始gridmap
width：宽度
输出：map：膨胀后的gridmap
/************************************************************************/
//void smoth_map(Map *map, int width);

YAT_POINT world_to_map(Map *map, YAT_POINTF pointworld);

YAT_POINTF map_to_world(Map *map, YAT_POINT pointmap);

#endif



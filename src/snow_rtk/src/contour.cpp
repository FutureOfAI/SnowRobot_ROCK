#include "contour.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>

#include "opencv2/opencv.hpp"
using namespace cv;

int isdigitstr(char *str)
{
	return (strspn(str, "0123456789.-") == strlen(str));
}

#define LINE 200  //每行读取的字符个数
#define MINDIST 0.01 //路径点最小距离的平方,即最小距离为1分米
void getPolygon(const char *filename, POLYGON *ppolygon)
{
	ppolygon->num = 0;

	FILE *fp;
        if ((fp = fopen(filename, "r")) == NULL)
	{
		printf("open file error!!\n");
		return;
	}

	int pointnum = 0;
	char buf[LINE];
	float cordinat[5];
	while (fgets(buf, LINE, fp))
	{
		char *retptr, *temp = buf;
		//printf("substr%s\n", buf);
		int digitnum = 0;
		while ((retptr = strtok(temp, " ")) != NULL && digitnum < 4)
		// while ((retptr = strtok(temp, "\t")) != NULL && digitnum < 4)
		{
			if (!isdigitstr(retptr))
				break;
			float val = atof(retptr);

			cordinat[digitnum] = val;
			temp = NULL;
			digitnum++;
		}

		if (digitnum == 4)
		{
			YAT_POINTF point;
			point.x = cordinat[0];
			point.y = cordinat[1];

			if (pointnum > 1)
			{
				YAT_POINTF lastpoint = ppolygon->points[ppolygon->num - 1];
				if (getdist2(lastpoint, point) < MINDIST)
					continue;
			}

			addOnePoint2Polygon(ppolygon, point);
			pointnum++;
		}
	}
}


void minMaxPoint(POLYGON &polygon, float &xmaxp, float &xminp, float &ymaxp, float &yminp)
{
	float xmax = -100000;
	float xmin = 100000;
	float ymax = -100000;
	float ymin = 100000;
	for (int i = 0; i < polygon.num; i++)
	{
		if (polygon.points[i].x > xmax)
		{
			xmax = polygon.points[i].x;
		}

		if (polygon.points[i].x < xmin)
		{
			xmin = polygon.points[i].x;
		}

		if (polygon.points[i].y > ymax)
		{
			ymax = polygon.points[i].y;
		}

		if (polygon.points[i].y < ymin)
		{
			ymin = polygon.points[i].y;
		}

		xmaxp = xmax;
		xminp = xmin;
		ymaxp = ymax;
		yminp = ymin;
	}
}

void normalizePolygon(POLYGON *ppolygon, NORMEDPOLYGON *pnormpolygon, Map map)
{
	int num = ppolygon->num;
    float offsetx = map.xoffset;
    float offsety = map.yoffset;
	// printf("OFFSET: x:%f, y:%f",offsetx, offsety);

	float scale = map.scale;

	for (int i = 0; i < num; i++)
	{
		YAT_POINT temp;
		temp.x = (ppolygon->points[i].x + offsetx) * scale;
		temp.y = (ppolygon->points[i].y + offsety) * scale;
		pnormpolygon->points[i]= temp;
	}
	pnormpolygon->num = num;
}

void normalizePolygon2(std::vector<Point> contour, POLYGON *polygon, Map map)
{
	int num = 0;
    float offsetx = map.xoffset;
    float offsety = map.yoffset;
	// printf("OFFSET: x:%f, y:%f",offsetx, offsety);

	YAT_POINTF lastpoint;

	float scale = map.scale;

	for (int i = 0; i < contour.size(); i++)
	{
		YAT_POINTF temp;
		temp.x = contour[i].x / scale - offsetx ;
		temp.y = contour[i].y /scale - offsety;
		
		if(abs(temp.x - lastpoint.x) > 0.05 || abs(temp.y - lastpoint.y) > 0.05)
		{
			polygon->points[num] = temp;
			num++;
		}

		
	}
	polygon->num = num + 1;
}

//get distance*distance between two points
float getdist2(YAT_POINTF point1, YAT_POINTF point2)
{
	float dist = 0;
	dist = (point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y);
	return dist;
}

Map createMap(int nWidth, int nHeight, float scale)
{
	Map map;
	map.width = nWidth;
	map.height = nHeight;
	map.scale = scale;
	map.data = (unsigned char *)malloc(nWidth * nHeight * sizeof(unsigned char));
	return map;
}

void releaseMap(Map map)
{
	free(map.data);
	map.data = NULL;
}

void addOnePoint2Polygon(POLYGON *ppolygon, YAT_POINTF point)
{
	if (ppolygon->num >= MAXPOINTNUM - 1)
	{
		printf("polygon point is full, can not add point to point");
		return;
	}
	ppolygon->points[ppolygon->num] = point;
	ppolygon->num++;
}

void dilateMap(Map * map, float dilatewidth)
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
					if (*(map->data + (i + k) * map->width + j + l) == 255)
						mask[i * map->width + j] = true;
				}
			}
		}
	}

	for (int i = 0; i < map->height; i++)
	{
		for (int j = 0; j < map->width; j++)
		{
			if (mask[i * map->width + j] == true && *(map->data + i * map->width + j) == 0)
				*(map->data + i * map->width + j) = 100;
				//*(map->data + i * map->width + j) = 100;	//warning range
		}
	}

	//原点　１米内设为缓冲区
	YAT_POINTF pointf{0, 0}; 
	dilate = 0.4 * map->scale * 2 + 1; 
	int x0 = (map->xoffset + pointf.x) * map->scale;
	int y0 = (map->yoffset + pointf.y) * map->scale;
	for (int k = -dilate / 2; k <= dilate / 2; k++)
	{
		if ((y0 + k) < 0 || (y0 + k) >= map->height)
			continue;
		for (int l = -dilate / 2; l <= dilate / 2; l++)
		{
			if ((x0 + l) < 0 || (x0 + l) >= map->width)
				continue;
			*(map->data + (y0 + k) * map->width + x0 + l) = 100;
		}
	}

	free(mask);
}

bool saveMap(Map * map, const char * filename)
{
	FILE *p_file = fopen(filename, "wb");
	if (p_file)
	{
		int headlength = sizeof(int) * 2 + sizeof(float) * 3 + 1;
		char *buf = (char *)malloc(map->height * map->width + headlength);
		buf[0] = 0x09; //数据头
		for (int i = 0; i < sizeof(int); i++)
		{
			char *charwidth = (char *)&map->width;
			char *charheight = (char *)&map->height;
			buf[i + 1] = charwidth[i];
			buf[sizeof(int) + i + 1] = charheight[i];
		}
		int offset = sizeof(int) * 2 + 1;
		for (int i = 0; i < sizeof(float); i++)
		{
			char *charoffsetx = (char *)&map->xoffset;
			char *charoffsety = (char *)&map->yoffset;
			char *charscale = (char *)&map->scale;
			buf[offset + i] = charoffsetx[i];
			buf[offset + sizeof(float) + i ] = charoffsety[i];
			buf[offset + sizeof(float)*2 + i] = charscale[i];
		}
		
		for (int i = 0; i < map->height * map->width; i++)
		{
			buf[headlength + i] = map->data[i];
		}

		int savednum = 0;
		while (savednum < map->height * map->width + headlength)
		{
			savednum += fwrite(&buf[savednum], 1, map->height * map->width + headlength, p_file);
		}

		free(buf);
		fclose(p_file);

		return true;
	}
	else
	{
		return false;
	}
}

Map loadMap(const char * filename)
{
	FILE *p_file = fopen(filename, "r");
	Map map;
	map.data = NULL;
	if (p_file)
	{
		int headlength = sizeof(int) * 2 + sizeof(float) * 3 + 1;
		char *buf = (char *)malloc(headlength);
		int size = fread(buf, 1, headlength, p_file);
		if (size == headlength && buf[0] == 0x09)
		{ 
			map.width = *((int *)&buf[1]);
			map.height = *((int *)&buf[1 + sizeof(int)]);
			int offset = 2 * sizeof(int) + 1;
 			map.xoffset = *((float *)&buf[offset]);
			map.yoffset = *((float *)&buf[offset + sizeof(float)]);
			map.scale = *((float *)&buf[offset + 2 * sizeof(float)]);
			if (map.width * map.height > 0 && map.width * map.height < 1000000)
			{

				map.data = (unsigned char *)malloc(map.width * map.height);

				int readcount = 0;
				while (readcount < map.width * map.height && size != 0)
				{
					size = fread(&map.data[readcount], 1, map.width * map.height, p_file);
					readcount += size;
				}

				if (size == 0)
				{
					releaseMap(map);
				}
			}

			fclose(p_file);
		}

		free(buf);
	}
	else
	{
		map.data = NULL;
	}
	return map;
}

POLYGON polygon;

Map createGridMap(const char *pointfilename, POLYGON &smoothpolygon)
{
	printf("Creating map...\n");
	NORMEDPOLYGON normedpolygon;
	Map map;
	map.data = NULL;
	float scale = 1.0 / RESOLUTION;

	getPolygon(pointfilename, &polygon);
	printf("Got polygon.\n");
	if (polygon.num <= 3)
	{
		printf("can not create map, polygon edge num is not enough");
		return map;
	}

	float minx;
	float miny;
	float maxx;
	float maxy;

	minMaxPoint(polygon, maxx, minx, maxy, miny);
	printf("Found max and min point.\n");
	int width = (maxx - minx + 2 * OFFSET) * scale;
	int height = (maxy - miny + 2 * OFFSET) * scale;

	map = createMap(width, height, scale);
	printf("Created map.\n");

	map.xoffset = OFFSET - minx;
	map.yoffset = OFFSET - miny;
	
	if (!map.data)
	{
		printf("can not create map data");
		return map;
	}

	memset(map.data, 255, width * height * sizeof(unsigned char));
	//dilateMap(&map, 0.1);
	normalizePolygon(&polygon, &normedpolygon, map);
	printf("Normalized polygon.\n");

	scanFill(normedpolygon.points, normedpolygon.num, map.data, map.width, map.height);

	printf("ScanFill done.\n");

	cv::Mat mat(map.height, map.width, CV_8UC1);
	for(int i=0; i<map.height; i++)
	{
		for(int j=0; j<map.width; j++)
		{
			mat.at<uchar>(i, j) = map.data[i * map.width + j]; 
		}
	}

	
	threshold(mat, mat, 100,  255, THRESH_BINARY_INV);
	cv::erode(mat, mat, cv::Mat());
	printf("Eroded map.\n");

	std::vector<Vec4i> hierarchy;
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(mat, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	printf("Found contours.\n");

	int maxareaid = 0;
	int maxarea = 0;
	for(int i=0; i<contours.size(); i++)
	{
		int temparea = contourArea(contours[i]);
		if(temparea > maxarea)
		{
			maxarea = temparea;
			maxareaid = i;
		}
	}

	cv::drawContours(mat, contours, maxareaid,cv::Scalar(255, 255, 255), -1);
	threshold(mat, mat, 100,  255, THRESH_BINARY_INV);
	//memcpy(map.data, mat.data, width * height * sizeof(unsigned char));

	for(int i=0; i<map.height; i++)
	{
		for(int j=0; j<map.width; j++)
		{
			map.data[i * map.width + j] = mat.at<uchar>(i, j); 
		}
	}

	//imshow("mat", mat);
	//waitKey(0);

	imwrite("/home/raspberry002/catkin_ws/src/zigzag_mower/map.jpg", mat);
	// imwrite("/home/yat20/simulation_ws/src/zigzag_mower/map.jpg", mat);

	printf("Draw contours.\n");

	normalizePolygon2(contours[maxareaid], &smoothpolygon, map);

	printf("Map compeleted!\n");
	return map;
}

void scanFill(YAT_POINT* polygon, int polygonNum, unsigned char *pImage, int nWidth, int nHeight)
{
	YAT_EDGE  **edges, *active;
	int        i, scan, scanmax = 0, scanmin = nHeight;
	//计算所有多边形顶点坐标中y的最大值和最小值，以此作为扫描线的处理范围
	for (i = 0; i < polygonNum/* - 1*/; i++)
	{
		if (scanmax < polygon[i].y)
		{
			scanmax = polygon[i].y;
		}
		if (scanmin > polygon[i].y)
		{
			scanmin = polygon[i].y;
		}
	}
	//初始化每条扫描线的边链表
	edges = (YAT_EDGE**)malloc(sizeof(YAT_EDGE*)*(scanmax - scanmin + 1));
	for (scan = scanmin; scan <= scanmax; scan++)
	{
		edges[scan - scanmin] = NULL;
	}
	buildEdgeList(polygonNum, polygon, edges, scanmin);         //建立有序边表
	active = NULL;
	for (scan = scanmin; scan <= scanmax; scan++)      //扫描每条扫描线，求活性表
	{
		buildActiveList((scan - scanmin), &active, edges);          //建立活性边表
		if (active)
		{
			fillScan(scan, active, pImage, nWidth);             //填充当前扫描线
			updateActiveList(scan, &active);            //更新活化边表
			resortActiveList(&active);                 //重排活化边表

			/*cv::Mat temp(cv::Size(nWidth, nHeight), CV_8UC1, pImage);

			imshow("scan", temp);
			cv::waitKey(100);*/
		}
	}
	free((void*)edges);
}

void buildEdgeList(int cnt, YAT_POINT* pts, YAT_EDGE* edges[], int yMin)
{
	YAT_EDGE*     edge;
	YAT_POINT     ps, pe, pss, pee;
	int           i;
	edge = (YAT_EDGE*)malloc(sizeof(YAT_EDGE));
	if (edge == NULL)
	{
		printf("edge malloc failed!\r\n");
		return;
	}
	for (i = 0; i < cnt; i++)
	{
		ps = pts[i];                       //当前处理边的起点
		pe = pts[(i + 1) % cnt];           //当前处理边的终点
		pss = pts[(i - 1 + cnt) % cnt];    //起点的前一个相邻点
		pee = pts[(i + 2) % cnt];          //终点的后一个相邻点
		if (ps.y != pe.y)                 //非水平线
		{
			edge->dx = (float)(pe.x - ps.x) / (float)(pe.y - ps.y);

			if (pe.y > ps.y)                             //当前顶点不是奇点，建立边表时使用下一个顶点的y值即yNext
			{
				edge->x = ps.x;
				if (pee.y >= pe.y)
				{
					edge->ymax = pe.y - 1;
				}
				else
				{
					edge->ymax = pe.y;
				}
				pushEdgeList(&edges[ps.y - yMin], edge);
			}
			else
			{
				edge->x = pe.x;
				if (pss.y >= ps.y)
				{
					edge->ymax = ps.y - 1;
				}
				else
				{
					edge->ymax = ps.y;
				}
				pushEdgeList(&edges[pe.y - yMin], edge);
			}
		}
	}
	free(edge);
}

void buildActiveList(int scan, YAT_EDGE **pactive, YAT_EDGE* edges[])
{
	YAT_EDGE *first= NULL; /*为原链表剩下用于直接插入排序的节点头指针*/
	YAT_EDGE *t; /*临时指针变量：插入节点*/
	YAT_EDGE *p = NULL; /*临时指针变量*/
	YAT_EDGE *q = NULL; /*临时指针变量*/
		
	first = edges[scan]; /*原链表剩下用于直接插入排序的节点链表*/

	while (first != NULL) /*遍历剩下无序的链表*/
	{
		for (t = first, q = *pactive; ((q != NULL) && (q->x < t->x)); p = q, q = q->Next); /*无序节点在有序链表中找插入的位置*/
																				  /*退出for循环，就是找到了插入的位置*/
		first = first->Next; /*无序链表中的节点离开，以便它插入到有序链表中。*/

		if (q == *pactive) /*插在第一个节点之前*/
		{
			*pactive = t;
		}
		else /*p是q的前驱*/
		{
			p->Next = t;
		}
		t->Next = q; /*完成插入动作*/
	}
}


void fillScan(int scan, YAT_EDGE* active, unsigned char* pImage, int nWidth)
{
	YAT_EDGE  *p1, *p2;
	int       i;
	p1 = active;
	while (p1)
	{
		p2 = p1->Next;
		for (i = p1->x; i < p2->x; i++)
		{
			pImage[scan * nWidth + i] = 0;       //对图形内部的点置1
		}
		//for (i = p2->x; i < p1->x; i++)
		//{
		//	pImage[scan * nWidth + i] = 255;       //对图形内部的点置1
		//}
		p1 = p2->Next;		                                     //活性表的下一条边表

	}
}

void updateActiveList(int scan, YAT_EDGE **pactive)
{
	YAT_EDGE  *q = *pactive;
	YAT_EDGE  *p = *pactive;
	YAT_EDGE  *pDelete = *pactive;
	while (p)
	{
		if (scan >= p->ymax)              //扫描线超过边的最大y值，此条件的节点应该删掉
		{
			//删除表头
			if (*pactive == p)
			{
				*pactive = p->Next;
				pDelete = p;
				p = p->Next;
				q = p;
				free(pDelete);
			}
			else
			{
				//删除列表中间的结点
				q->Next = p->Next;
				pDelete = p;
				p = p->Next;
				free(pDelete);
			}
		}
		else                               //扫描线未超过边的最大y值，相应的x值增加
		{
			p->x = p->x + p->dx;
			q = p;
			p = p->Next;
		}
	}
}

void resortActiveList(YAT_EDGE **pactive)
{
	YAT_EDGE *q;
	YAT_EDGE *p = *pactive;
	YAT_EDGE *pMin;
	YAT_EDGE *pChange;
	int         nMin;
	pChange = (YAT_EDGE*)malloc(sizeof(YAT_EDGE));
	while (p)
	{
		nMin = p->x;
		pMin = p;
		q = p;
		while (q)
		{
			if (nMin > q->x)
			{
				pMin = q;
				nMin = q->x;
			}
			q = q->Next;
		}
		if (pMin != p)
		{
			pChange->x = pMin->x;
			pChange->dx = pMin->dx;
			pChange->ymax = pMin->ymax;
			pMin->x = p->x;
			pMin->dx = p->dx;
			pMin->ymax = p->ymax;
			p->x = pChange->x;
			p->dx = pChange->dx;
			p->ymax = pChange->ymax;
		}
		p = p->Next;
	}
	free(pChange);
}


void pushEdgeList(YAT_EDGE **pList, YAT_EDGE* Edge)
{
	YAT_EDGE   *p, *p2;
	p = *pList;
	p2 = *pList;
	if (*pList)
	{
		while (p)
		{
			p2 = p;
			p = p->Next;
		}
		p = (YAT_EDGE*)malloc(sizeof(YAT_EDGE));
		p->x = Edge->x;
		p->dx = Edge->dx;
		p->ymax = Edge->ymax;
		p->Next = NULL;
		p2->Next = p;
	}
	else
	{
		*pList = (YAT_EDGE*)malloc(sizeof(YAT_EDGE));
		(*pList)->x = Edge->x;
		(*pList)->dx = Edge->dx;
		(*pList)->ymax = Edge->ymax;
		(*pList)->Next = NULL;
	}
}

bool isInMap(Map *map, YAT_POINTF pointf)
{
	YAT_POINT point;
	point.x = (map->xoffset + pointf.x) * map->scale;
	point.y = (map->yoffset + pointf.y) * map->scale;

    bool ret = (map->data[point.y * map->width + point.x] < 100);
	return ret;
}

bool isPointInMap(Map *map, YAT_POINT point)
{
    bool ret = (point.x >= 0 && point.x < map->width && point.y >= 0 && point.y < map->height && map->data[point.y * map->width + point.x] == 0);
	return ret;
}

void setUsed(Map *map, YAT_POINT point)
{
    map->data[point.y * map->width + point.x] = 1;
}

bool isOutOfRange(Map *map, YAT_POINT point)
{
    if(point.x > map->width || point.y > map->height || point.x < 0 || point.y < 0)
	{
		return true;
	}
	return false;
}

bool isWarnedRange(Map *map, YAT_POINTF pointf)
{
	YAT_POINT point;
	point.x = (map->xoffset + pointf.x) * map->scale;
	point.y = (map->yoffset + pointf.y) * map->scale;

    bool ret = (map->data[point.y * map->width + point.x] == 100);
	return ret;
}

/*
// void getEdgePoint(Map *map, YAT_POINTF point, YAT_POINTF vector_wall)
float getWallHeading(Map *map, YAT_POINTF point)
{
    YAT_POINTF pointout[2];
    float mindist = 1000000;
    int edgeid = 0;
    for (int i = 0; i < polygon.num; i++)
    {
            float dist = (polygon.points[i].x - point.x) * (polygon.points[i].x - point.x) + (polygon.points[i].y - point.y) * (polygon.points[i].y - point.y);
            if (dist < mindist)
            {
                    mindist = dist;
                    edgeid = i;
            }
    }

    if (edgeid < 2)
            pointout[0] = polygon.points[0];
    else
            pointout[0] = polygon.points[edgeid - 1];

    if (edgeid > polygon.num - 2)
            pointout[1] = polygon.points[edgeid];
    else
            pointout[1] = polygon.points[edgeid + 1];

//    if (edgeid == 0)
//            pointout[0] = polygon.points[0];
//    else
//            pointout[0] = polygon.points[edgeid - 1];

//    if (edgeid == polygon.num - 1)
//            pointout[1] = polygon.points[edgeid];
//    else
//            pointout[1] = polygon.points[edgeid + 1];

	// printf("point[0].x:%f,point[0].y:%f,point[1].x:%f,point[1].y:%f,",pointout[0].x,pointout[0].y,pointout[1].x,pointout[1].y);
    return line_heading(pointout[0],pointout[1]);
}
*/


YAT_POINT world_to_map(Map *map, YAT_POINTF pointworld)
{
	YAT_POINT point;
	point.x = (map->xoffset + pointworld.x) * map->scale;
	point.y = (map->yoffset + pointworld.y) * map->scale;

	return point;
}

YAT_POINTF map_to_world(Map *map, YAT_POINT pointmap)
{
	YAT_POINTF pointf;
	pointf.x = pointmap.x / map->scale - map->xoffset;
	pointf.y = pointmap.y / map->scale - map->yoffset;

	return pointf;
}

//navigation frame: x to the right side, y to the up, anti-clockwise as positive
float line_heading(YAT_POINTF start, YAT_POINTF end)
{
    float delta_x = end.x - start.x;		//△x
    float delta_y = end.y - start.y;		//△y
    float theta;
    float PI=3.1416;

    if (fabs(delta_x) <0.01 || fabs(delta_y) < 0.01)			//直线与x轴垂直或飞行
    {
        if (fabs(delta_x) <0.01 && delta_y > 0)
            theta = 90;
        if (fabs(delta_x) <0.01 && delta_y < 0)
            theta = -90;
        if (fabs(delta_y) < 0.01 && delta_x > 0)
            theta = 0;
        if (fabs(delta_y) < 0.01 && delta_x < 0)
            theta = 180;
    }
    else 
    {
        theta = atan2(delta_y, delta_x) * 180 / PI; //
    }
	if (fabs(theta)<0.0001) theta=0;

    return theta;
}


/* local frame: robot heading as x , y to the left 
float line_heading(YAT_POINTF start, YAT_POINTF end)
{
    float delta_x = end.x - start.x;		//△x
    float delta_y = end.y - start.y;		//△y
    float theta;
    float PI=3.1416;

    if (fabs(delta_x) <0.01 || fabs(delta_y) < 0.01)			//直线与x轴垂直或飞行
    {
        if (fabs(delta_x) <0.01 && delta_y > 0)
            theta = 0;
        if (fabs(delta_x) <0.01 && delta_y < 0)
            theta = 180;
        if (fabs(delta_y) < 0.01 && delta_x > 0)
            theta = -90;
        if (fabs(delta_y) < 0.01 && delta_x < 0)
            theta = 90;
    }
    else if (delta_x > 0)   //1,4 quadrant
    {
        theta = (atan(delta_y / delta_x) * 180 / PI) - 90; //
    }
    else if (delta_x < 0)   //2,3 quadrant
    {
        theta = (atan(delta_y / delta_x) * 180 / PI) + 90;
    }


    if (delta_x == 0 || delta_y == 0)			//直线与x轴垂直或飞行
    {
        if (delta_x == 0 && delta_y > 0)
            theta = 90;
        if (delta_x == 0 && delta_y < 0)
            theta = 270;
        if (delta_y == 0 && delta_x > 0)
            theta = 0;
        if (delta_y == 0 && delta_x < 0)
            theta = 180;
    }
    else if (delta_x > 0)
    {
        if(delta_y > 0)	//1 quateron
            theta = atan(delta_y / delta_x) * 180 / PI;
        else		//4 quateron
            theta = atan(delta_y / delta_x) * 180 / PI + 360;
    }
    else if (delta_x < 0 )
    {
        theta = atan(delta_y / delta_x) * 180 / PI + 180;	//2,3 quateron
    }
    *

    return theta;
}

*/

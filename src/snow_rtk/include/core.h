#ifndef _TOOL_H
#define _TOOL_H

#include <malloc.h>
#include "stdio.h"

#define MAXPOINTNUM 10000  //最大围圈点数
#define RESOLUTION 0.2      //每一格的宽度  /m
#define OFFSET 2          //地图边界预留 /m

typedef struct _YAT_TEDGE
{
	float  x;
	float  dx;
	int    ymax;
	struct _YAT_TEDGE*   Next;
}YAT_EDGE;

typedef struct _YAT_POINT
{
	int  x;
	int  y;
}YAT_POINT;

typedef struct _YAT_POINTF
{
	float  x;
	float  y;
}YAT_POINTF;

typedef struct _YAT_POINTF_THETA
{
	float  x;
	float  y;
	float  theta;
}_YAT_POINTF_THETA;

typedef struct _Map
{
	int width;
	int height;
	unsigned char *data;

	float xoffset;
	float yoffset;

	float scale;
} Map;


typedef struct _POLYGON
{
	int num;
	YAT_POINTF points[MAXPOINTNUM];
} POLYGON;

typedef struct _YAT_PATH
{
	int num;
	int totalnum;
	YAT_POINT *ppath;
} YAT_PATH;

typedef struct _NORMEDPOLYGON
{
	int num;
	YAT_POINT points[MAXPOINTNUM];
} NORMEDPOLYGON;

#endif


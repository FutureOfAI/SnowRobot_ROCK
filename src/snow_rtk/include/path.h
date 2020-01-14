#ifndef PATH_PLAN_PATH_H
#define PATH_PLAN_PATH_H

#include "astar.h"
#include "blockallocator.h"
#include "contour.h"
#include "core.h"
#include <vector>

void get_path_all(Map *map, YAT_PATH *path, YAT_POINT point_start, std::vector<YAT_POINT> &out_corner_path);

void getPathV(Map *map, YAT_PATH *pathV, YAT_POINT origin, std::vector<YAT_POINT> &outCornerPath);

void get_path(Map *map, YAT_PATH *path, YAT_POINT point_start, std::vector<YAT_POINT> &out_corner_path);

YAT_POINT find_next_rigion_left(Map *map);

YAT_PATH create_path(Map *map);

void release_path(YAT_PATH *path);

bool interval_path(AStar &algorithm, AStar::Params &param, YAT_PATH *path, YAT_POINT nextpoint);

bool add_path(YAT_PATH *path, YAT_POINT point);

//void right_path(YAT_POINT startpoint, Map *map, YAT_PATH *path);

bool turn_first_right(YAT_POINT *point_current, Map *map, YAT_PATH *path);

bool turn_second_right(YAT_POINT *point_current, Map *map, YAT_PATH *path);

bool right_toward_down(YAT_POINT *point_current, Map *map, YAT_PATH *path);

void right_path(YAT_POINT startpoint, Map *map, YAT_PATH *path);

void occupate_map(YAT_POINT startpoint, Map *map);

//void left_path(YAT_POINT startpoint, Map *map, YAT_PATH *path);

bool turn_first_left(YAT_POINT *point_current, Map *map, YAT_PATH *path);

bool turn_second_left(YAT_POINT *point_current, Map *map, YAT_PATH *path);

bool left_toward_up(YAT_POINT *point_current, Map *map, YAT_PATH *path);

void left_path(YAT_POINT startpoint, Map *map, YAT_PATH *path);

//latitude and longitutde in degree
//returned: xyz in earth ,//in earth-centered frame
//void get_XYZcoordinates(double latitude, double longititude,double xyz[3]);

#endif

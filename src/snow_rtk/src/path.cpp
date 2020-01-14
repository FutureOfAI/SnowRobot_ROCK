#include "path.h"
#include "iostream"

//opencv
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"

using namespace cv;


Map createHmap(Map *map)
{
	Map Hmap;
	Hmap.width = map->height;
	Hmap.height = map->width;
	Hmap.scale = map->scale;
	Hmap.data = (unsigned char *)malloc(Hmap.width  * Hmap.height * sizeof(unsigned char));

    for(int i=0; i<Hmap.height; i++)
    {
        for(int j=0; j<Hmap.width; j++)
        {
            Hmap.data[Hmap.width * i + j] = map->data[map->width *j + i];
        }
    }

    Mat Himg(Hmap.height, Hmap.width ,CV_8UC1);

    // for(int i=0; i<Hmap.height; i++)
    // {
    //     for(int j=0; j<Hmap.width; j++)
    //     {
    //         Himg.at<uchar>(i, j) = Hmap.data[i * Hmap.width + j];
    //     }
    // }

    // Mat img(map->height, map->width ,CV_8UC1);

    // for(int i=0; i<map->height; i++)
    // {
    //     for(int j=0; j<map->width; j++)
    //     {
    //         img.at<uchar>(i, j) = map->data[i * map->width + j];
    //     }
    // }

    // imshow("map", img);
    // imshow("Hmap", Himg);
    // waitKey(0);
    return Hmap;
}

void pathtoVpath(YAT_PATH *path)
{
    for(int i=0; i<path->num; i++)
    {
        int temp = path->ppath[i].x;
        path->ppath[i].x = path->ppath[i].y;
        path->ppath[i].y = temp;
    }
}

void getPathV(Map *map, YAT_PATH *pathV, YAT_POINT origin, std::vector<YAT_POINT> &outCornerPath)
{
     Map Hmap = createHmap(map);

    YAT_PATH Hpath = create_path(&Hmap);
    std::vector<YAT_POINT>  HoutCornerPath;

    YAT_POINT Horigin = origin;

    get_path(&Hmap, &Hpath, Horigin, HoutCornerPath);

    //std::cout<<Hpath.num<<std::endl;
    //std::cout<<Horigin.x <<" "<<Horigin.y <<std::endl;
    
    pathtoVpath(&Hpath);


    for(int i=0; i<HoutCornerPath.size(); i++)
    {
        int temp = HoutCornerPath[i].x;
        HoutCornerPath[i].x = HoutCornerPath[i].y;
        HoutCornerPath[i].y = temp;
    }

    for(int i=0; i<Hpath.num; i++)
    {
        add_path(pathV, Hpath.ppath[i]);
    }


    for(int i=0; i<HoutCornerPath.size(); i++)
    {
        outCornerPath.push_back(HoutCornerPath[i]);
    }
}

void get_path_all(Map *map, YAT_PATH *pathall, YAT_POINT origin, std::vector<YAT_POINT> &outCornerPathall)
{
    get_path(map, pathall, origin, outCornerPathall);

    Map Hmap = createHmap(map);

    YAT_PATH Hpath = create_path(&Hmap);
    std::vector<YAT_POINT>  HoutCornerPath;

    YAT_POINT Horigin = pathall->ppath[pathall->num - 1];

    get_path(&Hmap, &Hpath, Horigin, HoutCornerPath);

    pathtoVpath(&Hpath);

    for(int i=0; i<HoutCornerPath.size(); i++)
    {
        int temp = HoutCornerPath[i].x;
        HoutCornerPath[i].x = HoutCornerPath[i].y;
        HoutCornerPath[i].y = temp;
    }

    for(int i=0; i<Hpath.num; i++)
    {
        add_path(pathall, Hpath.ppath[i]);
    }


    for(int i=0; i<HoutCornerPath.size(); i++)
    {
        outCornerPathall.push_back(HoutCornerPath[i]);
    }

    release_path(&Hpath);
    releaseMap(Hmap);
}

void get_path(Map *map, YAT_PATH *path, YAT_POINT point_start, std::vector<YAT_POINT> &out_corner_path)
{
    Mat temp(map->height, map->width ,CV_8UC1, map->data); 

    //std::cout<<temp<<std::endl;

    //circle(temp, Point(46, 34), 5, Scalar(255,255,255), 4);

    map->data = temp.data;
    Mat img(map->height, map->width ,CV_8UC1);

    for(int i=0; i<map->height; i++)
    {
        for(int j=0; j<map->width; j++)
        {
            img.at<uchar>(i, j) = map->data[i * map->width + j];
        }
    }

    resize(img, img, Size(map->width * 5, map->height * 5), INTER_AREA);
    cvtColor(img, img, COLOR_GRAY2RGB);

    // 搜索参数
    AStar::Params param;
    param.width = map->width;
    param.height = map->height;
    param.corner = false;
    param.can_pass = [&](const AStar::Vec2 &pos)->bool
    {
         return map->data[pos.y * map->width + pos.x] < 100;
    };

    // 执行搜索
    BlockAllocator allocator;
    AStar algorithm(&allocator);

    right_path(point_start, map, path);
    int linenum = 0;
    for(int i = linenum; i<path->num-1; i++)
    {
        line(img, Point(path->ppath[i].x * 5 , path->ppath[i].y * 5), Point(path->ppath[i+1].x * 5 , path->ppath[i+1].y * 5), Scalar(255,255,0), 1);
    }
    linenum = path->num;

    interval_path(algorithm, param, path, point_start);
    for(int i = linenum; i<path->num-1; i++)
    {
        line(img, Point(path->ppath[i].x * 5 , path->ppath[i].y * 5), Point(path->ppath[i+1].x * 5 , path->ppath[i+1].y * 5), Scalar(255,0,0), 1);
    }
    linenum = path->num;
    
    left_path(point_start, map, path);

    for(int i = linenum; i<path->num-1; i++)
    {
        line(img, Point(path->ppath[i].x * 5 , path->ppath[i].y * 5), Point(path->ppath[i+1].x * 5 , path->ppath[i+1].y * 5), Scalar(255,0,255), 1);
    }
    linenum = path->num;

    while(1)
    {
        YAT_POINT next_region_start_point = find_next_rigion_left(map);
        if(next_region_start_point.x < 0)
        {
            break;
        }
        if(interval_path(algorithm, param, path, next_region_start_point))
        {
            for(int i = linenum; i<path->num-1; i++)
            {
                line(img, Point(path->ppath[i].x * 5 , path->ppath[i].y * 5), Point(path->ppath[i+1].x * 5 , path->ppath[i+1].y * 5), Scalar(0,255,255), 1);
            }
            linenum = path->num;

            right_path(next_region_start_point, map, path);

            for(int i = linenum; i<path->num-1; i++)
            {
                line(img, Point(path->ppath[i].x * 5 , path->ppath[i].y * 5), Point(path->ppath[i+1].x * 5 , path->ppath[i+1].y * 5), Scalar(0,0,255), 1);
            }
            linenum = path->num;
        }
        else
        {

            occupate_map(next_region_start_point, map);
        }
    }

    for(int i=0; i<map->height; i++)
    {
        for(int j=0; j<map->width; j++)
        {
            if(map->data[map->width * i + j] == 1)
                map->data[map->width * i + j] = 0;
        }
    }


    //Corner Path
    std::vector<int> cornerID;
    std::vector<YAT_POINT> cornerpath;
    cornerpath.push_back(path->ppath[0]);
    cornerID.push_back(0);
    for(int i = 1; i<path->num-1 ;i++)
    {
        YAT_POINT front = path->ppath[i-1];
        YAT_POINT cur = path->ppath[i];
        YAT_POINT next = path->ppath[i+1];

        if(cur.x != front.x || cur.x != next.x)
        {
            cornerpath.push_back(cur);
            cornerID.push_back(i);
        }
    }

    //Excute Path
    float cornerDist = 0.0;
    int insert_num = 0;
    out_corner_path.push_back(cornerpath[0]);
    for(int i = 0; i < cornerpath.size() - 1; i++)
    {
        if(out_corner_path[out_corner_path.size() - 1].x != cornerpath[i].x || out_corner_path[out_corner_path.size() - 1].y != cornerpath[i].y)
        {
            out_corner_path.push_back(cornerpath[i]);
        }
        
        cornerDist = cornerID[i+1] - cornerID[i];
        int map_interval = 5 / RESOLUTION;
        insert_num = cornerDist / map_interval;

        for(int j=0; j<insert_num; j++)
        {
            int insert_id = (j + 1) * (cornerID[i+1] - cornerID[i]) /insert_num  + cornerID[i];
            out_corner_path.push_back(path->ppath[insert_id]);
        }
    }

    out_corner_path.push_back(cornerpath[cornerpath.size() - 1]);

    //Calib Path, 筛选靠近底边的点作为校准点 
    if(cornerpath[0].y < cornerpath[1].y)
        //calib_path.push_back(cornerpath[0]);
    for(int i = 1; i < cornerpath.size() - 1 ; i++)
    {
        YAT_POINT last_calib_point;
        if(cornerpath[i].x == cornerpath[0].x && cornerpath[i].y == cornerpath[0].y)
        {
            continue;
        }
        else if(cornerpath[i].y < cornerpath[i-1].y && fabs(cornerpath[i].x - last_calib_point.x) > 1)     //找到直线上y较小的角点作为校准点    
        {
            //calib_path.push_back(cornerpath[i]);
            last_calib_point = cornerpath[i];
        }
    }

    for(int i=0; i<cornerpath.size(); i++)
    {
         circle(img, Point(cornerpath[i].x * 5 , cornerpath[i].y * 5), 5, Scalar(255,0,255));
    }

    circle(img, Point(path->ppath[0].x * 5 , path->ppath[0].y * 5), 5, Scalar(255,255,255));
    circle(img, Point(path->ppath[path->num -1].x * 5 , path->ppath[path->num -1].y * 5), 5, Scalar(255,255,255));

    // imshow("dst", img);
    // waitKey(0);

}

void release_path(YAT_PATH *path)
{
	path->totalnum = 0;
	path->num = 0;
	free(path->ppath); 
}

bool interval_path(AStar &algorithm, AStar::Params &param, YAT_PATH *path, YAT_POINT nextpoint)
{
    // param.start = AStar::Vec2(path->ppath[path->num-2].x, path->ppath[path->num-2].y);
    // param.end = AStar::Vec2(nextpoint.x, nextpoint.y);
    // std::vector<AStar::Vec2> pathinterval = algorithm.find(param);

    // if(pathinterval.size() == 0)
    // {
    //     return false;
    // }

    // for(int i=0; i<pathinterval.size(); i ++)
    // {
    //     YAT_POINT point;
    //     point.x = pathinterval[i].x;
    //     point.y = pathinterval[i].y;
    //     add_path(path, point);
    //     //circle(img, Point(pathinterval[i].x * 10 , pathinterval[i].y * 10), 5, Scalar(color,0,0));
    // }
    return true;
}


YAT_PATH create_path(Map *map)
{
	// int num = 0;
	// for(int i=0; i<map->height; i++)
	// {
	// 	for(int j=0; j<map->width; j++)
	// 	{
	// 		if(map->data[i * map->width + j] == 0)
	// 		{
	// 			num ++;
	// 		}
	// 	}
	// }

	YAT_PATH path;
	//path.totalnum = num +1;
    path.totalnum = map->height * map->width;
	path.num = 0;
    path.ppath = (YAT_POINT *)malloc(path.totalnum * sizeof(YAT_POINT));
	return path;
}

bool add_path(YAT_PATH *path, YAT_POINT point)
{
	if(path->num > (path->totalnum - 1))
		return false;
	path->ppath[path->num] = point;
	path->num++;
	return true;
}

// void left_path(YAT_POINT point_start, Map *map, YAT_PATH *path)
// {
// 	//YAT_POINT point_start = world_to_map(map, startpoint);
// 	YAT_POINT point_current = point_start;
// 	YAT_POINT point_next;
// 	add_path(path,point_start);
// 	setUsed(map, point_start);
//     bool searching = true;
   
// 	while(searching)
//     {
//          bool searchingup = true;
// 		//toward down
// 		point_next.x = point_current.x;
// 		point_next.y = point_current.y - 1;

// 		if(isOutOfRange(map, point_next))
// 		{
// 			searching = false;
// 			break;
// 		}

// 		if(isPointInMap(map, point_next))
// 		{
// 			add_path(path,point_next);
// 			setUsed(map, point_next);
// 			point_current = point_next;
// 		}
// 		else
// 		{
// 			while(searching)
// 			{
// 				//toward left
// 				point_next.x = point_current.x - 1;
// 				point_next.y += 1;	

// 				//out of map
// 				if(isOutOfRange(map, point_next))
// 				{
// 					searching = false;
// 					break;
// 				}

// 				if(isPointInMap(map, point_next))
// 				{
// 					add_path(path,point_next);
// 					setUsed(map, point_next);
// 					point_current = point_next;
// 					break;
//                 }
// 			}

// 			//toward up
//             while(searchingup && searching)
// 			{
// 				point_next.x = point_current.x;
// 				point_next.y = point_current.y + 1;

// 				//out of map
// 				if(isOutOfRange(map, point_next))
// 				{
// 					searching = false;
// 					break;
// 				}

// 				if(isPointInMap(map, point_next))
// 				{
// 					add_path(path,point_next);
// 					setUsed(map, point_next);
// 					point_current = point_next;
// 				}
// 				else
// 				{
//                     while(1)
//                     {
//                         //toward left
//                         point_next.x = point_current.x - 1;
//                         point_next.y -= 1;

//                         //out of map
//                         if(isOutOfRange(map, point_next))
//                         {
//                             searching = false;
//                             break;
//                         }

//                         if(isPointInMap(map, point_next))
//                         {
//                             add_path(path,point_next);
//                             setUsed(map, point_next);
//                             point_current = point_next;
//                             searchingup = false;
//                             break;
//                         }
//                     }
//                 }
// 			}
// 		}
// 	}
// }


bool turn_first_right(YAT_POINT *point_current, Map *map, YAT_PATH *path)
{
    YAT_POINT point_next;
    point_next.x = point_current->x + 1;
    point_next.y = point_current->y;


    //toward top right
    YAT_POINT last_point{-1, -1};
    while(isPointInMap(map, point_next))
    {
        last_point = point_next;
        point_next.y++;
    }

    if(last_point.x > 0)
    {
        add_path(path, last_point);
        setUsed(map, last_point);
        *point_current = last_point;
        return true;
    }
    
    //toward down right
    while(true)
    {
        point_next.y -= 1;
        //out of map
        if(isOutOfRange(map, point_next))
        {
            return false;
        }

        if(isPointInMap(map, point_next))
        {
            add_path(path,point_next);
            setUsed(map, point_next);
            *point_current = point_next;
            return true;
        }
    }
}

bool turn_first_left(YAT_POINT *point_current, Map *map, YAT_PATH *path)
{
    YAT_POINT point_next;
    point_next.x = point_current->x - 1;
    point_next.y = point_current->y;


    //toward down left
    YAT_POINT last_point{-1, -1};
    while(isPointInMap(map, point_next))
    {
        last_point = point_next;
        point_next.y--;
    }

    if(last_point.x > 0)
    {
        add_path(path,last_point);
        setUsed(map, last_point);
        *point_current = last_point;
        return true;
    }
    
    //toward up left
    while(true)
    {
        point_next.y += 1;
        //out of map
        if(isOutOfRange(map, point_next))
        {
            return false;
        }

        if(isPointInMap(map, point_next))
        {
            add_path(path,point_next);
            setUsed(map, point_next);
            *point_current = point_next;
            return true;
        }
    }
}

bool turn_second_right(YAT_POINT *point_current, Map *map, YAT_PATH *path)
{
    YAT_POINT point_next;
    point_next.x = point_current->x + 1;
    point_next.y = point_current->y;


    //toward down right
    YAT_POINT last_point{-1, -1};
    while(isPointInMap(map, point_next))
    {
        last_point = point_next;
        point_next.y--;
    }

    if(last_point.x > 0)
    {
        add_path(path, last_point);
        setUsed(map, last_point);
        *point_current = last_point;
        return true;
    }

    while(true)
    {
        point_next.y += 1;
        //out of map
        if(isOutOfRange(map, point_next))
        {
            return false;
        }

        if(isPointInMap(map, point_next))
        {
            add_path(path,point_next);
            setUsed(map, point_next);
            *point_current = point_next;
            return true;
        }    
    }
}

bool turn_second_left(YAT_POINT *point_current, Map *map, YAT_PATH *path)
{
    YAT_POINT point_next;
    point_next.x = point_current->x - 1;
    point_next.y = point_current->y;


    //toward up left
    YAT_POINT last_point{-1, -1};
    while(isPointInMap(map, point_next))
    {
        last_point = point_next;
        point_next.y++;
    }

    if(last_point.x > 0)
    {
        add_path(path,last_point);
        setUsed(map, last_point);
        *point_current = last_point;
        return true;
    }

    while(true)
    {
        point_next.y -= 1;
        //out of map
        if(isOutOfRange(map, point_next))
        {
            return false;
        }

        if(isPointInMap(map, point_next))
        {
            add_path(path,point_next);
            setUsed(map, point_next);
            *point_current = point_next;
            return true;
        }    
    }
}

bool right_toward_down(YAT_POINT *point_current, Map *map, YAT_PATH *path)
{
    //toward down

    YAT_POINT point_next;

    while(true)
    {
        point_next.x = point_current->x;
        point_next.y = point_current->y - 1;

        //out of map
        if(isOutOfRange(map, point_next))
        {
            return  false;
        }

        if(isPointInMap(map, point_next))
        {
            add_path(path,point_next);
            setUsed(map, point_next);
            *point_current = point_next;
        }
        else
        {
            if(turn_second_right(point_current, map, path))
            {
                break;
            }
            else
            {
                return false;
            }
        }
    }
}

bool left_toward_up(YAT_POINT *point_current, Map *map, YAT_PATH *path)
{
    //toward up

    YAT_POINT point_next;

    while(true)
    {
        point_next.x = point_current->x;
        point_next.y = point_current->y + 1;

        //out of map
        if(isOutOfRange(map, point_next))
        {
            return  false;
        }

        if(isPointInMap(map, point_next))
        {
            add_path(path,point_next);
            setUsed(map, point_next);
            *point_current = point_next;
        }
        else
        {
            if(turn_second_left(point_current, map, path))
            {
                break;
            }
            else
            {
                return false;
            }
        }
    }
}

void occupate_map(YAT_POINT point_start, Map *map)
{
    //YAT_POINT point_start = world_to_map(map, startpoint);
	YAT_POINT point_current = point_start;
	YAT_POINT point_next;
	setUsed(map, point_start);
    bool searching = true;
   
	while(searching)
    {
		//toward up
		point_next.x = point_current.x;
		point_next.y = point_current.y + 1;

		if(isOutOfRange(map, point_next))
		{
			searching = false;
			break;
		}

		if(isPointInMap(map, point_next))
		{
			setUsed(map, point_next);
			point_current = point_next;
		}
		else
		{
			while(searching)
			{
				//toward right
				point_next.x = point_current.x + 1;
				point_next.y -= 1;	

				//out of map
				if(isOutOfRange(map, point_next))
				{
					searching = false;
					break;
				}

				if(isPointInMap(map, point_next))
				{
					setUsed(map, point_next);
					point_current = point_next;
					break;
                }
			}

			//toward down
            bool searchingdown = true;
            while(searchingdown && searching)
			{
				point_next.x = point_current.x;
				point_next.y = point_current.y - 1;

				//out of map
				if(isOutOfRange(map, point_next))
				{
					searching = false;
					break;
				}

				if(isPointInMap(map, point_next))
				{
					setUsed(map, point_next);
					point_current = point_next;
				}
				else
				{
                    while(1)
                    {
                        //toward right
                        point_next.x = point_current.x + 1;
                        point_next.y += 1;

                        //out of map
                        if(isOutOfRange(map, point_next))
                        {
                            searching = false;
                            break;
                        }

                        if(isPointInMap(map, point_next))
                        {
                            setUsed(map, point_next);
                            point_current = point_next;
                            searchingdown = false;
                            break;
                        }
                    }
                }
			}
		}
	}
}

void right_path(YAT_POINT point_start, Map *map, YAT_PATH *path)
{
	//YAT_POINT point_start = world_to_map(map, startpoint);
	YAT_POINT point_current = point_start;
	YAT_POINT point_next;
	add_path(path,point_start);
	setUsed(map, point_start);
    bool searching = true;
   
	while(searching)
    {
		//toward up
		point_next.x = point_current.x;
		point_next.y = point_current.y + 1;

		if(isOutOfRange(map, point_next))
		{
			searching = false;
			break;
		}

		if(isPointInMap(map, point_next))
		{
			add_path(path,point_next);
			setUsed(map, point_next);
			point_current = point_next;
		}
		else
		{
			if(!turn_first_right(&point_current, map, path))
            {
                break;
            }

		//toward down

            if(!right_toward_down(&point_current, map, path))
            {
                break;
            }

		}
	}
}

void left_path(YAT_POINT point_start, Map *map, YAT_PATH *path)
{
	//YAT_POINT point_start = world_to_map(map, startpoint);
	YAT_POINT point_current = point_start;
	YAT_POINT point_next;
	add_path(path,point_start);
	setUsed(map, point_start);

   
	while(true)
    {
		//toward down
		point_next.x = point_current.x;
		point_next.y = point_current.y - 1;

		if(isOutOfRange(map, point_next))
		{
			break;
		}

		if(isPointInMap(map, point_next))
		{
			add_path(path,point_next);
			setUsed(map, point_next);
			point_current = point_next;
		}
		else
		{
			if(!turn_first_left(&point_current, map, path))
            {
                break;
            }

		//toward up

            if(!left_toward_up(&point_current, map, path))
            {
                break;
            }

		}
	}
}

// void right_path(YAT_POINT point_start, Map *map, YAT_PATH *path)
// {
// 	//YAT_POINT point_start = world_to_map(map, startpoint);
// 	YAT_POINT point_current = point_start;
// 	YAT_POINT point_next;
// 	add_path(path,point_start);
// 	setUsed(map, point_start);
//     bool searching = true;
   
// 	while(searching)
//     {
// 		//toward up
// 		point_next.x = point_current.x;
// 		point_next.y = point_current.y + 1;

// 		if(isOutOfRange(map, point_next))
// 		{
// 			searching = false;
// 			break;
// 		}

// 		if(isPointInMap(map, point_next))
// 		{
// 			add_path(path,point_next);
// 			setUsed(map, point_next);
// 			point_current = point_next;
// 		}
// 		else
// 		{
// 			while(searching)
// 			{
// 				//toward right
// 				point_next.x = point_current.x + 1;
// 				point_next.y -= 1;	

// 				//out of map
// 				if(isOutOfRange(map, point_next))
// 				{
// 					searching = false;
// 					break;
// 				}

// 				if(isPointInMap(map, point_next))
// 				{
// 					add_path(path,point_next);
// 					setUsed(map, point_next);
// 					point_current = point_next;
// 					break;
//                 }
// 			}

// 			//toward down
//             bool searchingdown = true;
//             while(searchingdown && searching)
// 			{
// 				point_next.x = point_current.x;
// 				point_next.y = point_current.y - 1;

// 				//out of map
// 				if(isOutOfRange(map, point_next))
// 				{
// 					searching = false;
// 					break;
// 				}

// 				if(isPointInMap(map, point_next))
// 				{
// 					add_path(path,point_next);
// 					setUsed(map, point_next);
// 					point_current = point_next;
// 				}
// 				else
// 				{
//                     while(1)
//                     {
//                         //toward right
//                         point_next.x = point_current.x + 1;
//                         point_next.y += 1;

//                         //out of map
//                         if(isOutOfRange(map, point_next))
//                         {
//                             searching = false;
//                             break;
//                         }

//                         if(isPointInMap(map, point_next))
//                         {
//                             add_path(path,point_next);
//                             setUsed(map, point_next);
//                             point_current = point_next;
//                             searchingdown = false;
//                             break;
//                         }
//                     }
//                 }
// 			}
// 		}
// 	}
// }

YAT_POINT find_next_rigion_left(Map *map)
{
    YAT_POINT retpoint;
    retpoint.x = -1;
    retpoint.y = -1;
    for(int i=0; i<map->width - 1; i++)
    {
        for(int j=0; j<map->height; j++)
        {
            if(map->data[j * map->width + i] == 0 && map->data[j * map->width + i + 1] == 0)
            {
                retpoint.x = i;
                retpoint.y = j;
                return retpoint;
            }
        }
    }

    return retpoint;
}


//latitude and longitutde in degree
//returned: xyz in earth ,//in earth-centered frame
void get_XYZcoordinates(double latitude, double longititude,double xyz[])
{
    double a=6378137.0; //m
    double f=1/298.257223563;
    double N,e2;
    double PI=3.1415926;
    double h=3.7;//altitude in jia xing
    e2=f*(2-f);
    N=a/sqrt(1-e2*sin(latitude/180*PI)*sin(latitude/180*PI));
    xyz[0]=(N+h)*cos(latitude/180*PI)*cos(longititude/180*PI);
    xyz[1]=(N+h)*cos(latitude/180*PI)*sin(longititude/180*PI);
    xyz[2]=(N*(1-e2)+h)*sin(latitude/180*PI);
}


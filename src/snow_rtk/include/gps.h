#ifndef __GPS_H
#define __GPS_H	 	


#define		GPS_BUF_SIZE   160
#define		GPS_GPRMC   0
#define		GPS_GNRMC   1
#define		GPS_GPGGA   0
#define		GPS_GNGGA   1
#define		BESTXYZA    1
#define		REFSTATIONA 1
#define		HEADINGA 1


#define    UM220_INS_N  1   //和芯星通 GPS导航模块
//GPS NMEA-0183协议重要参数结体定义 

// #define  PI  3.1415926
#define  EARTH_RADIUS  6378.137  //地球近似半径

/*
每一纬度 长度 = 2 * 圆周率 * R(地球极半径）/ 360度
= 2 * 3.1415926 * 6356.9088  / 360
*/
#define  DIMEN_DU   110.9489   //  km 维度一度长度km
#define  DIMEN_MIN  DIMEN_DU/60     //  km 维度每一分长度km
#define  DIMEN_SEC  DIMEN_DU/3.6       //  m  维度每一分长度为m

/*
每一经度 长度 =  2 * 圆周率 * R(地球赤道半径）/ 360度 * cosB（当地纬度）
 = 2 * 3.1415926 * 6377.830 / 360 *cosB
*/

// #define  LONG_DU(longitude)   111.3141*cos(longitude)//kn 经度每一度长度
// #define  LONG_MIN			  LONG_DU(longitude)/60 // km 经度每一分长度
// #define  LONG_SEC             LONG_DU(longitude)/3.6 //m  经度每一秒长度
typedef struct  
{
 	double x;
	double y;
	double z;
}PointECEF;

typedef struct  
{
 	double latitude;
	double longitude;
	double altitude;
	double undulation;
}PointBLH; 

//UTC时间信息
typedef struct  
{										    
 	u16 year;	//年份
	u8 month;	//月份
	u8 date;	//日期
	u8 hour; 	//小时
	u8 min; 	//分钟
	double sec; 	//秒钟
}nmea_utc_time; 

//用户数据结构体
typedef struct
{
	double lat;		//纬度 用户给的维度参考点
	char ns;		//北纬/南纬,N:北纬;S:南纬				  
	double lon;		//经度 用户给的经度参考点 
	char ew;		//东经/西经,E:东经;W:西经
	float x;        //横坐标 返回给用户的
	float y;        //纵坐标 返回给用户的
	u8 state;       //数据状态
	u16 time_out;    //数据超时 
	float distance;  //两点之间的距离  
}Nmea_Typedef;
typedef enum
{
  NO_DATA=0,
  WRONG_DATA=1,
  GOOD_DATA=2,
  TIME_OUT=3,
  VALUABLE=4,
}GPS_State;	   
//NMEA 0183 协议解析后数据存放结构体
typedef struct  
{										    
	nmea_utc_time utc;	//UTC时间
	char status;        //定位状态  A有效定位  V 无效定位
	double latitude;		//纬度 
	char nshemi;				//北纬/南纬,N:北纬;S:南纬				  
	double longitude;		//经度 
	char ewhemi;			  //东经/西经,E:东经;W:西经			  
	double speed;       //地面速率 (000.0 - 999.9节，前面的0也将被传输)
	double direction;   //地面航向 (000.0~359.9度，以正北为参考基准,前面的0也将被传输)
	char mode;          //模式指示(仅NMEA0183 3.0版本输出, A=自主定位,D=差分,E=估算,N=数据无效)
	u8 check;           //校验

	char  type;
	u8 svNum;
	double hdop;
	double altitude;
	char altUnits;
	double undulation;
	char undUnits;
	double lostTime;
	char stnID;

	Nmea_Typedef user;  //用户自定义的坐标原点 
}GNGGA_msg;

typedef struct  
{										    
	nmea_utc_time utc;	//UTC时间
	char status;        //定位状态  A有效定位  V 无效定位
	double latitude;		//纬度 
	char nshemi;				//北纬/南纬,N:北纬;S:南纬				  
	double longitude;		//经度 
	char ewhemi;			  //东经/西经,E:东经;W:西经			  
	double speed;       //地面速率 (000.0 - 999.9节，前面的0也将被传输)
	double direction;   //地面航向 (000.0~359.9度，以正北为参考基准,前面的0也将被传输)
	char mode;          //模式指示(仅NMEA0183 3.0版本输出, A=自主定位,D=差分,E=估算,N=数据无效)
	u8 check;           //校验

	char  type;
	u8 svNum;
	double hdop;
	double altitude;
	char altUnits;
	double undulation;
	char undUnits;
	double lostTime;
	char stnID;

	Nmea_Typedef user;  //用户自定义的坐标原点 
}GNRMC_msg;

typedef struct  
{										    
	char statusPos;
	char typePos;
	PointECEF position;
	PointECEF stdPos;

	char statusVel;
	char typeVel;
	PointECEF velocity;
	PointECEF stdVel;

	char stnID;
	double diffAge;
	double solAge;
	int svNum;
}BESTXYZA_msg;

typedef struct  
{										    
	int status;
	PointECEF position;
	char health[4];
	char type[8];
	char stnID[8];
}REFSTATIONA_msg;

typedef struct  
{										    
	int status;
	double baseLine;
	double yaw;
	double pitch;
}HEADINGA_msg;

void NMEA_Str2num(u8 *buf,u8*dx,double * rs);
void NMEA_Str1num(u8 *buf,u8*dx,u32 *inte, u32 *deci);

double getdistance(double lat1,double lng1,double lat2,double lng2);
u8 NMEA_Comma_Pos(u8 *buf,u8 cx);
char *str( char *str1, char *str2);
u8 NMEA_Comma_Pos(u8 *buf,u8 cx);
char *str( char *str1, char *str2);
u8 GPS_Check(u8 *buf);	//异或校验和
u8 GPS_verify(u8 *buf); //寻找校验值
extern uint8_t      gnrmc_buf[GPS_BUF_SIZE];     //定义接收缓存
extern buf_type     gnrmc_buf_contr_op;
#endif
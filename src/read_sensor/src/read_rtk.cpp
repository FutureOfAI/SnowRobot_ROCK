// ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include <string>
#include <iostream>
#include <cstdio>

//c++
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>

#include "serial/serial.h"
#include <math.h>

#include "buf.h"

typedef uint8_t u8 ;
typedef uint16_t u16 ;
typedef uint32_t u32 ;

#define bufLength 512

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

#include  "navigationFunction.cpp"

//////////////////////////////////////////////////////data_type.h////////////////////////////////////
std::string getDate()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d-%H-%M-%S",localtime(&timep) );
    return tmp;
}

std::string point_path = "/home/rock/catkin_ws/src/read_sensor/data/";
// std::string point_path = "/home/yat/catkin_ws/src/rtk_zigzag/data/";
std::string file_gnss = point_path + getDate() + "gnssdata.txt";
std::ofstream fout_point(file_gnss.c_str());

/***********************************************GNss************************************************************/
/**-------------------------------------------------------
  * @函数名 __enter_critcal
  * @功能   屏蔽中断
  * @参数   
  * @返回值 无
***------------------------------------------------------*/
// __ASM  void __enter_critcal(void)
// {
// 	 CPSID I; PRIMASK=1	
// 	 bx lr
// }

/**-------------------------------------------------------
  * @函数名 __exit_critcal
  * @功能   使能中断
  * @参数   
  * @返回值 无
***------------------------------------------------------*/
// __ASM  void __exit_critcal(void)
// {
// 	 CPSIE I; PRIMASK=0	
// 	 bx lr
// }


/*
假定串口的数据传输比调度器速度要快，所以使用串口中断完成数据接收，为了是数据不丢失，给一个充分大的环形存储器作为串口的缓存			 
*/


/*******************************************************  
功能描述:初始化缓存
参数 _os :   需要初始化的缓存管理结构指针
     _Event: 作为缓存的数组
     buf_size:缓存数组大小
**********************************************************/
void  buf_init(buf_type * _os,uint8_t  *_Event,uint8_t buf_size)  //
{    
      
      uint8_t i;
      for(i=0;i<buf_size;i++)
      {
	      _Event[i]=0;    
      }
     	
      _os->Start=  _Event;                       
      _os->QEnd=   _Event+buf_size;      
      _os->OSQIn=  _Event;            
      _os->OSQOut= _Event;            
      _os->OSQSize= buf_size;          
      _os->OSQEntries=0;              
	
}

/*
计算当前缓存中存储的数据总量，考虑到数据覆盖
返回值:当前存储的数据字节数
*/
static uint8_t  buf_rxnum(buf_type *p)
{
	if (p->OSQOut > p->OSQIn){ 
		return   (p->QEnd - p->OSQOut) + (p->OSQIn - p->Start) ;
	} 
	if (p->OSQOut < p->OSQIn){
  		return  p->OSQIn - p->OSQOut ;
	}
	return 0 ; 	 
}


/*
    读缓存一个字节
    返回值oK及成功，返回值EMPTY没有数据可读
*/
uint8_t   buf_read(buf_type * _os,uint8_t *p) 
{     
	if ( buf_rxnum(_os) > 0){  //有数据允许读
		*p = *((_os->OSQOut)++); 
		if (_os->OSQOut==_os->QEnd){ 
			_os->OSQOut = _os->Start ; 
			
		}
		return OK;
	}
	return EMPTY;
}


/*
   写入缓存一个字节，没有条件的写，因可覆盖
*/
void  buf_write(buf_type * _os,uint8_t *p)
{  
	if (_os->OSQIn ==_os->QEnd)  
		_os->OSQIn = _os->Start;
	*((_os->OSQIn)++)= *p ;
	if(_os->OSQIn==_os->QEnd) 
		_os->OSQIn=_os->Start; 
 }
	   
#include <stdio.h>	 
#include "stdarg.h"	 
#include "string.h"	 

#include "gps.h" 

GNRMC_msg		dataRMC;
GNGGA_msg		dataGGA;
BESTXYZA_msg 	bestXYZ;
REFSTATIONA_msg stationXYZ;
HEADINGA_msg    dataHEAD;

uint8_t      gnss_buf[GPS_BUF_SIZE];     //定义接收缓存
buf_type     gnss_buf_contr_op;

//从buf里面得到第cx个逗号所在的位置
//返回值:0~0XFE,代表逗号所在位置的偏移.
//       0XFF,代表不存在第cx个逗号							  
u8 NMEA_Comma_Pos(u8 *buf,u8 cx)
{	 		    
	u8 *p=buf;
	for(int i=bufLength;cx>0 && i>0;i--)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;
}

int findSemicolon(u8 *buf,u8 cx)
{
	u8 *p=buf;
	for(int i=bufLength;cx>0 && i>0;i--)
	{		 
		if(*buf=='*')return -1;//遇到'*'或者非法字符,则不存在第cx个逗号
		if(*buf==';')cx--;
		buf++;
	}
	return buf-p;
}

//经度数据长度过长,整数和小数部分单独返回
/*
buf:数字存储区
dx:小数点位数,返回给调用函数
inte 整数部分
deci 小数部分
*/
double NMEAstr2Double(u8 *buf)
{
	u8 mask=0;
	double npFlag = 1;

	double result = 0;
	double index = 1;

	for(int i = 0; i < 64; i++) //得到整数和小数的长度  两个逗号之间的数最长为24位( int is 10 bit, float is 14 bit)
	{
		// if(buf[i]==','||(buf[i]=='*')) break;//遇到结束了

		if(buf[i] =='.')mask|=0X01;//遇到小数点了
		else if(buf[i] == '-') npFlag = -1;
		else if(buf[i]<='9'&& (buf[i]>='0'))
		{	
			int data = buf[i] - '0';
			if(mask&0X01)
			{
				index = index*0.1;
				result = result + data*index;
			}
			else
			{
				result = result*10 +data;
			}
		}
		else
		{
			break;
		}	
	}

	return npFlag * result;
}

//字符串查找函数
bool compareString(u8 *buf,char targetString[])
{
	int stringLength = strlen(targetString);
	int sameNum = stringLength;

	for(int i = 0; i < stringLength; i++)
	{
		if(buf[i] != targetString[i])
		{
			sameNum = i;
		}
	}

   if(sameNum == stringLength) return true;
   else return false;
}

//分析GNss信息
//gnssData:nmea信息结构体
//buf:接收到的GPS/北斗数据缓冲区首地址
void NMEA_GNGGA_Analysis(GNGGA_msg *gnssData,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;     
	u32 temp,temp1;	   
	double rs;

	p1 = buf;

	if(p1 == 0) return;

	for(int i = 0;i<bufLength;i++)
	{
		// ROS_INFO("the NO.%d data is %c ",i,buf[i]);
	}

	posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
	if(posx!=0XFF)
	{
		rs = NMEAstr2Double(p1+posx);
		temp = (int)rs;
		gnssData->utc.hour=temp/10000;
		gnssData->utc.min = (temp/100)%100;
		gnssData->utc.sec = rs - gnssData->utc.hour*10000 - gnssData->utc.min*100;	 	 

	}	
	posx=NMEA_Comma_Pos(p1,2);	//获取纬度
	if(posx!=0XFF)
	{
		double latitudeData = NMEAstr2Double(p1+posx);		 	 		
		temp=(int)(latitudeData/100);	//得到°
		double floatRes = (latitudeData - temp*100)/60;				//得到' 
		gnssData->latitude = temp + floatRes;//转换为° 
	}
	posx=NMEA_Comma_Pos(p1,3);								//南纬还是北纬 
	if(posx!=0XFF)gnssData->nshemi=*(p1+posx);					

	posx=NMEA_Comma_Pos(p1,4);								//获取经度
	if(posx!=0XFF)
	{											  
		double longitudeData = NMEAstr2Double(p1+posx);		 	 		
		temp=(int)(longitudeData/100);	//得到°
		double floatRes = (longitudeData - temp*100)/60;				// get ' , convert to °		 
		gnssData->longitude=temp + floatRes;//转换为° 
	}
	posx=NMEA_Comma_Pos(p1,5);								//东经还是西经
	if(posx!=0XFF)gnssData->ewhemi=*(p1+posx);		 
	
	posx=NMEA_Comma_Pos(p1,6);			  //获取定位类型 	
	if(posx!=0XFF)gnssData->type=*(p1+posx);					
	
	posx=NMEA_Comma_Pos(p1,7);	  //获取用于定位卫星数量 
	if(posx!=0XFF)gnssData->svNum=*(p1+posx);								//获取
	if(posx!=0XFF)
	{
		gnssData->svNum = (int)(NMEAstr2Double(p1+posx));
	} 

	posx=NMEA_Comma_Pos(p1,8);								//获取
	if(posx!=0XFF)
	{		 				 
		gnssData->hdop = NMEAstr2Double(p1+posx);
	} 

	posx=NMEA_Comma_Pos(p1,9);	 //获取天线高度
	if(posx!=0XFF)
	{		 				 
		gnssData->altitude = NMEAstr2Double(p1+posx);
	} 

	posx=NMEA_Comma_Pos(p1,10);	  //获取高度单位
	if(posx!=0XFF)gnssData->altUnits=*(p1+posx);

	posx=NMEA_Comma_Pos(p1,11);	 //获取天线高度
	if(posx!=0XFF)
	{		 				 
		gnssData->undulation = NMEAstr2Double(p1+posx);
	} 

	posx=NMEA_Comma_Pos(p1,12);	  //获取高度单位
	if(posx!=0XFF)gnssData->undUnits=*(p1+posx);

	posx=NMEA_Comma_Pos(p1,13);	 //获取 差分数据丢失时间
	if(posx!=0XFF)
	{	 				 
		gnssData->lostTime = NMEAstr2Double(p1+posx);
	} 

	// posx=NMEA_Comma_Pos(p1,14);	 //获取基站编号
	// if(posx!=0XFF)
	// {		 				 
	// 	gnssData->stnID = 0;	//得到
	// } 

	for(int i=0;i < bufLength;i++)
    {
		posx = 0XFF;
	    if(*(p1+i)=='*')
		{
			posx = i;
			break;  //如果碰到 * 就退出，表明校验已经完成,最长校验字符长度不能超过 bufLength 个
		}
    }

	if(posx!=0XFF)
	{
		//  gnssData->mode=*(p1+posx);
		temp=(*(p1+posx+1)>='A'?(*(p1+posx+2)-55):(*(p1+posx+2)-'0'))*16 ; //校验值
		temp1=*(p1+posx+2)>='A'?(*(p1+posx+3)-55):(*(p1+posx+3)-'0') ; //校验值
		gnssData->check=temp+temp1;
	}
}

//分析GNss信息
//gnssData:nmea信息结构体
//buf:接收到的GPS/北斗数据缓冲区首地址
void NMEA_GNRMC_Analysis(GNRMC_msg *gnssData,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;     
	u32 temp,temp1;	   
	double rs;

	p1 = buf;

	for(int i = 0;i<75;i++)
	{
		// ROS_INFO("the NO.%d data is %c ",i,buf[i]);
	}

	posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
	if(posx!=0XFF)
	{
		rs = NMEAstr2Double(p1+posx);
		temp = (int)rs;
		gnssData->utc.hour = temp/10000;
		gnssData->utc.min = (temp/100)%100;	
		gnssData->utc.sec = rs - gnssData->utc.hour *10000 - gnssData->utc.min*100; 	 
	}

	posx=NMEA_Comma_Pos(p1,2);			//A=有效定位，V=无效定位
	if(posx!=0XFF)gnssData->status=*(p1+posx);	

	posx=NMEA_Comma_Pos(p1,3);								//获取纬度
	if(posx!=0XFF)
	{
		double latitudeData = NMEAstr2Double(p1+posx);
		temp=(int)(latitudeData/100);	//得到°
		double floatRes = (latitudeData - temp*100)/60;	 
		gnssData->latitude = temp + floatRes;//转换为° 
	}
	
	posx=NMEA_Comma_Pos(p1,4);								//南纬还是北纬 
	if(posx!=0XFF)gnssData->nshemi=*(p1+posx);		

	posx=NMEA_Comma_Pos(p1,5);								//获取经度
	if(posx!=0XFF)
	{											  	 	 		
		double longitudeData = NMEAstr2Double(p1+posx);
		temp=(int)(longitudeData/100);	//得到°
		double floatRes = (longitudeData - temp*100)/60;				// get ' , convert to °			 
		gnssData->longitude = temp + floatRes;//转换为° 
	}

	posx=NMEA_Comma_Pos(p1,6);								//东经还是西经
	if(posx!=0XFF)gnssData->ewhemi=*(p1+posx);		 

	posx=NMEA_Comma_Pos(p1,7);			  //获取地面速率 					
	if(posx!=0XFF)
	{
		gnssData->speed = NMEAstr2Double(p1+posx);	//获取地面速率
	}	
	posx=NMEA_Comma_Pos(p1,8);								//获取地面航向
	if(posx!=0XFF)
	{												  		 	 
		gnssData->direction=NMEAstr2Double(p1+posx);	//得到°
	}
	posx=NMEA_Comma_Pos(p1,9);								//获取UTC日期
	if(posx!=0XFF)
	{
		temp=(int)(NMEAstr2Double(p1+posx));			
		gnssData->utc.date=temp/10000;
		gnssData->utc.month=(temp/100)%100;
		gnssData->utc.year=(2000+temp%100);	 	 
	} 
	posx=NMEA_Comma_Pos(p1,12);								//模式指示
	if(posx!=0XFF)
	{
	 gnssData->mode=*(p1+posx);
	 temp=(*(p1+posx+2)>='A'?(*(p1+posx+2)-55):(*(p1+posx+2)-'0'))*16 ; //校验值
	 temp1=*(p1+posx+3)>='A'?(*(p1+posx+3)-55):(*(p1+posx+3)-'0') ; //校验值
	 gnssData->check=temp+temp1;
	}
}

//分析GNSS信息
//gnssData:nmea信息结构体
//buf:接收到的数据缓冲区首地址
void NMEA_BESTXYZA_Analysis(BESTXYZA_msg *gnssData,u8 *buf)
{
	u8 *p1,dx;			 
	int posx, startNum = findSemicolon(buf,1); // find the first ; and return the position for the symbol
	if(startNum < 0) return;

	p1 = buf + startNum;

	for(int i = 0; i < bufLength; i++)
	{
		// ROS_INFO("the NO.%d data is %c ",i,buf[i]);
	}

	posx=NMEA_Comma_Pos(p1,2);
	if(posx!=0XFF)
	{
		gnssData->position.x = NMEAstr2Double(p1+posx);  // the velocity of X-axis
	}

	posx=NMEA_Comma_Pos(p1,3);
	if(posx!=0XFF)
	{
		gnssData->position.y = NMEAstr2Double(p1+posx);  // the velocity of Y-axis
	}

	posx=NMEA_Comma_Pos(p1,4);
	if(posx!=0XFF)
	{
		gnssData->position.z = NMEAstr2Double(p1+posx);  // the velocity of Z-axis
	}

	posx=NMEA_Comma_Pos(p1,5);
	if(posx!=0XFF)
	{
		gnssData->stdPos.x = NMEAstr2Double(p1+posx);  // the position standard deviation of X-axis
	}

	posx=NMEA_Comma_Pos(p1,6);
	if(posx!=0XFF)
	{
		gnssData->stdPos.y = NMEAstr2Double(p1+posx);  // the position standard deviation of Y-axis
	}

	posx=NMEA_Comma_Pos(p1,7);
	if(posx!=0XFF)
	{
		gnssData->stdPos.z = NMEAstr2Double(p1+posx);  // the position standard deviation of Z-axis
	}

	posx=NMEA_Comma_Pos(p1,10);
	if(posx!=0XFF)
	{
		gnssData->velocity.x = NMEAstr2Double(p1+posx); // the velocity of X-axis
	}

	posx=NMEA_Comma_Pos(p1,11);
	if(posx!=0XFF)
	{
		gnssData->velocity.y = NMEAstr2Double(p1+posx); // the velocity of Y-axis
	}

	posx=NMEA_Comma_Pos(p1,12);
	if(posx!=0XFF)
	{
		gnssData->velocity.z = NMEAstr2Double(p1+posx); // the velocity of Z-axis
	}

	posx=NMEA_Comma_Pos(p1,13);
	if(posx!=0XFF)
	{
		gnssData->stdVel.x = NMEAstr2Double(p1+posx); // the velocity standard deviation of X-axis
	}

	posx=NMEA_Comma_Pos(p1,14);
	if(posx!=0XFF)
	{
		gnssData->stdVel.y = NMEAstr2Double(p1+posx); // the velocity standard deviation of Y-axis
	}

	posx=NMEA_Comma_Pos(p1,15);
	if(posx!=0XFF)
	{
		gnssData->stdVel.z = NMEAstr2Double(p1+posx); // the velocity standard deviation of Z-axis
	}

	posx=NMEA_Comma_Pos(p1,18);
	if(posx!=0XFF)
	{
		gnssData->diffAge = NMEAstr2Double(p1+posx); // rtcm data age
	}
	
	posx=NMEA_Comma_Pos(p1,19);
	if(posx!=0XFF)
	{
		gnssData->solAge = NMEAstr2Double(p1+posx); // solution age
	}
	
	posx=NMEA_Comma_Pos(p1,20);
	if(posx!=0XFF)
	{
		gnssData->svNum = (int)(NMEAstr2Double(p1+posx)); // the number of SV
	}
}

void NMEA_HEADINGA_Analysis(HEADINGA_msg *gnssData,u8 *buf)
{
	u8 *p1,dx;			 
	int posx,startNum = findSemicolon(buf,1); // find the first ; and return the position for the symbol
	if(startNum < 0) return;

	p1 = buf + startNum;

	for(int i = startNum; i < bufLength; i++)
	{
		if(buf[i] == '*') break;
		// ROS_INFO("the NO.%d data is %c ",i,buf[i]);
	}

	char intString[] = "NARROW_INT";
	char floatString[] = "NARROW_FLOAT";
	char noneString[] = "NONE";

	posx=NMEA_Comma_Pos(p1,1);
	if(posx!=0XFF)
	{
		// for(int i = posx; p1[i] != ',';i++ )
		// {
		// 	ROS_INFO("station x data is %c",p1[i]);
		// }

		if(compareString(p1+posx,intString))
		{
			gnssData->status = 3;
		}
		else if(compareString(p1+posx,floatString))
		{
			gnssData->status = 2;
		}
		else if(compareString(p1+posx,noneString))
		{
			gnssData->status = 1;
		}
		else
		{
			gnssData->status = 0;
		}
		// ROS_INFO("the station status is %d ",gnssData->status);
	}

	posx=NMEA_Comma_Pos(p1,2);
	if(posx!=0XFF)
	{
		gnssData->baseLine = NMEAstr2Double(p1+posx);  // base line message measure from rtk
	}

	posx=NMEA_Comma_Pos(p1,3);
	if(posx!=0XFF)
	{
		gnssData->yaw = NMEAstr2Double(p1+posx);  // yaw message measure from rtk
	}

	posx=NMEA_Comma_Pos(p1,4);
	if(posx!=0XFF)
	{
		gnssData->pitch = NMEAstr2Double(p1+posx);  // pitch message measure from rtk
	}
}

void NMEA_REFSTATIONA_Analysis(REFSTATIONA_msg *gnssData,u8 *buf)
{
	u8 *p1,dx;			 
	int posx,startNum = findSemicolon(buf,1); // find the first ; and return the position for the symbol
	if(startNum < 0) return;

	p1 = buf + startNum;

	for(int i = startNum; i < bufLength; i++)
	{
		if(buf[i] == '*') break;
		// ROS_INFO("the NO.%d data is %c ",i,buf[i]);
	}

	gnssData->status = (int)(NMEAstr2Double(p1));

	// ROS_INFO("the station status is %d ",gnssData->status);
	
	posx=NMEA_Comma_Pos(p1,1);
	if(posx!=0XFF)
	{
		// for(int i = posx; p1[i] != ',';i++ )
		// {
		// 	ROS_INFO("station x data is %c",p1[i]);
		// }
		gnssData->position.x = NMEAstr2Double(p1+posx);  // the velocity of X-axis
	}

	posx=NMEA_Comma_Pos(p1,2);
	if(posx!=0XFF)
	{
		gnssData->position.y = NMEAstr2Double(p1+posx);  // the velocity of Y-axis
	}

	posx=NMEA_Comma_Pos(p1,3);
	if(posx!=0XFF)
	{
		gnssData->position.z = NMEAstr2Double(p1+posx);  // the velocity of Z-axis
	}
}


u8 GPS_Check(u8 *buf)	//异或校验和
{
    u8 checksum=0;
    int i=0;

    for(i=0;i<bufLength;i++)
    {
	    if(buf[i]=='*') break;  //如果碰到 * 就退出，表明校验已经完成,最长校验字符长度不能超过 bufLength 个
        else checksum ^=buf[i];
    }
	if(i<bufLength) return checksum; //
	else  return 0;   //校验失败 没有找到*
}

u8 GPS_verify(u8 *buf) //寻找校验值
{	 
	int posx = -1;     
	u8 temp = 0,temp1 = 0;

	for(int i=0;i < bufLength;i++)
    {
	    if(buf[i]=='*')
		{
			posx = i;
			break;  //如果碰到 * 就退出，表明校验已经完成,最长校验字符长度不能超过 bufLength 个
		}
    }

	if(posx != -1)
	{
		temp=((buf[posx+1])>='A'?(buf[posx+1]-55):(buf[posx+1]-'0'))*16; //校验值
		temp1=(buf[posx+2])>='A'?(buf[posx+2]-55):(buf[posx+2]-'0'); //校验值
		temp = temp + temp1;
	} 

	return temp;
}

/*******************************************串口接收数据处理*****************************************/

uint8_t buff[bufLength] = {EMPTY,};//第一个字节存放附加信息，说明读取信息的有效性，二级存储器
uint8_t * read_gnss_buf(void)
{
	uint8_t data,temp,temp1;
	static int index=0;
	static uint8_t count=0;
	static  uint8_t  flag=0,cout=0;
	static bool findHeader = false;

    if ((*buff) == EMPTY )
	{  //说明当前还没有从缓存中取数据或者取得的数据不完整,需要继续

    	if ((*(buff+1) != '$')&&(*(buff+1) != '#'))
		{     //还没有找到 报头，搜索报头
	    	for (int i=0;i<GPS_BUF_SIZE;i++)
				{  
					temp = buf_read(& gnss_buf_contr_op,&data);
					if (temp == EMPTY )     //如果搜完缓存中的所有存储的信息都没有找到报头，本次搜索失败。
						return  buff;  
					if(data == '$' || data == '#') 
					{
						index = bufLength;
						*(buff+1) = data;        //读出的报头写入到二级缓存
						break;                //跳出寻找报头，继续循找剩下的数据
					}
	    	}
		}	

		for (;index>0;)
		{             //读取余下的字节
			temp= buf_read(& gnss_buf_contr_op,&data);
			if(temp == EMPTY )        //当前缓存中没有可读数据了
			{
				return  buff; 
			}
			else
			{
				count++;
				*(buff+ 1 + count) = data;
				if(buff[1 + count] == '*') flag=1;  //寻找 *
			}

      		if(flag == 1)  //寻找 *
			{
				cout++;
				if(cout==3)
				{
					cout=0; 
					flag=0;
					count=0;
					index=0;
					if(buff[1] == '$')
					{
						temp = GPS_Check(buff+2);  //计算校验值 从$符号往后计算
						temp1 = GPS_verify(buff+1);  //获取校验码
					}
					else
					{
						temp1 = 1;
						temp = 1; // headle # is not checked
					}

					if(temp == temp1)  //校验码正确
					{ 
						*buff = FULL;           //二级存储器头部信息
						return  buff; 
					}

					for(int i = 0;i<bufLength;i++) buff[i] = 0; //校验通不过,重新来过
						        
					return  buff; 
				}
			}
		}
  }

}

void analyseNMEA(int &lineID)
{
	uint8_t *buf;
	buf = read_gnss_buf();
	if(*buf==FULL)  //收到数据
	{
		char *stringGN,*stringGP;
		char gpggaString[] = "GPGGA";
		char gnggaString[] = "GNGGA";
		char gprmcString[] = "GPRMC";
		char gnrmcString[] = "GNRMC";
		char stationString[] = "REFSTATIONA";
		char bestPosString[] = "BESTXYZA";
		char headingString[] = "HEADINGA";

		#if GPS_GNGGA || GPS_GPGGA
			stringGN = gnggaString;
			stringGP = gpggaString;
			if(compareString(buf + 2,stringGN)||compareString(buf + 2,stringGP)) 
			{
				// ROS_INFO("start read the GNGGA data.");
				NMEA_GNGGA_Analysis(&dataGGA,buf+1); 
				lineID = 1;
			}
		#endif

		#if GPS_GPRMC || GPS_GNRMC
			stringGN = gnrmcString;
			stringGP = gprmcString;
			if(compareString(buf + 2,stringGN)||compareString(buf + 2,stringGP)) 
			{
				// ROS_INFO("start read the GNRMC data.");
				NMEA_GNRMC_Analysis(&dataRMC,buf+1); 
				lineID = 2;
			}
		#endif

		#if BESTXYZA
			stringGN = bestPosString;
			// ROS_INFO("the best head is : %c,%c,%c,%c,%c,%c,%c,%c,%c",buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9]);
			if(compareString(buf + 2,stringGN))
			{
				// ROS_INFO("start read the BESTXYZA data.");
				// NMEA_BESTXYZA_Analysis(&bestXYZ,buf+1); 
				// lineID = 3;
				// ROS_INFO("Best Position in ECEF is : x = %f, y = %f, z = %f.",bestXYZ.position.x,bestXYZ.position.y,bestXYZ.position.z);
				// ROS_INFO("Position standard deviation is : x = %f, y = %f, z = %f.",bestXYZ.stdPos.x,bestXYZ.stdPos.y,bestXYZ.stdPos.z);
				// ROS_INFO("Best Velocity  std in ECEF is : x = %f, y = %f, z = %f.",bestXYZ.velocity.x,bestXYZ.velocity.y,bestXYZ.velocity.z);
				// ROS_INFO("Velocity standard deviation is : x = %f, y = %f, z = %f.",bestXYZ.stdVel.x,bestXYZ.stdVel.y,bestXYZ.stdVel.z);
			}
		#endif

		#if REFSTATIONA
			stringGN = stationString;
			if(compareString(buf + 2,stringGN))
			{
				// ROS_INFO("start read the REFSTATIONA data.");
				NMEA_REFSTATIONA_Analysis(&stationXYZ,buf+1); 
				lineID = 4;
			}
		#endif

		#if HEADINGA
			stringGN = headingString;
			if(compareString(buf + 2,stringGN))
			{
				// ROS_INFO("start read the REFSTATIONA data.");
				NMEA_HEADINGA_Analysis(&dataHEAD,buf+1); 
				lineID = 5;
			}
		#endif

		// if(*(buf+2)=='G') NMEA_GNGGA_Analysis(&gnss,buf+1); //开始数据解析

		for(int i=0;i<bufLength;i++)  *(buf+i)=0;
	}
}


ros::Publisher gnss_pub;
ros::Publisher state_pub;
ros::Publisher ecef_pub;
ros::Publisher station_pub;
ros::Publisher heading_pub;

//由经纬度获取建立平面坐标系
/*
x: 坐标系x轴
y: 坐标系y轴
*/
void gnssDataPublish()
{
	int lineID = 0; // 0 is NULL; 1 is "GPGGA" or "GNGGA"; 2 is "GPRMC" or "GNRMC";
	analyseNMEA(lineID);

	if(lineID == 0) return;

	/*******************************************************************************/

	geometry_msgs::PoseStamped positionMessage;

	positionMessage.header.stamp = ros::Time::now();
	positionMessage.header.frame_id = "position";
	positionMessage.pose.position.x = dataGGA.type;
	positionMessage.pose.position.y = dataGGA.svNum;
	positionMessage.pose.position.z = dataGGA.lostTime;

	if(dataGGA.nshemi == 'N') positionMessage.pose.orientation.x = dataGGA.latitude;
	else if(dataGGA.nshemi == 'S') positionMessage.pose.orientation.x = - dataGGA.latitude;

	if(dataGGA.ewhemi == 'E') positionMessage.pose.orientation.y = dataGGA.longitude;
	else if(dataGGA.ewhemi == 'W') positionMessage.pose.orientation.y = - dataGGA.longitude;
	
	positionMessage.pose.orientation.z = dataGGA.altitude;
	positionMessage.pose.orientation.w = dataGGA.undulation;
	
	geometry_msgs::PoseStamped stateMessage;

	stateMessage.header.stamp = ros::Time::now();
	stateMessage.header.frame_id = "state";
	stateMessage.pose.position.x = dataRMC.utc.hour;
	stateMessage.pose.position.y = dataRMC.utc.min;
	stateMessage.pose.position.z = dataRMC.utc.sec;
	
	stateMessage.pose.orientation.x = dataGGA.hdop;
	stateMessage.pose.orientation.y = dataRMC.status;
	stateMessage.pose.orientation.z = dataRMC.direction;
	stateMessage.pose.orientation.w = dataRMC.speed;

/********************************************************************************************/
    // POINT_XYZ pointBLH;
    // pointBLH.x = positionMessage.pose.orientation.x * PI/180;
    // pointBLH.y = positionMessage.pose.orientation.y * PI/180;
    // pointBLH.z = positionMessage.pose.orientation.z + positionMessage.pose.orientation.w;

    // POINT_XYZ pointECEF = BLHtoECEF(pointBLH);
	// bestXYZ.position.x = pointECEF.x;
	// bestXYZ.position.y = pointECEF.y;
	// bestXYZ.position.z = pointECEF.z;

	// bestXYZ.stdPos.x = 0.0124;
	// bestXYZ.stdPos.y = 0.0134;
	// bestXYZ.stdPos.z = 0.0102;
	// bestXYZ.diffAge = 0;

	// bestXYZ.velocity.x = 0.514444 * dataRMC.speed * cos(PI/2 - dataRMC.direction * PI/180);
	// bestXYZ.velocity.y = 0.514444 * dataRMC.speed * sin(PI/2 - dataRMC.direction * PI/180);;
	// bestXYZ.velocity.z = 0.0;
	
	// bestXYZ.stdVel.x = 0.1439;
	// bestXYZ.stdVel.y = 0.1802;
	// bestXYZ.stdVel.z = 0.1297;
	// bestXYZ.solAge = 0;

/********************************************************************************************/

	geometry_msgs::PoseArray PosVelECEF;
	PosVelECEF.poses.resize(2);
	PosVelECEF.header.stamp = ros::Time::now();
	PosVelECEF.header.frame_id = "PosVelECEF";
	PosVelECEF.poses[0].position.x = bestXYZ.position.x;
	PosVelECEF.poses[0].position.y = bestXYZ.position.y;
	PosVelECEF.poses[0].position.z = bestXYZ.position.z;
	
	PosVelECEF.poses[0].orientation.x = bestXYZ.stdPos.x;
	PosVelECEF.poses[0].orientation.y = bestXYZ.stdPos.y;
	PosVelECEF.poses[0].orientation.z = bestXYZ.stdPos.z;
	PosVelECEF.poses[0].orientation.w = bestXYZ.diffAge;

	PosVelECEF.poses[1].position.x = bestXYZ.velocity.x;
	PosVelECEF.poses[1].position.y = bestXYZ.velocity.y;
	PosVelECEF.poses[1].position.z = bestXYZ.velocity.z;
	
	PosVelECEF.poses[1].orientation.x = bestXYZ.stdVel.x;
	PosVelECEF.poses[1].orientation.y = bestXYZ.stdVel.y;
	PosVelECEF.poses[1].orientation.z = bestXYZ.stdVel.z;
	PosVelECEF.poses[1].orientation.w = bestXYZ.solAge;

	geometry_msgs::PointStamped stationMessage;
	
	stationMessage.header.stamp = ros::Time::now();
	stationMessage.header.frame_id = "refstation";
	stationMessage.point.x = stationXYZ.position.x;
	stationMessage.point.y = stationXYZ.position.y;
	stationMessage.point.z = stationXYZ.position.z;

	geometry_msgs::PointStamped headingMessage;
	
	headingMessage.header.stamp = ros::Time::now();
	headingMessage.header.frame_id = "heading";
	headingMessage.point.x = dataHEAD.status;
	headingMessage.point.y = dataHEAD.baseLine;
	headingMessage.point.z = dataHEAD.yaw;

	switch (lineID)
	{
		case 1:
			gnss_pub.publish(positionMessage);
			// ROS_INFO("UTC time is :hour = %d , min = %d , sec = %f .",dataGGA.utc.hour,dataGGA.utc.min,dataGGA.utc.sec);	 
			// ROS_INFO("gps location is : latitude = %f %c, longitude = %f %c.",dataGGA.latitude,dataGGA.nshemi,dataGGA.longitude,dataGGA.ewhemi);
			// ROS_INFO("location type is %c , SV num = %d , HDOP = %f .",dataGGA.type,dataGGA.svNum,dataGGA.hdop);
			// ROS_INFO("altitude = %f %c ,undulation = %f %c.",dataGGA.altitude,dataGGA.altUnits,dataGGA.undulation,dataGGA.undUnits);
			// ROS_INFO("rtcm lost time = %f s, base station ID is %c.",dataGGA.lostTime,dataGGA.stnID);
			break;
		case 2:
			state_pub.publish(stateMessage);
			// ROS_INFO("location status is %c , direction = %f , speed = %f .",dataRMC.status,dataRMC.direction,dataRMC.speed);
			break;
		case 3:
			ecef_pub.publish(PosVelECEF);
			// ROS_INFO("get the BESTXYZA data.");
			// ROS_INFO("Best Position in ECEF is : x = %f, y = %f, z = %f.",bestXYZ.position.x,bestXYZ.position.y,bestXYZ.position.z);
			// ROS_INFO("Position standard deviation is : x = %f, y = %f, z = %f.",bestXYZ.stdPos.x,bestXYZ.stdPos.y,bestXYZ.stdPos.z);
			// ROS_INFO("Best Velocity  std in ECEF is : x = %f, y = %f, z = %f.",bestXYZ.velocity.x,bestXYZ.velocity.y,bestXYZ.velocity.z);
			// ROS_INFO("Velocity standard deviation is : x = %f, y = %f, z = %f.",bestXYZ.stdVel.x,bestXYZ.stdVel.y,bestXYZ.stdVel.z);
			// ROS_INFO("diffAge = %f, solAge = %f , svNum =%d",bestXYZ.diffAge,bestXYZ.solAge,bestXYZ.svNum);
			break;
		case 4:
			if(stationXYZ.status == 0)
			{
				station_pub.publish(stationMessage);
			}
			// ROS_INFO("station XYZ in ECEF is : status = %d, x = %f, y = %f, z = %f.",stationXYZ.status,stationXYZ.position.x,stationXYZ.position.y,stationXYZ.position.z);
			break;
		case 5:
			heading_pub.publish(headingMessage);
			// ROS_INFO("station XYZ in ECEF is : status = %d, x = %f, y = %f, z = %f.",stationXYZ.status,stationXYZ.position.x,stationXYZ.position.y,stationXYZ.position.z);
			break;
		default:
			// ROS_INFO("get others NMEA data, but no analyze this data.");
			break;
	}
}

int main(int argc, char **argv)
{

  	ros::init(argc, argv, "gps_reader");
	ros::NodeHandle nh;

    buf_init( &gnss_buf_contr_op,gnss_buf,GPS_BUF_SIZE);

	gnss_pub = nh.advertise<geometry_msgs::PoseStamped>("/gnss_position", 10);
	state_pub =  nh.advertise<geometry_msgs::PoseStamped>("/gnss_state", 10);
	ecef_pub = nh.advertise<geometry_msgs::PoseArray>("/ecef_PosVel", 10);
	station_pub = nh.advertise<geometry_msgs::PointStamped>("/ecef_station", 10);
	heading_pub = nh.advertise<geometry_msgs::PointStamped>("/rtk_yaw", 10);

    static ros::Time current_time;

	string port("/dev/rtk");
	// string port("/dev/ttyUSB0"); // read the NMEA data, write the RTCM data 

	// string readPort("/dev/rtcmr");
	// string readPort("/dev/ttyUSB1"); //read the RTCM date 

	// string sendPort("/dev/rtcms");
	// string readPort("/dev/ttyUSB2"); //read the RTCM date 

	// Argument 2 is the baudrate
	unsigned long baud = 115200;

	ROS_INFO("openning serial");

	// port, baudrate, timeout in milliseconds
	serial::Serial gnss_serial(port, baud, serial::Timeout::simpleTimeout(1));
	if(gnss_serial.isOpen())
		ROS_INFO( "gnss serial open success");
	else
		ROS_ERROR("gnss serial open failed");

	// serial::Serial read_serial(readPort, baud, serial::Timeout::simpleTimeout(1));
	// if(read_serial.isOpen())
	// 	ROS_INFO( "read serial open success");
	// else
	// 	ROS_ERROR("read serial open failed");

	// cout <<"starting to publish gps topic...\n";

	// serial::Serial send_serial(sendPort, baud, serial::Timeout::simpleTimeout(1));
	// if(send_serial.isOpen())
	// 	ROS_INFO( "read serial open success");
	// else
	// 	ROS_ERROR("read serial open failed");

	// cout <<"starting to publish gps topic...\n";

	static int counter=0;
	string result;
	string rtcmData;

    uint8_t data = 0;

    //ros::Rate loop_rate(10000);
	while(ros::ok())
    {
		// rtcmData = read_serial.read(1);
		// if(rtcmData.length())
		// {
		// 	send_serial.write(rtcmData);
		// 	// ROS_INFO("gnss message get the rtcm data is %d",rtcmData.at(0));
		// }
		
		result = gnss_serial.read(1);
		if (result.length()) 
		{
			data = result.at(0);
			if((data != 13)&&(data != 10))
			{
				buf_write(&gnss_buf_contr_op,&data);
				char msgData = data;
				// ROS_INFO("gnss message send the data is %c",data);
				if((data == '$')||(data == '#'))
				{
					fout_point << std::endl;
				}
				fout_point << msgData;
			}
		}

        // ros::spinOnce();
        gnssDataPublish();
    }

	fout_point.close();
    return 0;
}

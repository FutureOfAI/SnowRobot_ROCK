// ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include <string>
#include <iostream>
#include <cstdio>

#include "serial/serial.h"
#include <math.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>

// #define PI 3.1415926535897932384626433

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

//////////////////////////////////////////////////////data_type.h////////////////////////////////////

uint16_t alf001_pub_period;  

/* exact-width integer types */
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;

/* exact-width unsigned integer types */
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;

#define IMU_BUF_SIZE 75

typedef  struct 
{               
	uint8_t       *Start;                   
	uint8_t       *QEnd; 
	uint8_t       *OSQIn;
	uint8_t       *OSQOut; 
	uint8_t       OSQSize;  
	uint8_t     OSQEntries;
}buf_type;

typedef enum { EMPTY=3,OK=1,FULL=2}buf;

typedef struct
{	
	double dt;
	int Magx;
	int Magy;
	int Magz;
	int MagYaw;
}Mag_TypeDef;

Mag_TypeDef  mag;
uint8_t      imu_buf[IMU_BUF_SIZE];     //?����??����??o��?
buf_type     imu_buf_contr_op;  

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


uint8_t   buf_read(buf_type * _os,uint8_t *p) 
{     
	if ( buf_rxnum(_os) > 0){  //��D��y?Y?��D��?��
		*p = *((_os->OSQOut)++); 
		if (_os->OSQOut==_os->QEnd){ 
			_os->OSQOut = _os->Start ; 
			
		}
		return OK;
	}
	return EMPTY;
}


void  buf_write(buf_type * _os,uint8_t *p)
{  
	if (_os->OSQIn ==_os->QEnd)  
		_os->OSQIn = _os->Start;
	*((_os->OSQIn)++)= *p ;
	if(_os->OSQIn==_os->QEnd) 
		_os->OSQIn=_os->Start; 
 }


bool CRC_Check(uint8_t *p)
{
	uint8_t temp = 0;
	for(int i = 2; i <= 9; i++)
	{
		temp += p[i];
	}

	temp = temp % 256;
	int rcc = p[10];

	if(rcc == temp)
	{
		// ROS_INFO("temp = %d, rcc= %d",temp,rcc);
		return true;
	}
	else
	{
		return false;
	}
}

static uint8_t buff[IMU_BUF_SIZE] = {EMPTY};
uint8_t K;
uint8_t * read_imu_buf(void)
{
		uint8_t i,data,temp;
	  static int index = 10;
	  static int count = 2;
	
    if ((*buff) == EMPTY )
		{  
    	if (*(buff+1) != 0xAA)
		{
			for (i=0;i<IMU_BUF_SIZE;i++)
			{  
				temp = buf_read(& imu_buf_contr_op,&data);
				if (temp == EMPTY ) return  buff;  

				if(data == 0xAA) 
				{
					*(buff+1) = data;
					break;
				}
			}
		}	

		// ROS_INFO("flag 1");

		for (;index>0;)
		{
			// ROS_INFO("flag 1.1");
			K=temp= buf_read(& imu_buf_contr_op,&data);
			if(temp == EMPTY ) return  buff; 
			*(buff + count) = data;
			count++;
			index--;
			if(index == 0)
			{
				count = 2;
				index = 10;
				if(CRC_Check(buff) && (buff[11] == 0xBB))
				{ 
					*buff = FULL;
					return  buff; 
				} 
				for(i = 1; i <=11; i++)  buff[i] = 0;

				buff[0] = EMPTY;    

				return  buff; 
			}
		}
  } 

}

bool dataUpdate = false;

int signData(uint8_t highData,uint8_t lowData)
{
	int result = lowData + highData * 256;
	uint16_t unit = 1;
	if(highData >= 128)
	{
		result = result - 256*256;
	}

	return result;
}


void dataAnalyse(Mag_TypeDef * magData)

{
	static ros::Time startTime = ros::Time::now();

	uint8_t *buf,*p,i;
	int16_t yaw,pitch,roll;
	buf=read_imu_buf();
	p=buf;
	if(*buf==FULL)
	{			
		magData->Magx = signData(p[2],p[1]);
		magData->Magy = signData(p[4],p[3]);
		magData->Magz = signData(p[6],p[5]);

		magData->MagYaw = signData(p[8],p[7]) / 10.0;

		magData->dt = ros::Time::now().toSec() - startTime.toSec();
		startTime = ros::Time::now();

		dataUpdate = true;

		//ROS_INFO("data is recepted successful");
		for(i=1;i<16;i++)  *(buf+i)=0;
		*buf=EMPTY;
	}
}

ros::Publisher mag_pub;

void updateMagData()
{
    geometry_msgs::PoseStamped message_mag;
    message_mag.header.stamp = ros::Time::now();
    message_mag.header.frame_id = "messageMag";

    message_mag.pose.position.x = mag.Magx;
	message_mag.pose.position.y = mag.Magy;
	message_mag.pose.position.z = mag.Magz;

    message_mag.pose.orientation.w = mag.MagYaw;

    mag_pub.publish(message_mag);
} 


geometry_msgs::PointStamped dataAngle;
geometry_msgs::PoseStamped dataIMU;

int main(int argc, char **argv)
{

  	ros::init(argc, argv, "mag_reader");
	ros::NodeHandle ni;
	// ros::Publisher imu_pub = ni.advertise<geometry_msgs::PointStamped>("/alf001_dis", 1000);

    mag_pub = ni.advertise<geometry_msgs::PoseStamped>("/mag_data", 1); // 1 buffer data

    static ros::Time current_time;

    string port("/dev/mag");
	// string port("/dev/ttyUSB0");

	// Argument 2 is the baudrate
	unsigned long baud = 9600;

	// port, baudrate, timeout in milliseconds
	serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1));
	buf_init((buf_type *)&imu_buf_contr_op ,(uint8_t  *)imu_buf,IMU_BUF_SIZE);

	if(my_serial.isOpen())
	{
	  ROS_INFO("mag serial open success");
	}
	else
	{
	  ROS_INFO("mag serial open failed");
	}

	ROS_INFO("starting to publish mag topic...\n");

	string result;
	uint8_t  data;

    ros::Time prev_time = ros::Time::now();

    //ros::Rate loop_rate(10000);
	while(ros::ok())
    {
		double dt = ros::Time::now().toSec() - prev_time.toSec();

        current_time=ros::Time::now();
		
		result = my_serial.read(1);

		if (result.length()) 
		{
			data = result.at(0);
			buf_write(&imu_buf_contr_op,&data);
			// ROS_INFO("read the data is %d",data);
		}
		
		dataAnalyse(&mag);
		
		if (dataUpdate)
		{
			updateMagData();
			dataUpdate = false;
		}

        ros::spinOnce();
    }

    return 0;
}

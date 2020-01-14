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

#define PI 3.1415926535897932384626433

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
	double Accx;
	double Accy;
	double Accz;

	double Gyrox;
	double Gyroy;
	double Gyroz;
	
	uint16_t Magx;
	uint16_t Magy;
	uint16_t Magz;
	uint16_t MagYaw;

	double pitch;
	double yaw;
	double roll;

	double temprature;
	double dt;
}IMU_TypeDef;


IMU_TypeDef  imu;
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
	for(int i = 1; i <= 8; i++)
	{
		temp += p[i];
	}

	temp = temp ;
	int rcc = p[9] ;

	// if( (p[29] == 0x0D) && (p[30] == 0x0A) ) % (256*256)  % (256*256)

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
	  static int index = 28;
	  static int count = 3;
	
    if ((*buff) == EMPTY )
		{  
    	if (*(buff+2) != 0xFF)
		{
			for (i=0;i<IMU_BUF_SIZE;i++)
			{  
				temp = buf_read(& imu_buf_contr_op,&data);
				if (temp == EMPTY ) return  buff;  

				if((*(buff+1) == 0xFF)&&(data == 0xFF)) 
				{
					*(buff+2) = data;
					break;
				}
				else
				{
					*(buff+1) = data;
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
				count = 3;
				index = 28;
				if(CRC_Check(buff))
				{ 
					*buff = FULL;
					return  buff; 
				} 
				for(i = 1; i <=30; i++)  buff[i] = 0;

				buff[0] = EMPTY;    

				return  buff; 
			}
		}
  } 

}

bool dataUpdate = false;

int signData(uint8_t highData,uint8_t lowData)
{
	int result = lowData | highData<<8;
	uint16_t unit = 1;
	if(highData & 0x80)
	{
		result = result - (unit<<16);
	}

	return result;
}

void Imudata_analyse(IMU_TypeDef * imuData)
{
	static ros::Time startTime = ros::Time::now();

	uint8_t *buf,*p,i;
	int16_t yaw,pitch,roll;
	buf=read_imu_buf();
	p=buf;
	if(*buf==FULL)
	{			
		imuData->Magx = signData(p[2],p[1]);
		imuData->Magy = signData(p[4],p[3]);
		imuData->Magz = signData(p[6],p[5]);

		imuData->MagYaw = signData(p[8],p[7]);

		imuData->dt = ros::Time::now().toSec() - startTime.toSec();

		startTime = ros::Time::now();

		// ROS_INFO("the head is: %d, %d:",p[1],p[2]);
		// ROS_INFO("the end is : %d, %d:",p[29],p[30]);

		dataUpdate = true;

		//ROS_INFO("data is recepted successful");
		for(i=1;i<16;i++)  *(buf+i)=0;

		*buf=EMPTY;
	}
}

/****************************************************************************************/

Eigen::Quaterniond euler2Quaternion(Eigen::Vector3d rotationVector)
{
    Eigen::AngleAxisd AngleX(rotationVector(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd AngleY(rotationVector(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd AngleZ(rotationVector(2), Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = AngleZ * AngleY * AngleX;
    return q;
}

ros::Publisher mag_pub;


void updateEulerAngle()
{
    Eigen::Vector3d eulerAngle(imu.roll,imu.pitch,imu.yaw);
    eulerAngle *= PI / 180;
    Eigen::Quaterniond Qbn = euler2Quaternion(eulerAngle);

    geometry_msgs::PoseArray messageAlf001;
    messageAlf001.poses.resize(2);
    messageAlf001.header.stamp = ros::Time::now();
    messageAlf001.header.frame_id = "messageDR";

    messageAlf001.poses[0].position.x = imu.roll;
    messageAlf001.poses[0].position.y = imu.pitch; 
    messageAlf001.poses[0].position.z = imu.yaw;

    messageAlf001.poses[0].orientation.x = Qbn.x();
    messageAlf001.poses[0].orientation.y = Qbn.y();
    messageAlf001.poses[0].orientation.z = Qbn.z();
    messageAlf001.poses[0].orientation.w = Qbn.w();

    messageAlf001.poses[1].position.x = imu.Accx;
    messageAlf001.poses[1].position.y = imu.Accy; 
    messageAlf001.poses[1].position.z = imu.Accz;

    messageAlf001.poses[1].orientation.x = imu.Magx;
    messageAlf001.poses[1].orientation.y = imu.Magy;
    messageAlf001.poses[1].orientation.z = imu.Magz;
    messageAlf001.poses[1].orientation.w = imu.MagYaw;

    mag_pub.publish(messageAlf001);
} 


geometry_msgs::PointStamped dataAngle;
geometry_msgs::PoseStamped dataIMU;

int main(int argc, char **argv)
{

  	ros::init(argc, argv, "imu_reader");
	ros::NodeHandle ni;
	// ros::Publisher imu_pub = ni.advertise<geometry_msgs::PointStamped>("/alf001_dis", 1000);

    mag_pub = ni.advertise<geometry_msgs::PoseArray>("/mymag", 1);

    static ros::Time current_time;

    //string port("/dev/imu");
	string port("/dev/ttyUSB0");

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
		Imudata_analyse(&imu);
		if (dataUpdate)
		{
			updateEulerAngle();
			// if(dt > 0.095)
			// {
			// 	if(initialFinish) topicPubish();
			// 	// ROS_INFO("get data delta time is %f.",dt);
		    //     prev_time = ros::Time::now();
			// }
			dataUpdate = false;
		}

        ros::spinOnce();
    }

    return 0;
}

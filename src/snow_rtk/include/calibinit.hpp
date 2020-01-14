#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d/calib3d_c.h"
#include <stdarg.h>
#include <vector>
#include <map>

#include <contour.h>
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

//ros
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <string>

using namespace std;
using namespace cv;


bool ytfindChessboardCorners(cv::Mat _image, cv::Size patternSize, cv::OutputArray corners, int flags);

bool init_calib(string filename, Mat &intrinsic, Mat &distortion);

void goto_calib_pose();

bool goto_calib();

bool find_chessboard();

void update_base(YAT_POINTF cali_base_pose);

void cali_calback(const std_msgs::Bool msg);

void pose_Callback(const geometry_msgs::PoseStampedConstPtr& msg_rac, const geometry_msgs::PoseArrayConstPtr& msg_alf);
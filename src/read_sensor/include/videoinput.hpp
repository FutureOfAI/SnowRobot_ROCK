#include "opencv2/opencv.hpp"
#include<pthread.h>
#include <iostream>
#include "unistd.h"

using namespace cv;
using namespace std;



class ImgThread
{
public:
    ImgThread();
    ~ImgThread();
    Mat getframe();
    void flushframe();
    bool open();
    void close();
    bool isruning;

private:
    pthread_mutex_t mutex_lock;
    pthread_t m_pthread;
    VideoCapture vp;
    Mat lastimg;
    
};
//
// Created by liangxiao on 2020/09/02.
//

#ifndef RINGDETECT5_XIMEA_H
#define RINGDETECT5_XIMEA_H

#include "stdafx.h"
using namespace std;
using namespace cv;



class Ximea: public xiAPIplusCameraOcv
{
public:
    Ximea(){}

    Ximea(const bool& slave, const unsigned int &width, const unsigned int &height, const unsigned int &framerate, char* serialNumber,const unsigned int &expTime_us, const bool& isRGB=false,const bool& isBinning= true);

    Ximea(const unsigned int &width, const unsigned int &height, const unsigned int &framerate, char* serialNumber, const unsigned int &expTime_us, const bool& isRGB=false, const bool& isBinning=true);

    Ximea(const unsigned int &width, const unsigned int &height, const unsigned int &framerate, char* serialNumber, const bool& isRGB=false,const bool& isBinning=true);

private:

    int initCamera(const unsigned int &width, const unsigned int &height, const unsigned int &framerate, char* serialNumber1,const unsigned int &expTime_us, const bool& isRGB, const bool &isBinning);

    int limitSizeStep(const unsigned int &size, const unsigned int &maxSize, const unsigned int &step);
};

class stereoXimea
{
public:
    Ximea cam1,cam2;
    stereoXimea(const unsigned int &width, const unsigned int &height, const unsigned int &framerate, char* serialNumber1, char* serialNumber2,const unsigned int &expTime_us, const bool& isRGB, const bool &isBinning);
    void getImages(Mat& left_image,Mat& eight_image);
    Mat matRotateClockWise180(Mat src);

private:

};

class XIMEA
{
public:
    XIMEA(const unsigned int &width, const unsigned int &height, char* serialNumber1, char* serialNumber2, const unsigned int &expTime_us,const bool isRGB=false, const bool setROI=false, const unsigned int &offsetx=0, const unsigned int &offsety=0, int mXi_TotalBandwidth=3200,int mXi_BandwidthMargin=10);
    ~XIMEA();
    void SetROI(const unsigned int &width, const unsigned int &height,const unsigned int &offsetx, const unsigned int &offsety);
    void getImages(Mat &view_left, Mat & view_right);
    Mat matRotateClockWise180(Mat src);
    xiAPIplusCameraOcv cam1, cam2;


private:

};




















#endif //RINGDETECT5_XIMEA_H

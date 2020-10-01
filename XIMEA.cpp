//
// Created by liangxiao on 2020/09/02.
//
#include "XIMEA.h"


/*Ximea->  class for single camera
 *XIMEA->  class for stereo  camera
 */


Ximea::Ximea(const bool& slave, const unsigned int &width, const unsigned int &height, const unsigned int &framerate, char* serialNumber, const unsigned int &expTime_us, const bool& isRGB,const bool& isBinning)
{

    initCamera(width, height, framerate, serialNumber, expTime_us, isRGB,isBinning);
    if (!slave) {
        SetTriggerSource(XI_TRG_SOFTWARE);
        SetGPOSelector(XI_GPO_PORT1);
        SetGPOMode(XI_GPO_EXPOSURE_ACTIVE);
    }
    else {
        SetTriggerSource(XI_TRG_EDGE_RISING);
        SetGPISelector(XI_GPI_PORT1);
        SetGPIMode(XI_GPI_TRIGGER);
    }

    std::cout << "Starting acquisition..." << std::endl;
    StartAcquisition();
    std::cout << "Camera started!!" << std::endl;
}

Ximea::Ximea(const unsigned int &width, const unsigned int &height, const unsigned int &framerate, char* serialNumber, const unsigned int &expTime_us, const bool& isRGB, const bool &isBinning)
{
    initCamera(width, height, framerate, serialNumber, expTime_us, isRGB,isBinning);

    std::cout << "Starting acquisition..." << std::endl;
    StartAcquisition();
    std::cout << "Camera started!!" << std::endl;
}

Ximea::Ximea(const unsigned int &width, const unsigned int &height, const unsigned int &framerate, char* serialNumber,const bool& isRGB, const bool &isBinning)
{
    const int expOffset = 210;
    const unsigned int expTime_us = static_cast<int>(1000.0 * 1000.0 / framerate) - expOffset;

    initCamera(width, height, framerate, serialNumber, expTime_us,isRGB, isBinning);

    std::cout << "Starting acquisition..." << std::endl;
    StartAcquisition();
    std::cout << "Camera started!!" << std::endl;
}

int Ximea::initCamera(const unsigned int & width, const unsigned int & height, const unsigned int & framerate, char* serialNumber,const unsigned int & expTime_us, const bool& isRGB, const bool& isBinning)
{
    std::cout << "Initialize camera" << std::endl;
    char serialChar[24];
    std::cout << serialNumber << std::endl;
    OpenBySN(serialNumber);
    std::cout << "Opening camera[" << serialNumber << "]..." << std::endl;
    const auto maxWidth = GetWidth_Maximum();
    const auto maxHeight = GetHeight_Maximum();
    std::cout<<"maxWidth: "<<maxWidth<<" maxHeight："<<maxHeight<<std::endl;
    if (isBinning && width <= maxWidth*0.5 && height <= maxHeight*0.5) {
        SetDownsampling(XI_DWN_2x2);
        SetDownsamplingType(XI_SKIPPING);
    }

    const auto widthStep = GetWidth_Increment();
    const auto heightStep = GetHeight_Increment();
    SetWidth(limitSizeStep(width, maxWidth, widthStep));
    SetHeight(limitSizeStep(height, maxHeight, heightStep));
    std::cout << "Image Width: " << GetWidth() << std::endl;
    std::cout << "Image Height: " << GetHeight() << std::endl;
    std::cout << "Image Height: " << GetHeight() << std::endl;

//    SetOffsetX(GetOffsetX_Maximum() / 2);
//    SetOffsetY(GetOffsetY_Maximum() / 2);
    SetOffsetX((maxWidth-width)/2);
    SetOffsetY((maxHeight-height)/2);
    std::cout << "Offset X: " << GetOffsetX() << std::endl;
    std::cout << "Offset Y: " << GetOffsetY() << std::endl;


    SetAcquisitionTimingMode(XI_ACQ_TIMING_MODE_FRAME_RATE);
    SetExposureTime(100);
    auto frameRate = static_cast<float>(framerate);
    const auto maxFR = static_cast<float>(GetFrameRate_Maximum()*0.9);
    const auto minFR = static_cast<float>(GetFrameRate_Minimum()*1.1);
    frameRate = std::min(frameRate, maxFR);
    frameRate = std::max(frameRate, minFR);
    SetFrameRate(frameRate);
    const unsigned int maxExpTime = 1000.0 * 1000.0 / frameRate * 0.9;
//    std::cout << maxExpTime << std::endl;
    SetExposureTime(std::min(expTime_us,maxExpTime));
    SetFrameRate(frameRate);
//    std::cout << "Framerate: " << minFR << " - " << maxFR << std::endl;
    std::cout << "Frame rate: " << GetFrameRate() << std::endl;
    std::cout << "Exp. Time: " << GetExposureTime() << std::endl;
    DisableAutoExposureAutoGain();

    XI_IMG_FORMAT format= GetImageDataFormat();
    if(isRGB)
        format = XI_RGB24;
    SetImageDataFormat(format);

    return 0;
}

int Ximea::limitSizeStep(const unsigned int & size, const unsigned int & maxSize, const unsigned int & step)
{
    auto checkedSize = std::min(size, maxSize);

    if (checkedSize % step != 0) {
        checkedSize -= step - size%step;
    }
    return checkedSize;
}




XIMEA::XIMEA (const unsigned int &width, const unsigned int &height,  char* serialNumber1,  char* serialNumber2, const unsigned int &expTime_us, const bool isRGB, const bool setROI, const unsigned int &offsetx, const unsigned int &offsety, int mXi_TotalBandwidth,int mXi_BandwidthMargin)
{
    try
    {
        cam1.OpenBySN(serialNumber1);//06956451
        cam2.OpenBySN(serialNumber2);//06953151
        cam1.DisableAutoBandwidthCalculation();
        cam2.DisableAutoBandwidthCalculation();
        int cameraDataRate = (int) (mXi_TotalBandwidth *0.5 * (100.0 - mXi_BandwidthMargin) / 100);
        cam1.SetBandwidthLimit(cameraDataRate);
        cam2.SetBandwidthLimit(cameraDataRate);
//        cout<<"camera1 bandwidth "<< cam1.GetBandwidthLimit()<<"camera2 bandwidth "<<cam2.GetBandwidthLimit()<<endl;
        cam1.SetSensorFeatureValue(XI_ON);
        cam2.SetSensorFeatureValue(XI_ON);
        cam1.SetExposureTime(expTime_us);
        cam2.SetExposureTime(expTime_us);

        cam1.SetGain(20);
        cam2.SetGain(20);

        XI_IMG_FORMAT format= cam1.GetImageDataFormat();
        if(isRGB)
            format = XI_RGB24;
        cam1.SetImageDataFormat(format);
        cam2.SetImageDataFormat(format);
        int sensor_feature_selector1 = 0;
        int sensor_feature_selector2 = 0;
        cam1.GetXIAPIParamInt(XI_PRM_SENSOR_FEATURE_SELECTOR, &sensor_feature_selector1);
        cam2.GetXIAPIParamInt(XI_PRM_SENSOR_FEATURE_SELECTOR, &sensor_feature_selector2);
        cam1.SetXIAPIParamInt(XI_PRM_SENSOR_FEATURE_SELECTOR,XI_SENSOR_FEATURE_ZEROROT_ENABLE);
        cam2.SetXIAPIParamInt(XI_PRM_SENSOR_FEATURE_SELECTOR,XI_SENSOR_FEATURE_ZEROROT_ENABLE);
        cam1.SetRegion_selector(0); // default is 0
        cam2.SetRegion_selector(0);
        if (setROI)
            SetROI(width,height,offsetx,offsety);
        cam1.SetDownsampling(XI_DWN_2x2);
        cam1.SetDownsamplingType(XI_SKIPPING);
        cam2.SetDownsampling(XI_DWN_2x2);
        cam2.SetDownsamplingType(XI_SKIPPING);
//        cam1.SetAcquisitionTimingMode(XI_ACQ_TIMING_MODE_FRAME_RATE);
//        cam1.SetFrameRate(500);
//        cam2.SetAcquisitionTimingMode(XI_ACQ_TIMING_MODE_FRAME_RATE);
//        cam2.SetFrameRate(500);

//        cam1.SetTriggerSource(XI_TRG_OFF);
//        cam1.SetGPIMode(XI_GPI_OFF);
//        cam1.SetGPOMode(XI_GPO_EXPOSURE_ACTIVE);
//        cam2.SetTriggerSource(XI_TRG_EDGE_RISING);
//        cam2.SetGPIMode(XI_GPI_TRIGGER);
//        cam2.SetGPOMode(XI_GPO_OFF);
//
//        cam1.SetTriggerSource(XI_TRG_SOFTWARE);
//        cam1.SetGPOSelector(XI_GPO_PORT1);
//        cam1.SetGPOMode(XI_GPO_EXPOSURE_ACTIVE);
//
//        cam2.SetGPISelector(XI_GPI_PORT1);
//        cam2.SetGPIMode(XI_GPI_TRIGGER);
//        cam2.SetTriggerSource(XI_TRG_EDGE_RISING);

//        cam1.SetXIAPIParamInt( XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE);
//        cam1.SetXIAPIParamInt( XI_PRM_GPO_SELECTOR,1);
//        cam1.SetXIAPIParamInt( XI_PRM_GPO_MODE,  XI_GPO_EXPOSURE_ACTIVE);
//        cam2.SetXIAPIParamInt( XI_PRM_GPI_SELECTOR, 1);
//        cam2.SetXIAPIParamInt( XI_PRM_GPI_MODE,  XI_GPI_TRIGGER);
//        cam2.SetXIAPIParamInt( XI_PRM_TRG_SOURCE, XI_TRG_EDGE_RISING);
        cam1.StartAcquisition();
        cam2.StartAcquisition();

//        waitKey(2234);
//        cam1.SetXIAPIParamInt( XI_PRM_TRG_SOFTWARE, 1);



    }
    catch(xiAPIplus_Exception& exp)
    {
        printf("Error:\n");
        exp.PrintError();
        cv::waitKey(1000);
    }
}

XIMEA::~XIMEA(){
    cam1.StopAcquisition();
    cam2.StopAcquisition();
    cam1.Close();
    cam2.Close();
    printf("Done\n");
    cv::waitKey(500);
}

void XIMEA::SetROI(const unsigned int &width, const unsigned int &height,const unsigned int &offsetx, const unsigned int &offsety) {
     cam2.SetWidth(width);
     cam2.SetHeight(height);
     cam2.SetOffsetX(offsetx);
     cam2.SetOffsetY(offsety);

     cam1.SetWidth(width);
     cam1.SetHeight(height);
     cam1.SetOffsetX(offsetx);
     cam1.SetOffsetY(offsety);
}

void XIMEA::getImages(Mat &view_left, Mat & view_right) {

    view_left = cam1.GetNextImageOcvMat();
    view_left = matRotateClockWise180(view_left);

    view_right = cam2.GetNextImageOcvMat();
    view_right = matRotateClockWise180(view_right);

}

Mat XIMEA::matRotateClockWise180(Mat src)//clockwise rotation 180
{
    if (src.empty())
    {
        cout << "Roratation Matrix src is empty!";
    }
    //flip(src, src, 0);// mode: flipCode == 0 vertical rotation，flipCode>0 horizontal rotation，flipCode<0 rotation 180
    //flip(src, src, 1);
    flip(src, src, -1);
    return src;
}

stereoXimea::stereoXimea(const unsigned int &width, const unsigned int &height, const unsigned int &framerate, char* serialNumber1, char* serialNumber2,const unsigned int &expTime_us, const bool& isRGB, const bool &isBinning)
{
    cam1=Ximea(width, height, framerate, serialNumber1, expTime_us, isRGB, isBinning);
    cam2=Ximea(width, height, framerate, serialNumber2, expTime_us, isRGB, isBinning);

}

void stereoXimea::getImages(Mat& left_image,Mat& right_image)
{
    left_image=cam1.GetNextImageOcvMat();
    left_image=matRotateClockWise180(left_image);

    right_image=cam2.GetNextImageOcvMat();
    right_image=matRotateClockWise180(right_image);
}

Mat stereoXimea::matRotateClockWise180(Mat src)//clockwise rotation 180
{
    if (src.empty())
    {
        cout << "Roratation Matrix src is empty!";
    }
    //flip(src, src, 0);// mode: flipCode == 0 vertical rotation，flipCode>0 horizontal rotation，flipCode<0 rotation 180
    //flip(src, src, 1);
    flip(src, src, -1);
    return src;
}

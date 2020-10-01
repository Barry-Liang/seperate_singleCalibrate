#include"stdafx.h"
#include"XIMEA.h"
#define CE(func) {XI_RETURN stat = (func); if (XI_OK!=stat) {printf("Error:%d returned from function:"#func"\n",stat);throw "Error";}}
using namespace cv;
using namespace std;
bool iscamera = false;
bool camopened = false;
const int FRAME_WIDTH = 1280;
const int FRAME_HEIGHT = 1024;
const unsigned int offsetX = 0;
const unsigned int offsetY = 0;
char* serialNumeber1= "13958850"; //left camera
char* serialNumeber2= "13956150"; //right camera
//    char* serialNumeber1= "06953151"; //left camera
//    char* serialNumeber2= "06956451"; //right camera
const unsigned int expTime = 1000;

static void help()
{
    cout <<  "This is a camera calibration sample." << endl
         <<  "Usage: camera_calibration [configuration_file -- default ./default.xml]"  << endl
         <<  "Near the sample file you'll find the configuration file, which has detailed help of "
             "how to edit it.  It may be any OpenCV supported file format XML/YAML." << endl;
}

class Settings
{
public:
    Settings() : goodInput(false) {}
    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
    enum InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };

    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{"
           << "BoardSize_Width"  << boardSize.width
           << "BoardSize_Height" << boardSize.height
           << "Square_Size"         << squareSize
           << "Calibrate_Pattern" << patternToUse
           << "Calibrate_NrOfFrameToUse" << nrFrames
           << "Calibrate_FixAspectRatio" << aspectRatio
           << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
           << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

           << "Write_DetectedFeaturePoints" << writePoints
           << "Write_extrinsicParameters"   << writeExtrinsics
           << "Write_outputFileName_left"  << outputFileName_left
           << "Write_outputFileName_right"  << outputFileName_right

           << "Show_UndistortedImage" << showUndistorsed

           << "Input_FlipAroundHorizontalAxis" << flipVertical
           << "Input_Delay" << delay
           << "Input" << input
           << "}";
    }
    void read(const FileNode& node)                          //Read serialization for this class
    {   node["isRGB" ] >> isRGB;
        node["BoardSize_Width" ] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Calibrate_Pattern"] >> patternToUse;
        node["Square_Size"]  >> squareSize;
        node["Calibrate_NrOfFrameToUse"] >> nrFrames;
        node["Calibrate_FixAspectRatio"] >> aspectRatio;
        node["Write_DetectedFeaturePoints"] >> writePoints;
        node["Write_extrinsicParameters"] >> writeExtrinsics;
        node["Write_outputFileName_left"] >> outputFileName_left;
        node["Write_outputFileName_right"] >> outputFileName_right;
        node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
        node["Calibrate_UseFisheyeModel"] >> useFisheye;
        node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
        node["Show_UndistortedImage"] >> showUndistorsed;
        node["Input"] >> input;
        node["Input_Delay"] >> delay;
        node["Fix_K1"] >> fixK1;
        node["Fix_K2"] >> fixK2;
        node["Fix_K3"] >> fixK3;
        node["Fix_K4"] >> fixK4;
        node["Fix_K5"] >> fixK5;

        validate();
    }
    void validate()   //test the input style
    {
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6)
        {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }
        if (nrFrames <= 0)
        {
            cerr << "Invalid number of frames " << nrFrames << endl;
            goodInput = false;
        }

        if (input.empty())      // Check for valid input
        { inputType = INVALID;
        }
        else {
            if (input[0] >= '0' && input[0] <= '9') {
                stringstream ss(input);
                ss >> cameraID;
                inputType = CAMERA;

            } else {
                if (isListOfImages(input) && readStringList(input, imageList)) {
                    inputType = IMAGE_LIST;
                    nrFrames = (nrFrames < (int) imageList.size()) ? nrFrames : (int) imageList.size();
                } else
                    inputType = VIDEO_FILE;
            }
            if (inputType == CAMERA) {
                iscamera = true;

            }
            else {
                if (inputType == VIDEO_FILE)
                    inputCapture.open(input);
                if (inputType != IMAGE_LIST && !inputCapture.isOpened())
                    inputType = INVALID;
            }
        }
        if (inputType == INVALID)
        {
            cerr << " Input does not exist: " << input;
            goodInput = false;
        }

        flag = 0;
        if(calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
        if(calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
        if(aspectRatio)            flag |= CALIB_FIX_ASPECT_RATIO;
        if(fixK1)                  flag |= CALIB_FIX_K1;
        if(fixK2)                  flag |= CALIB_FIX_K2;
        if(fixK3)                  flag |= CALIB_FIX_K3;
        if(fixK4)                  flag |= CALIB_FIX_K4;
        if(fixK5)                  flag |= CALIB_FIX_K5;

        if (useFisheye) {
            // the fisheye model has its own enum, so overwrite the flags
            flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC;
            if(fixK1)                   flag |= fisheye::CALIB_FIX_K1;
            if(fixK2)                   flag |= fisheye::CALIB_FIX_K2;
            if(fixK3)                   flag |= fisheye::CALIB_FIX_K3;
            if(fixK4)                   flag |= fisheye::CALIB_FIX_K4;
            if (calibFixPrincipalPoint) flag |= fisheye::CALIB_FIX_PRINCIPAL_POINT;
        }

        calibrationPattern = NOT_EXISTING;
        if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
        if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
        if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
        if (calibrationPattern == NOT_EXISTING)
        {
            cerr << " Camera calibration mode does not exist: " << patternToUse << endl;
            goodInput = false;
        }
        atImageList = 0;

    }
    Mat nextImage()
    {
        Mat result;

        if( atImageList < imageList.size() )
            result = imread(imageList[atImageList++], IMREAD_COLOR);

        return result;
    }

    static bool readStringList( const string& filename, vector<string>& l )
    {
        l.clear();
        FileStorage fs(filename, FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != FileNode::SEQ )
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        for( ; it != it_end; ++it )
            l.push_back((string)*it);
        return true;
    }

    static bool isListOfImages( const string& filename)
    {
        string s(filename);
        // Look for file extension
        if( s.find(".xml") == string::npos && s.find(".yaml") == string::npos && s.find(".yml") == string::npos )
            return false;
        else
            return true;
    }
public:
    Size boardSize;              // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;  // One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;                // The number of frames to use from the input for calibration
    float aspectRatio;           // The aspect ratio
    int delay;                   // In case of a video input
    bool writePoints;            // Write detected feature points
    bool writeExtrinsics;        // Write extrinsic parameters
    bool calibZeroTangentDist;   // Assume zero tangential distortion
    bool calibFixPrincipalPoint; // Fix the principal point at the center
    bool flipVertical;           // Flip the captured images around the horizontal axis
    string outputFileName_left;       // The name of the file where to write
    string outputFileName_right;
    // The name of the file where to write
    bool showUndistorsed;        // Show undistorted images after calibration
    string input;                // The input ->
    bool useFisheye;             // use fisheye camera model for calibration
    bool fixK1;                  // fix K1 distortion coefficient
    bool fixK2;                  // fix K2 distortion coefficient
    bool fixK3;                  // fix K3 distortion coefficient
    bool fixK4;                  // fix K4 distortion coefficient
    bool fixK5;                  // fix K5 distortion coefficient
    bool isRGB;                  // RGB camera or Gray camera
    int cameraID;
    vector<string> imageList;
    size_t atImageList;
    VideoCapture inputCapture;
    InputType inputType;
    bool goodInput;
    int flag;

private:
    string patternToUse;


};

static inline void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints,int left_right);

int main(int argc, char* argv[])
{   int count=0;
    help();
    //! [file_read]
    Settings s;
    //const string inputSettingsFile = argc > 1 ? argv[1] : "default.xml";
    const string inputSettingsFile="/home/liangxiao/XIMEA/package/samples/xiAPIplusOpenCV/seperate_singleCalibrate/in_VID5.xml";
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file
    //! [file_read]


    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

    vector<vector<Point2f> > imagePoints[2];
    Mat cameraMatrix_left, cameraMatrix_right, distCoeffs_left, distCoeffs_right;
    Size imageSize;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;

    try
    {

        XIMEA  stereoCam(FRAME_WIDTH,FRAME_HEIGHT,serialNumeber1,serialNumeber2,expTime,s.isRGB,false, offsetX, offsetY);
        camopened = true;
        Mat view_left, view_right;
        for(;;)
        {
            stereoCam.getImages(view_left,view_right);
            bool blinkOutput = false;
            cout<<"get frame number="<<imagePoints[0].size()<<endl;
            if( mode == CAPTURING && imagePoints[0].size() >= (size_t)s.nrFrames ) {
                if (runCalibrationAndSave(s, imageSize, cameraMatrix_left, distCoeffs_left, imagePoints[0],0)
                    && runCalibrationAndSave(s, imageSize, cameraMatrix_right, distCoeffs_right, imagePoints[1],1))
                {mode = CALIBRATED;
                cout<<"finished"<<endl;
                }
                else
                    mode = DETECTION;
            }
            if(view_left.empty()||view_right.empty())          // If there are no more images stop the loop
            {
                // if calibration threshold was not reached yet, calibrate now
                if( mode != CALIBRATED && !imagePoints[0].empty() )
                    runCalibrationAndSave(s, imageSize,  cameraMatrix_left, distCoeffs_left, imagePoints[0],0);
                    runCalibrationAndSave(s, imageSize,  cameraMatrix_right, distCoeffs_right, imagePoints[1],1);
                break;
            }
            //! [get_input]

            imageSize = view_left.size();  // Format input image.
            if( s.flipVertical ) {
                flip(view_left, view_left, 0);
                flip(view_right, view_right, 0);
            }
            //! [find_pattern]
            vector<Point2f> pointBuf_left,pointBuf_right;

            bool found;
            int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
            if(!s.useFisheye) {
                // fast check erroneously fails with high distortions like fisheye
                chessBoardFlags |= CALIB_CB_FAST_CHECK;
            }
            switch( s.calibrationPattern ) // Find feature points on the input format
            {
                case Settings::CHESSBOARD:
                    found = findChessboardCorners( view_left, s.boardSize, pointBuf_left, chessBoardFlags)&&findChessboardCorners( view_right, s.boardSize, pointBuf_right, chessBoardFlags);
                    break;
                case Settings::CIRCLES_GRID:
                    found = findCirclesGrid( view_left, s.boardSize, pointBuf_left )&&findCirclesGrid( view_right, s.boardSize, pointBuf_right );
                    break;
                case Settings::ASYMMETRIC_CIRCLES_GRID:
                    found = findCirclesGrid( view_left, s.boardSize, pointBuf_left, CALIB_CB_ASYMMETRIC_GRID )&&findCirclesGrid( view_right, s.boardSize, pointBuf_right, CALIB_CB_ASYMMETRIC_GRID );
                    break;
                default:
                    found = false;
                    break;
            }
            //! [find_pattern]
            //! [pattern_found]
            if ( found)                // If done with success,
            {
                cout<<"found"<<endl;
                Mat tem_left=view_left;
                Mat tem_right=view_right;
                 //improve the found corners' coordinate accuracy for chessboard
                if( s.calibrationPattern == Settings::CHESSBOARD)
                {
                    Mat viewGray_left=view_left;
                    Mat viewGray_right=view_right;
                    if(s.isRGB)
                    {
                        cvtColor(viewGray_left, viewGray_left, COLOR_BGR2GRAY);
                        cvtColor(viewGray_right, viewGray_right, COLOR_BGR2GRAY);
                    }

                    cornerSubPix( viewGray_left, pointBuf_left, Size(11,11),
                                  Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
                    cornerSubPix( viewGray_right, pointBuf_right, Size(11,11),
                                  Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
                }

                if( mode == CAPTURING &&  // For camera only take new samples after delay time
                    (camopened && clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) )
                {   if( s.calibrationPattern == Settings::CHESSBOARD) {
                        count++;
                        char file1[100], file2[100];
                        if (count < 10) {
                            sprintf(file1,
                                    "/home/liangxiao/XIMEA/package/samples/xiAPIplusOpenCV/stereoCalibrate/left0%u.jpg",
                                    count);
                            sprintf(file2,
                                    "/home/liangxiao/XIMEA/package/samples/xiAPIplusOpenCV/stereoCalibrate/right0%u.jpg",
                                    count);
                        } else {
                            sprintf(file1,
                                    "/home/liangxiao/XIMEA/package/samples/xiAPIplusOpenCV/stereoCalibrate/left%u.jpg",
                                    count);
                            sprintf(file2,
                                    "/home/liangxiao/XIMEA/package/samples/xiAPIplusOpenCV/stereoCalibrate/right%u.jpg",
                                    count);
                        }
                        imwrite(file1, tem_left);
                        imwrite(file2, tem_right);
                    }
                    if( s.calibrationPattern == Settings::ASYMMETRIC_CIRCLES_GRID) {
                        count++;
                        char file1[100], file2[100];
                        if (count < 10) {
                            sprintf(file1,
                                    "/home/liangxiao/XIMEA/package/samples/xiAPIplusOpenCV/seperate_singleCalibrate/left0%u.jpg",
                                    count);
                            sprintf(file2,
                                    "/home/liangxiao/XIMEA/package/samples/xiAPIplusOpenCV/seperate_singleCalibrate/right0%u.jpg",
                                    count);
                        } else {
                            sprintf(file1,
                                    "/home/liangxiao/XIMEA/package/samples/xiAPIplusOpenCV/seperate_singleCalibrate/left%u.jpg",
                                    count);
                            sprintf(file2,
                                    "/home/liangxiao/XIMEA/package/samples/xiAPIplusOpenCV/seperate_singleCalibrate/right%u.jpg",
                                    count);
                        }
                        imwrite(file1, tem_left);
                        imwrite(file2, tem_right);
                    }
                    imagePoints[0].push_back(pointBuf_left);
                    imagePoints[1].push_back(pointBuf_right);
                    prevTimestamp = clock();
                    blinkOutput = camopened;
                    cout<<"addnumber"<<endl;
                }
                // Draw the corners.
                drawChessboardCorners( view_left, s.boardSize, Mat(pointBuf_left), found );
                drawChessboardCorners( view_right, s.boardSize, Mat(pointBuf_right), found );
            }
            //! [pattern_found]
            //----------------------------- Output Text ------------------------------------------------
            //! [output_text]
            string msg = (mode == CAPTURING) ? "100/100" :
                         mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
            int baseLine = 0;
            Size textSize = getTextSize(msg, 3, 3, 3, &baseLine);
            Point textOrigin(view_left.cols - 2*textSize.width - 10, view_left.rows - 2*baseLine - 10);

                if( mode == CAPTURING )
                {
                    if(s.showUndistorsed)
                        msg = cv::format( "%d/%d Undist", (int)imagePoints[0].size(), s.nrFrames);
                    else
                        msg = cv::format( "%d/%d", (int)imagePoints[0].size(), s.nrFrames);
                }

                putText( view_left, msg, textOrigin, 3, 3, mode == CALIBRATED ?  GREEN : RED);
                putText( view_right, msg, textOrigin, 3, 3, mode == CALIBRATED ?  GREEN : RED);

            if( blinkOutput )
                bitwise_not(view_left, view_left);
                bitwise_not(view_right, view_right);
            //! [output_text]
            //------------------------- Video capture  output  undistorted ------------------------------
            //! [output_undistorted]
            if( mode == CALIBRATED && s.showUndistorsed )
            {
                Mat temp_left = view_left.clone();
                Mat temp_right = view_right.clone();
                if (s.useFisheye) {
                    cv::fisheye::undistortImage(temp_left, view_left, cameraMatrix_left, distCoeffs_left);
                    cv::fisheye::undistortImage(temp_right, view_right, cameraMatrix_right, distCoeffs_right);
                }
                else {
                    undistort(temp_left, view_left, cameraMatrix_left, distCoeffs_left);
                }
            }
            //! [output_undistorted]
            //------------------------------ Show image and check for input commands -------------------
            //! [await_input]
            imshow("Image View left", view_left);
            imshow("Image View right", view_right);
            char key = (char)waitKey(camopened ? 50 : s.delay);
           // cout<<"key"<<key<<endl;
            if( key  == ESC_KEY )
                break;
            if( key == 'u' && mode == CALIBRATED )
                s.showUndistorsed = !s.showUndistorsed;
            if( camopened && key == 'g' )
            {   cout<<"pressed g"<<endl;
                mode = CAPTURING;
                imagePoints[0].clear();
                imagePoints[1].clear();
            }
            //! [await_input]
            if (mode == CALIBRATED)
            {
                cout<<"calibrated"<<endl;
                stereoCam.~XIMEA();
                return 0;
            }
        }




        // -----------------------Show the undistorted image for the image list ------------------------
        //! [show_results]
        if( s.inputType == Settings::IMAGE_LIST && s.showUndistorsed )
        {
            Mat view_left, rview_left, map1, map2,view_right, rview_right, map3, map4;

            if (s.useFisheye)
            {
                Mat newCamMat_left,newCamMat_right;
                fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix_left, distCoeffs_left, imageSize,
                                                                    Matx33d::eye(), newCamMat_left, 1);
                fisheye::initUndistortRectifyMap(cameraMatrix_left, distCoeffs_left, Matx33d::eye(), newCamMat_left, imageSize,
                                                 CV_16SC2, map1, map2);
                fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix_right, distCoeffs_right, imageSize,
                                                                    Matx33d::eye(), newCamMat_right, 1);
                fisheye::initUndistortRectifyMap(cameraMatrix_right, distCoeffs_right, Matx33d::eye(), newCamMat_right, imageSize,
                                                 CV_16SC2, map3, map4);

            }
            else
            {
                initUndistortRectifyMap(
                        cameraMatrix_left, distCoeffs_left, Mat(),
                        getOptimalNewCameraMatrix(cameraMatrix_left, distCoeffs_left, imageSize, 1, imageSize, 0), imageSize,
                        CV_16SC2, map1, map2);
                initUndistortRectifyMap(
                        cameraMatrix_right, distCoeffs_right, Mat(),
                        getOptimalNewCameraMatrix(cameraMatrix_right, distCoeffs_right, imageSize, 1, imageSize, 0), imageSize,
                        CV_16SC2, map3, map4);
            }

            for(size_t i = 0; i < s.imageList.size(); i++ )//may need further modification
            {   if (i%2==0) {
                   view_left = imread(s.imageList[i], IMREAD_COLOR);
                   if (view_left.empty())
                       continue;
                   remap(view_left, rview_left, map1, map2, INTER_LINEAR);
                   imshow("Image View right", rview_left);
                   char c = (char)waitKey();
                   if( c  == ESC_KEY || c == 'q' || c == 'Q' )
                       break;
               }
            else {
                   view_right = imread(s.imageList[i], IMREAD_COLOR);
                   if (view_right.empty())
                       continue;
                   remap(view_right, rview_right, map3, map4, INTER_LINEAR);
                   imshow("Image View right", rview_right);
                   char c = (char)waitKey();
                   if( c  == ESC_KEY || c == 'q' || c == 'Q' )
                       break;
               }

            }
        }
        waitKey(500);


    }

    catch(xiAPIplus_Exception& exp)
    {
        printf("Error:\n");
        exp.PrintError();
#ifdef WIN32
        Sleep(2000);
#endif
       waitKey(2000);
        return -1;
    }

    //! [get_input]
    //! [show_results]

    return 0;
}

//! [compute_errors]
static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors, bool fisheye)
{
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for(size_t i = 0; i < objectPoints.size(); ++i )
    {
        if (fisheye)
            fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
                                   distCoeffs);
        else
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);

        err = norm(imagePoints[i], imagePoints2, NORM_L2);
        size_t n = objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}
//! [compute_errors]
//! [board_corners]
static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();

    switch(patternType)
    {
        case Settings::CHESSBOARD:
        case Settings::CIRCLES_GRID:
            for( int i = 0; i < boardSize.height; ++i )
                for( int j = 0; j < boardSize.width; ++j )
                    corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
            break;

        case Settings::ASYMMETRIC_CIRCLES_GRID:
            for( int i = 0; i < boardSize.height; i++ )
                for( int j = 0; j < boardSize.width; j++ )
                    corners.push_back(Point3f((2*j + i % 2)*squareSize, i*squareSize, 0));
            break;
        default:
            break;
    }
}
//! [board_corners]
static bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr)
{
    //! [fixed_aspect]
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( s.flag & CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = s.aspectRatio;
    //! [fixed_aspect]
    if (s.useFisheye) {
        distCoeffs = Mat::zeros(4, 1, CV_64F);
    } else {
        distCoeffs = Mat::zeros(8, 1, CV_64F);
    }

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms;

    if (s.useFisheye) {
        Mat _rvecs, _tvecs;
        rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs,
                                 _tvecs, s.flag);

        rvecs.reserve(_rvecs.rows);
        tvecs.reserve(_tvecs.rows);
        for(int i = 0; i < int(objectPoints.size()); i++){
            rvecs.push_back(_rvecs.row(i));
            tvecs.push_back(_tvecs.row(i));
        }
    } else {
        rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
                              s.flag);
    }

    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
                                            distCoeffs, reprojErrs, s.useFisheye);

    return ok;
}

// Print camera parameters to the output file
static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr,int left_right ) {
    FileStorage fs;
    if (left_right==0)
        fs.open(s.outputFileName_left, FileStorage::WRITE);
    else
        fs.open(s.outputFileName_right, FileStorage::WRITE);

    time_t tm;
    time(&tm);
    struct tm *t2 = localtime(&tm);
    char buf[1024];
    strftime(buf, sizeof(buf), "%c", t2);
    fs << "calibration_time" << buf;

    if (!rvecs.empty() || !reprojErrs.empty())
        fs << "nr_of_frames" << (int) std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << s.boardSize.width;
    fs << "board_height" << s.boardSize.height;
    fs << "square_size" << s.squareSize;
    if (s.flag & CALIB_FIX_ASPECT_RATIO)
        fs << "fix_aspect_ratio" << s.aspectRatio;
    if (s.flag) {
        std::stringstream flagsStringStream;
        if (s.useFisheye) {
            flagsStringStream << "flags:"
                              << (s.flag & fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "")
                              << (s.flag & fisheye::CALIB_FIX_K1 ? " +fix_k1" : "")
                              << (s.flag & fisheye::CALIB_FIX_K2 ? " +fix_k2" : "")
                              << (s.flag & fisheye::CALIB_FIX_K3 ? " +fix_k3" : "")
                              << (s.flag & fisheye::CALIB_FIX_K4 ? " +fix_k4" : "")
                              << (s.flag & fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
        } else {
            flagsStringStream << "flags:"
                              << (s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
                              << (s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
                              << (s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
                              << (s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
                              << (s.flag & CALIB_FIX_K1 ? " +fix_k1" : "")
                              << (s.flag & CALIB_FIX_K2 ? " +fix_k2" : "")
                              << (s.flag & CALIB_FIX_K3 ? " +fix_k3" : "")
                              << (s.flag & CALIB_FIX_K4 ? " +fix_k4" : "")
                              << (s.flag & CALIB_FIX_K5 ? " +fix_k5" : "");
        }
        fs.writeComment(flagsStringStream.str());
    }

    fs << "flags" << s.flag;

    fs << "fisheye_model" << s.useFisheye;
    if (left_right == 0) {
        fs << "camera_matrix_left" << cameraMatrix;
        fs << "distortion_coefficients_left" << distCoeffs;
        fs << "avg_reprojection_error_left" << totalAvgErr;
    }
    else
    {   fs << "camera_matrix_right" << cameraMatrix;
        fs << "distortion_coefficients_right" << distCoeffs;
        fs << "avg_reprojection_error_right" << totalAvgErr;
    }
    if (s.writeExtrinsics && !reprojErrs.empty())
    {
        if (left_right == 0)
            fs << "per_view_reprojection_errors_left" << Mat(reprojErrs);
        else
            fs << "per_view_reprojection_errors_right" << Mat(reprojErrs);
    }

    if(s.writeExtrinsics && !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
        bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
        bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

        for( size_t i = 0; i < rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(int(i), int(i+1)), Range(0,3));
            Mat t = bigmat(Range(int(i), int(i+1)), Range(3,6));

            if(needReshapeR)
                rvecs[i].reshape(1, 1).copyTo(r);
            else
            {
                //*.t() is MatExpr (not Mat) so we can use assignment operator
                CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
                r = rvecs[i].t();
            }

            if(needReshapeT)
                tvecs[i].reshape(1, 1).copyTo(t);
            else
            {
                CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
                t = tvecs[i].t();
            }
        }
        if (left_right == 0) {
            fs.writeComment("left:a set of 6-tuples (rotation vector + translation vector) for each view");
            fs << "extrinsic_parameters_left" << bigmat;
        }
        else{
            fs.writeComment("right:a set of 6-tuples (rotation vector + translation vector) for each view");
            fs << "extrinsic_parameters_right" << bigmat;
        }

    }

    if(s.writePoints && !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( size_t i = 0; i < imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        if (left_right == 0) {
            fs << "image_points_left" << imagePtMat;
        }
        else{
            fs << "image_points_right" << imagePtMat;

        }

    }
}

//! [run_and_save]
bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints,int left_right)
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;
    bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
                             totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
         << ". avg re projection error = " << totalAvgErr << endl;
    if (ok&&  s.calibrationPattern == Settings::ASYMMETRIC_CIRCLES_GRID)
        saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
                         totalAvgErr, left_right);
    return ok;
}
//! [run_and_save]

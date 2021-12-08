#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <sys/time.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

class Settings
{
public:
    Settings() : goodInput(false) {}

    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "image_width"  << image_width;
        fs << "image_height" << image_height;
        fs << "camera_matrix" << camera_matrix;
        fs << "distortion_coefficients" << distortion_coefficients;
    }

    void read(const FileNode& node)                          //Read serialization for this class
    {
        node["image_width" ] >> image_width;
        node["image_height"] >> image_height;
        node["camera_matrix"] >> camera_matrix;
        node["distortion_coefficients"] >> distortion_coefficients;

        validate();
    }

    void read(const FileStorage& fs)                          //Read serialization for this class
    {
        fs["image_width" ] >> image_width;
        fs["image_height"] >> image_height;
        fs["camera_matrix"] >> camera_matrix;
        fs["distortion_coefficients"] >> distortion_coefficients;

        validate();
    }

    void validate()
    {
        goodInput = true;
        if (image_width <= 0 || image_height <= 0)
        {
            cerr << "Invalid image size: " << image_width << " " << image_height << endl;
            goodInput = false;
        }
    }

public:
    int image_width;
    int image_height;
    Mat camera_matrix;
    Mat distortion_coefficients;
    Mat rectification_matrix;
    Mat projection_matrix;
    bool goodInput;

    bool useFisheye;             // use fisheye camera model for calibration

};

int main(int argc, char* argv[])
{

    const String keys
        = "{help h usage ? |           | print this message            }"
          "{@settings      |default.xml| input setting file            }";
    CommandLineParser parser(argc, argv, keys);
    parser.about("This is a camera calibration sample.\n"
                 "Usage: camera_calibration [configuration_file -- default ./default.xml]\n"
                 "Near the sample file you'll find the configuration file, which has detailed help of "
                 "how to edit it. It may be any OpenCV supported file format XML/YAML.");
    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    std::vector<cv::String> imageList;
    std::string imageDir("/home/henry/Documents/data/000-calibration/");
    std::string imageSuffix("*.png");
    glob((imageDir+imageSuffix).c_str(), imageList, false);

    //! [file_read]
    Settings s;
    const string inputSettingsFile = parser.get<string>(0);
    std::cout << "input file name: " << inputSettingsFile << std::endl;
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        parser.printMessage();
        return -1;
    }
    int width;
    s.read(fs);
    fs.release();                                         // close Settings file
    //! [file_read]

    //FileStorage fout("settings.yml", FileStorage::WRITE); // write config as YAML
    //fout << "Settings" << s;

    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }



    // if( mode == CALIBRATED && s.showUndistorted )
    // {
    //     Mat temp = view.clone();
    //     if (s.useFisheye)
    //     {
    //         Mat newCamMat;
    //         fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize,
    //                                                             Matx33d::eye(), newCamMat, 1);
    //         cv::fisheye::undistortImage(temp, view, cameraMatrix, distCoeffs, newCamMat);
    //     }
    //     else
    //         undistort(temp, view, cameraMatrix, distCoeffs);
    // }
    // -----------------------Show the undistorted image for the image list ------------------------
    //! [show_results]

    Mat cameraMatrix = s.camera_matrix.clone();
    Mat distCoeffs = s.distortion_coefficients.clone();
    Size imageSize;
    imageSize.width = s.image_width;
    imageSize.height = s.image_height;
    const char ESC_KEY = 27;

    if( !cameraMatrix.empty())
    {
        Mat view, rview, map1, map2;

        if (s.useFisheye)
        {
            Mat newCamMat;
            fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize,
                                                                Matx33d::eye(), newCamMat, 1);
            fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, Matx33d::eye(), newCamMat, imageSize,
                                             CV_16SC2, map1, map2);
        }
        else
        {
            initUndistortRectifyMap(
                cameraMatrix, distCoeffs, Mat(),
                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0, imageSize, 0), imageSize,
                CV_16SC2, map1, map2);
        }

        std::cout << "imageSize: " << imageSize << std::endl;



        std::vector<Mat> imageReadList;

        for(size_t i = 0; i < imageList.size(); i++ )
        {
            view = imread(imageList[i], IMREAD_COLOR);
            if(view.empty())
                continue;
            imageReadList.push_back(view);
        }
        
        timeval begin, end;
        long seconds, u_seconds;
        gettimeofday(&begin, 0);

        for(size_t i = 0; i < imageReadList.size(); i++ )
        {
            remap(imageReadList[i], rview, map1, map2, INTER_LINEAR);
            // undistort(imageReadList[i], rview, cameraMatrix, distCoeffs);
            namedWindow("Image", WINDOW_GUI_NORMAL);
            imshow("Image", rview);
            char c = (char)waitKey();
            if( c  == ESC_KEY || c == 'q' || c == 'Q' )
                break;

        }

        gettimeofday(&end, 0);
        seconds = end.tv_sec - begin.tv_sec;
        u_seconds = end.tv_usec - begin.tv_usec;
        std::cout << seconds << "s " << u_seconds << "us" << std::endl;
    }
    //! [show_results]

    return 0;
}
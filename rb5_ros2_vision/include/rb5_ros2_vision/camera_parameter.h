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

class CameraParameter
{
public:
    CameraParameter() : goodInput(false) {}

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

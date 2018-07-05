#include <iostream>
#include <opencv2/opencv.hpp>
#include "ArucoMarker/ArucoMarker.h"
#include "CamCalibWI/CamCalibWI.h"

int main() {


    timur::ArucoMarkers arucoMarkers(0.64,false);
    timur::CamCalibWi camCalibWi("../CamCalibStable.txt");
    cv::VideoCapture cap(0);

    std::vector<cv::Vec3d> rvec, tvec;
    std::vector<int> markId;
    cv::Mat frame;
    cap >> frame;
    while(!arucoMarkers.estimateMarkersPose(frame,camCalibWi.cameraMatrix(),camCalibWi.distortionCoefficients(),rvec,tvec,markId))
    {
        cap >> frame;
    }
    std::cout << tvec[0] << ' ' <<rvec[0] << '\n';
    return 0;
}
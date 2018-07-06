//
// Created by timur on 06.07.18.
//

#ifndef COMPUTERVISIONSYSTEM_CVS_H
#define COMPUTERVISIONSYSTEM_CVS_H

#include <iostream>
#include <opencv2/calib3d.hpp>

#include "ArucoMarker/ArucoMarker.h"
#include "CamCalibWI/CamCalibWI.h"
#include "FanucModel/FanucModel.h"


namespace timur
{
    class CVS {
    private:
        float arucoSqureDimension = 0.062f;
        timur::ArucoMarkers arucoMarkers = timur::ArucoMarkers(arucoSqureDimension, false);
        cv::VideoCapture vid = cv::VideoCapture(1);
        timur::CamCalibWi camera = timur::CamCalibWi("../CamCalibStable.txt");
        nikita::FanucModel fanuc;

        cv::Mat createTransformationMatrix(const cv::Vec3d& rotationVector,
                                                  const cv::Vec3d& translationVector);
        std::array<double, 3> calculateMarkerPose(cv::Mat transformationMatrix,
                                                         std::array<double, 6> jointCorners);
    public:
        CVS();
        ~CVS();

        std::array<double, 3> getMarkerPose(std::array<double, 6> jointCorners);
    };
}

#endif //COMPUTERVISIONSYSTEM_CVS_H

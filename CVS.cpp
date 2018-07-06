//
// Created by timur on 06.07.18.
//

#include "CVS.h"

cv::Mat timur::CVS::createTransformationMatrix(const cv::Vec3d& rotationVector,
                                   const cv::Vec3d& translationVector)
{
    cv::Mat rotationMatrix;
    cv::Rodrigues(rotationVector, rotationMatrix);
    cv::Mat transformationMatrix = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
    rotationMatrix.copyTo(transformationMatrix(cv::Rect(0, 0, 3, 3)));
    cv::Mat(translationVector * 1000).copyTo(transformationMatrix(cv::Rect(3, 0, 1, 3)));
    transformationMatrix.at<double>(3, 3) = 1;
    return transformationMatrix;
}

std::array<double, 3> timur::CVS::calculateMarkerPose(const cv::Mat transformationMatrix,
                                          const std::array<double, 6> jointCorners)
{
    const cv::Mat p6 = fanuc.fanucForwardTask(jointCorners);
    cv::Mat res = p6 * fanuc.getToCamera() * transformationMatrix * fanuc.getToSixth();
    return std::array<double, 3>{res.at<double>(0, 3), res.at<double>(1, 3), res.at<double>(2, 3)};
}

timur::CVS::CVS()
{
    if (!vid.isOpened())
    {
        throw std::system_error();
    }
}

timur::CVS::~CVS()
{
    vid.release();
}

std::array<double, 3> timur::CVS::getMarkerPose(const std::array<double, 6> jointCorners)
{
    cv::Mat frame;
    std::vector<cv::Vec3d> rotationVectors, translationVectors;
    std::vector<int> markerIds;
    if (!vid.read(frame))
    {
        throw std::system_error();
    }

    bool foundMarkers = arucoMarkers.estimateMarkersPose(frame, camera.cameraMatrix(),
                                                               camera.distortionCoefficients(),
                                                               rotationVectors,
                                                               translationVectors, markerIds);
    if(foundMarkers)
    {
        cv::Mat transformMatrix = createTransformationMatrix(cv::Vec3d(0, 0, 0), translationVectors[0]);
        std::array<double, 3> res = calculateMarkerPose(transformMatrix,jointCorners);
        return res;
    }
    return std::array<double, 3>{0.0, 0.0, 0.0};
}

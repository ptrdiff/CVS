cmake_minimum_required(VERSION 3.10)
project(ComputerVisionSystem)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

add_library(ArucoMarker STATIC ArucoMarker/ArucoMarker.cpp)
target_link_libraries(ArucoMarker ${OpenCV_LIBS} stdc++fs)

add_library(CamCalibWI STATIC CamCalibWI/CamCalibWI.cpp CamCalibWI/CameraCalibration/CameraCalibration.cpp)
target_link_libraries(CamCalibWI ${OpenCV_LIBS})

add_library(FanucModel STATIC FanucModel/RoboModelGeneral/RoboModelGeneral.cpp FanucModel/FanucModel.cpp)
target_link_libraries(FanucModel ${OpenCV_LIBS})

add_library(ComputerVisionSystem CVS.cpp)
target_link_libraries(ComputerVisionSystem ${OpenCV_LIBS} CamCalibWI ArucoMarker FanucModel)
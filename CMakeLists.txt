cmake_minimum_required(VERSION 3.21)
project(coconut)

#notice the "recycling" of CMAKE_C_FLAGS
#this is necessary to pick up android flags
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pedantic" )

# Eigen3
# include(cmake/find_eigen3.cmake)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "EIGEN3_VERSION: ${EIGEN3_VERSION_STRING}")

# OpenCV
find_package(OpenCV 3 COMPONENTS opencv_core opencv_cudacodec opencv_cudaimgproc opencv_cudastereo
                                 opencv_cudawarping opencv_highgui opencv_imgcodecs opencv_imgproc
                                 opencv_tracking opencv_cudafeatures2d REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
message(STATUS "OpenCV_VERSION: ${OpenCV_VERSION}")


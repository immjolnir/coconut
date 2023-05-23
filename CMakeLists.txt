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
find_package(OpenCV 4 COMPONENTS opencv_core opencv_highgui opencv_imgcodecs opencv_imgproc
                                 opencv_tracking REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
message(STATUS "OpenCV_VERSION: ${OpenCV_VERSION}")

# PCL
find_package(PCL COMPONENTS common filters io sample_consensus visualization segmentation REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")
message(STATUS "PCL_VERSION: ${PCL_VERSION}")
# VTK brought be PCL will add some global definitions that will lead to nvcc compilation problems.
get_directory_property(DirDefs COMPILE_DEFINITIONS)
foreach(d ${DirDefs})
  if(d MATCHES "vtk")
    remove_definitions(-D${d})
  endif()
endforeach()

# absl
find_package(absl REQUIRED)

add_subdirectory(examples)

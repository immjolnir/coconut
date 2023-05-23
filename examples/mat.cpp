/*
* 3 purpose
* 1. Shadow copy
* 2. Deep copy
*    2.1 copyTo
*    2.2 re-create
* 3. matrix multiplication
*/
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

// modules/core/include/opencv2/core/mat.hpp
//
int main() {
    // create a new image made of 240 rows and 320 columns
    cv::Mat image1(240, 320, CV_8U, 100);
    cv::imshow("Image", image1);  // show the image
    cv::waitKey(0);               // wait for a key pressed
    {                             // shadow copy
        cv::Mat copy = image1;
        copy = 255;
        cv::imshow("Origin", image1); // Change to white
        cv::imshow("Copy", copy); // change to white two
        cv::waitKey(0);
    }

    {  // Deep copy
        cv::Mat copy;
        image1.copyTo(copy);
        copy = 0;
        cv::imshow("Origin", image1);  // no change
        cv::imshow("Copy", copy);      // black
        cv::waitKey(0);
    }

    // re-allocate a new image
    // (only if size or type are different)
    image1.create(200, 200, CV_8U);
    image1 = 200;

    cv::imshow("Image1", image1);  // show the image
    cv::waitKey(0);                // wait for a key pressed

    // Test cv::Matx
    // a 3x3 matrix of double-precision
    // clang-format off
    cv::Matx33d matrix(3.0, 2.0, 1.0,
                       2.0, 1.0, 3.0,
                       1.0, 2.0, 3.0);
    // clang-format on
    // a 3x1 matrix (a vector)
    cv::Matx31d vector(5.0, 1.0, 3.0);
    // multiplication
    cv::Matx31d result = matrix * vector;

    std::cout << result << std::endl;
}

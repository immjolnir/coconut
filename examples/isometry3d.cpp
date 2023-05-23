#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <cxxabi.h>
#include <sstream>
#include <string>
#include <typeinfo>  // for typeid

template <typename T>
std::string typeInfo(T& t) {
    int status;
    char* realname;
    const std::type_info& ti = typeid(t);
    realname = abi::__cxa_demangle(ti.name(), NULL, NULL, &status);
    std::stringstream ss;
    ss << "Type Signature: " << ti.name() << "\t=> " << realname << "\t: " << status;
    return ss.str();
}

#include <iostream>

int main() {
    std::string calib_path = "/work/data/issues/PE-31623/res/j7-l4e-LFWSRXSJ8M1F50504_20230412_front_left_camera.yml";

    Eigen::Matrix4d _Tr_cam_to_imu;
    Eigen::Isometry3d _Tr_cam2veh = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d _Tr_veh2cam = Eigen::Isometry3d::Identity();

    cv::FileStorage fs(calib_path, cv::FileStorage::READ);
    cv::Mat Tr_rectifyCam2Imu, R_originCam2rectifyCam;
    Eigen::Matrix<double, 4, 4> Tr;
    Eigen::Matrix<double, 3, 3> R;
    fs["R"] >> R_originCam2rectifyCam;
    fs["Tr_cam_to_imu"] >> Tr_rectifyCam2Imu;

    cv::cv2eigen(R_originCam2rectifyCam, R);
    cv::cv2eigen(Tr_rectifyCam2Imu, Tr);

    std::cout << "R=" << R << std::endl;
    std::cout << "Tr=" << Tr << std::endl;

    _Tr_cam2veh = Eigen::Isometry3d(Tr);
    {
        // https://stackoverflow.com/questions/52255810/displaying-an-affine-transformation-in-eigen
        // std::cout << "_Tr_cam2veh=" << _Tr_cam2veh << std::endl;
        Eigen::Matrix3d m = _Tr_cam2veh.rotation();
        Eigen::Vector3d t = _Tr_cam2veh.translation();
        std::cout << "rotation=" << m << std::endl;
        std::cout << "translation=" << t << std::endl;
    }

    // Eigen::Transform<double, 3, 1, 0>
    std::cout << "typeinfo \n" << typeInfo(_Tr_cam2veh) << "\n" << std::endl;

    _Tr_cam2veh.matrix().block(0, 0, 3, 3) = _Tr_cam2veh.matrix().block(0, 0, 3, 3) * R;
    _Tr_veh2cam = _Tr_cam2veh.inverse();

    auto m = _Tr_cam2veh.matrix();
    std::cout << "_Tr_cam2veh.matrix=" << m << std::endl;
    std::cout << "typeinfo \n" << typeInfo(m) << "\n" << std::endl;  // Eigen::Matrix<double, 4, 4, 0, 4, 4>	: 0

    auto rotation = m.block(0, 0, 3, 3);
    std::cout << "rotation: " << rotation << std::endl;

    // Eigen::Vector3d optical_center = m.block(0, 3, 0, 3);
    Eigen::Vector3d translation = m.block(0, 3, 3, 1);
    std::cout << "translation: " << translation << std::endl;
}

/**
~/github/eigen/Eigen/src/Core/Block.h
138      Dynamic-size constructor
139
140     EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
141     Block(XprType& xpr,
142           Index startRow, Index startCol,
143           Index blockRows, Index blockCols)
144       : Impl(xpr, startRow, startCol, blockRows, blockCols)
145     {
146       eigen_assert((RowsAtCompileTime==Dynamic || RowsAtCompileTime==blockRows)
147           && (ColsAtCompileTime==Dynamic || ColsAtCompileTime==blockCols));
148       eigen_assert(startRow >= 0 && blockRows >= 0 && startRow  <= xpr.rows() - blockRows
149           && startCol >= 0 && blockCols >= 0 && startCol <= xpr.cols() - blockCols);
150     }
*/

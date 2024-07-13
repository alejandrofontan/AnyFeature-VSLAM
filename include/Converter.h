/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>

#include "g2o/types/types_six_dof_expmap.h"
#include "g2o/types/types_seven_dof_expmap.h"

#include "Types.h"
#include "Definitions.h"

namespace ANYFEATURE_VSLAM
{

class Converter
{
public:
    static std::vector<cv::Mat> toDescriptorVector_mat(const cv::Mat &Descriptors);
    static std::vector<std::vector<float>> toDescriptorVector_float(const cv::Mat &Descriptors);

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);

    static Eigen::Matrix<float,4,4> toMatrix4f(const cv::Mat &cvMat4);
    static Eigen::Matrix<float,3,3> toMatrix3f(const cv::Mat &cvMat3);
    static Eigen::Matrix<float,3,1> toVector3f(const cv::Mat &cvVector);
    static Eigen::Matrix<float,4,4> toMatrix4f(const g2o::SE3Quat &SE3);

    static cv::Mat toCvMat(const Eigen::Matrix4f &m);
    static cv::Mat toCvMat(const Eigen::Matrix3f &m);
    static cv::Mat toCvMat(const Eigen::Vector3f &m);

    static mat4f toMatrix4f(const mat3 &R, const vec3 &t);
    static g2o::SE3Quat toSE3Quat(const mat4f &m);
    static mat4f toMatrix4f(const g2o::Sim3 &Sim3);


};

}// namespace ORB_SLAM

#endif // CONVERTER_H

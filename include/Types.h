//
// Created by fontan on 15/02/24.
//

#ifndef ANYFEATURE_VSLAM_TYPES_H
#define ANYFEATURE_VSLAM_TYPES_H

#include <Eigen/Dense>
#include <iostream>

enum KeypointType {
    KEYP_ANYFEATNONBIN = 8,
    KEYP_ANYFEATBIN = 7,
    KEYP_R2D2 = 6,
    KEYP_SIFT = 5,
    KEYP_KAZE = 4,
    KEYP_SURF = 3,
    KEYP_BRISK = 2,
    KEYP_AKAZE = 1,
    KEYP_ORB = 0,
};

enum DescriptorType {
    DESC_ANYFEATNONBIN = 8,
    DESC_ANYFEATBIN = 7,
    DESC_R2D2 = 6,
    DESC_SIFT128 = 5,
    DESC_KAZE64 = 4,
    DESC_SURF64 = 3,
    DESC_BRISK = 2,
    DESC_AKAZE61 = 1,
    DESC_ORB = 0,
};

enum FeatureType {
    FEAT_ANYFEATNONBIN = 8,
    FEAT_ANYFEATBIN = 7,
    FEAT_R2D2 = 6,
    FEAT_SIFT128 = 5,
    FEAT_KAZE64 = 4,
    FEAT_SURF64 = 3,
    FEAT_BRISK = 2,
    FEAT_AKAZE61 = 1,
    FEAT_ORB = 0,
};

inline KeypointType GetKeypointType(const FeatureType& featureType){
    switch (featureType) { // GetKeypointType
        case FEAT_ANYFEATNONBIN:
            return KEYP_ANYFEATNONBIN;
        case FEAT_ANYFEATBIN:
            return KEYP_ANYFEATBIN;
        case FEAT_R2D2:
            return KEYP_R2D2;
        case FEAT_SIFT128:
            return KEYP_SIFT;
        case FEAT_ORB:
            return KEYP_ORB;
        case FEAT_AKAZE61:
            return KEYP_AKAZE;
        case FEAT_BRISK:
            return KEYP_BRISK;
        case FEAT_SURF64:
            return KEYP_SURF;
        case FEAT_KAZE64:
            return KEYP_KAZE;
        default:{
            std::cout <<"GetKeypointType"<< std::endl;
            std::terminate();
        }

    }
}

inline DescriptorType GetDescriptorType(const FeatureType& featureType){
    switch (featureType) { // GetDescriptorType
        case FEAT_ANYFEATNONBIN:
            return DESC_ANYFEATNONBIN;
        case FEAT_ANYFEATBIN:
            return DESC_ANYFEATBIN;
        case FEAT_R2D2:
            return DESC_R2D2;
        case FEAT_SIFT128:
            return DESC_SIFT128;
        case FEAT_ORB:
            return DESC_ORB;
        case FEAT_AKAZE61:
            return DESC_AKAZE61;
        case FEAT_BRISK:
            return DESC_BRISK;
        case FEAT_SURF64:
            return DESC_SURF64;
        case FEAT_KAZE64:
            return DESC_KAZE64;
        default:{
            std::cout <<"GetDescriptorType"<< std::endl;
            std::terminate();
        }
    }
}

inline int get_feature_id(const std::string& str) {
    if (str == "orb32") {
        return 0;
    } else if (str == "akaze61") {
        return 1;
    } else if (str == "brisk48") {
        return 2;
    } else if (str == "surf64") {
        return 3;
    } else if (str == "kaze64") {
        return 4;
    } else if (str == "sift128") {
        return 5;
    } else if (str == "r2d2_128") {
        return 6;
    } else if (str == "anyfeatbin") {
        return 7;
    } else if (str == "anyfeatnonbin") {
        return 8;
    } else {
        return 0;
    }
}

//typedef int Descriptor_Distance_Type;
typedef float Descriptor_Distance_Type;

using KeyframeId = long unsigned int;
using FrameId = long unsigned int;
using PtId = long unsigned int;
using KeypointIndex = int;

typedef Eigen::Matrix<double,4,4> mat4;
typedef Eigen::Matrix<double,3,3> mat3;
typedef Eigen::Matrix<double,2,2> mat2;
typedef Eigen::Matrix<float,4,4> mat4f;
typedef Eigen::Matrix<float,3,3> mat3f;
typedef Eigen::Matrix<float,2,2> mat2f;

typedef Eigen::Matrix<double,4,1> vec4;
typedef Eigen::Matrix<double,3,1> vec3;
typedef Eigen::Matrix<float,4,1> vec4f;
typedef Eigen::Matrix<float,3,1> vec3f;

typedef Eigen::Matrix<float,2,1> vec2f;

typedef Eigen::Matrix<float,3,4> mat34f;


enum VerbosityLevel{
    NONE = 0,
    LOW = 1,
    MEDIUM = 2,
    HIGH = 3,
    ABLATION = 4
};

#endif //ANYFEATURE_VSLAM_TYPES_H

//
// Created by fontan on 20/02/24.
//

#include <fstream>
#include "Utils.h"

std::random_device rd;
std::mt19937 ANYFEATURE_VSLAM::RandomIntegerGenerator::randomIntGenerator{std::mt19937(rd())};

void ANYFEATURE_VSLAM::printInfo(const std::string& function, const std::string& message,
                          const VerbosityLevel& verbosityLevel, const VerbosityLevel& verbosityLevelRequired,
                          std::string color){
    if(verbosityLevel >= verbosityLevelRequired)
        std::cout << color << "[" << function << "] : " <<  message << "\x1b[0m"<< std::endl;
}

void ANYFEATURE_VSLAM::printError(const std::string& function, const std::string& message){
    std::cout << "\x1b[91m" << "[" << function << "] : " <<  message << "\x1b[0m"<< std::endl;
}

std::string ANYFEATURE_VSLAM::keypointName(const KeypointType & keypointType){
    switch (keypointType) {
        // keypoint name
        case KEYP_ANYFEATNONBIN:{
            return "AnyFeatNonBin";
        }
        case KEYP_ANYFEATBIN:{
            return "AnyFeatBin";
        }
        case KEYP_R2D2:{
            return "R2d2";
        }
        case KEYP_SIFT:{
            return "Sift";
        }
        case KEYP_KAZE:{
            return "Kaze";
        }
        case KEYP_SURF:{
            return "Surf";
        }
        case KEYP_BRISK:{
            return "Brisk";
        }
        case KEYP_AKAZE:{
            return "Akaze";
        }
        case KEYP_ORB:{
            return "Orb";
        }
    }
}
std::string ANYFEATURE_VSLAM::descriptorName(const DescriptorType& descriptorType){
    switch (descriptorType) {
        // descriptor name
        case DESC_ANYFEATNONBIN:{
            return "AnyFeatNonBin";
        }
        case DESC_ANYFEATBIN:{
            return "AnyFeatBin";
        }
        case DESC_R2D2:{
            return "R2d2";
        }
        case DESC_SIFT128:{
            return "Sift128";
        }
        case DESC_KAZE64:{
            return "Kaze64";
        }
        case DESC_SURF64:{
            return "Surf64";
        }
        case DESC_BRISK:{
            return "Brisk";
        }
        case DESC_AKAZE61:{
            return "Akaze61";
        }
        case DESC_ORB:{
            return "Orb";
        }
    }
}

std::string ANYFEATURE_VSLAM::featureName(const FeatureType& featureType){
    switch (featureType) {
        // feature name
        case FEAT_ANYFEATNONBIN:{
            return "anyFeatNonBin";
        }
        case FEAT_ANYFEATBIN:{
            return "anyFeatBin";
        }
        case FEAT_R2D2:{
            return "r2d2";
        }
        case FEAT_SIFT128:{
            return "sift128";
        }
        case FEAT_KAZE64:{
            return "Kaze64";
        }
        case FEAT_SURF64:{
            return "Surf64";
        }
        case FEAT_BRISK:{
            return "Brisk";
        }
        case FEAT_AKAZE61:{
            return "Akaze61";
        }
        case FEAT_ORB:{
            return "Orb";
        }
    }
}

std::string ANYFEATURE_VSLAM::matType(const int& matTypeIndex){
    switch(matTypeIndex) {
        case 0:
            return "CV_8U";
        case 1:
            return "CV_8S";
        case 2:
            return "CV_16U";
        case 3:
            return "CV_16S";
        case 4:
            return "CV_32S";
        case 5:
            return "CV_32F";
        case 6:
            return "CV_64F ";
    }
}

cv::Scalar ANYFEATURE_VSLAM::getFeatureColor(const FeatureType& featureType, const int& format){
    Eigen::Matrix<int,3,1> color{0,0,0};
    switch(featureType) {
        // descriptor color
        case FEAT_ANYFEATNONBIN:{
            color << 122 , 255, 122;
            break;
        }
        case FEAT_ANYFEATBIN:{
            color << 255 , 122, 255;
            break;
        }
        case FEAT_R2D2:{
            color << 255 , 0, 255;
            break;
        }
        case FEAT_SIFT128:{
            color << 255 , 0, 0;
            break;
        }
        case FEAT_KAZE64:{
            color << 0, 255, 255;
            break;
        }
        case FEAT_SURF64:{
            color << 0, 0, 255;
            break;
        }
        case FEAT_BRISK:{
            color << 120, 255, 120;
            break;
        }
        case FEAT_AKAZE61:{
            color <<  255 , 255, 0;
            break;
        }
        case FEAT_ORB: {
            color << 0, 255, 0;
            break;
        }
        default:
            return {0,0,0};
    }

    switch(format) {
        case 0:
            return {float(color(0)),float(color(1)),float(color(2))};
        default:
            return {float(color(2)),float(color(1)),float(color(0))};
    }

}

void ANYFEATURE_VSLAM::linearRegression(float& m, float& b, float& R2, const std::vector<float>& x_, const std::vector<float>& y_){
    Eigen::VectorXf x(x_.size());
    for (int i = 0; i < x_.size(); ++i)
        x(i) = x_[i];
    Eigen::VectorXf y(y_.size());
    for (int i = 0; i < y_.size(); ++i)
        y(i) = y_[i];

    Eigen::MatrixXf A(x.size(), 2);
    A.col(0) = x;
    A.col(1) = Eigen::VectorXf::Ones(x.size());

    Eigen::VectorXf theta = (A.transpose() * A).inverse() * A.transpose() * y;
    Eigen::VectorXf y_pred = A * theta;
    float ss_res = (y - y_pred).squaredNorm();
    float ss_tot = (y - y.mean() * Eigen::VectorXf::Ones(y.size())).squaredNorm();

    m = theta(0);
    b = theta(1);
    R2 = float (1.0 - ss_res / ss_tot);
}

std::vector<std::vector<float>> ANYFEATURE_VSLAM::loadBinFile(const std::string& filename, const int& numFloats ){

    std::vector<std::vector<float>> floats{};
    std::vector<double> floatsRow(numFloats);

    std::ifstream binFile(filename, std::ios::binary);
    while (binFile.read(reinterpret_cast<char*>(floatsRow.data()), numFloats * sizeof(double))) {
        std::vector<float> floats_;
        for (double f : floatsRow)
            floats_.push_back(float(f));
        floats.push_back(floats_);
    }
    binFile.close();
    return floats;
}

std::string ANYFEATURE_VSLAM::replaceAllOccurrences(std::string str, const std::string& from, const std::string& to) {
    size_t startPos = 0;
    while ((startPos = str.find(from, startPos)) != std::string::npos) {
        str.replace(startPos, from.length(), to);
        startPos += to.length(); // Handles case when 'to' is a substring of 'from'
    }
    return str;
}

int ANYFEATURE_VSLAM::RandomIntegerGenerator::GetRandomInteger(const int& minNumber, const int& maxNumber){
    std::uniform_int_distribution<> distrib(minNumber, maxNumber);
    return distrib(randomIntGenerator);
}

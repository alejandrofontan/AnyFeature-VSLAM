#include "FeatureExtractor.h"
#include "Utils.h"
#include "MathFunctions.h"

#include <opencv2/core/core.hpp>
#include <vector>

#include <yaml-cpp/yaml.h>

using namespace cv;
using namespace std;

ANYFEATURE_VSLAM::FeatureExtractorSettings::FeatureExtractorSettings(
        const KeypointType& keypointType_, const DescriptorType& descriptorType_,
        const std::string &settingsYamlFile):
        keypointType(keypointType_), descriptorType(descriptorType_){

    YAML::Node settings = YAML::LoadFile(settingsYamlFile);
    nOctaves = settings["FeatureExtractor.nOctaves"].as<int>();
    scaleFactor = settings["FeatureExtractor.scaleFactor"].as<float>();
    detectTh = settings["FeatureExtractor.detectTh"].as<float>();
    maxNumFeatures = settings["FeatureExtractor.maxNumFeatures"].as<int>();
    
    cout << endl  << "Loading Feature Extractor Settings from : " << settingsYamlFile << endl;
    std::cout << "- nOctaves = " << nOctaves << std::endl;
    std::cout << "- scaleFactor = " << scaleFactor << std::endl;
    std::cout << "- detectTh = " << detectTh << std::endl; 
    std::cout << "- maxNumFeatures = " << maxNumFeatures << std::endl;       

    maxKeyPtSize0  = pow(scaleFactorOrb,float(nOctavesOrb - 1.0));
    maxKeyPtSigma0 = pow(scaleFactorOrb,float(nOctavesOrb - 1.0));
    maxKeyPtSize = pow(scaleFactorOrb,float(nOctavesOrb - 1.0));
    minKeyPtSize = 1.0f;
}

void ANYFEATURE_VSLAM::FeatureExtractor::operator()(const Image& img,
                                             std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                                             std::vector<mat2f>& keyPtsSigma2, std::vector<mat2f>& keyPtsInf, std::vector<float>& keyPtsSize)
{
    setupImage(img);
    detectAndCompute(img, keypoints, descriptors);
    computeSize(keyPtsSize,keypoints);
    computeSigma(keyPtsSigma2, keyPtsInf,keyPtsSize,keypoints,img,CovarianceMethod::SIZE);
}

void ANYFEATURE_VSLAM::FeatureExtractor::computeSize(std::vector<float>& keyPtsSize, const std::vector<cv::KeyPoint>& keypoints){
    keyPtsSize.clear();
    keyPtsSize.reserve(keypoints.size());
    for(auto& keyPt: keypoints){
        float keyPtSize = GetKeypointSize(keyPt);
        float keyPtSize_norm{settings->maxKeyPtSize};
        if(settings->maxKeyPtSize > settings->minKeyPtSize)
            keyPtSize_norm =  1.0f +  (keyPtSize - settings->minKeyPtSize) * (settings->maxKeyPtSize0 - 1.0f)/(settings->maxKeyPtSize - settings->minKeyPtSize);
        keyPtsSize.push_back(keyPtSize_norm);
    }
}

void ANYFEATURE_VSLAM::FeatureExtractor::computeSigma(std::vector<mat2f>& keyPtsSigma2, std::vector<mat2f>& keyPtsInf,
                                                      const std::vector<float>& keyPtsSize, const std::vector<cv::KeyPoint>& keypoints,
                                                      const Image& img, const CovarianceMethod& method){
    keyPtsSigma2.clear();
    keyPtsInf.clear();
    keyPtsSigma2.reserve(keypoints.size());
    keyPtsInf.reserve(keypoints.size());
    switch (method) {
        case NONE:{
            for(const auto& keypoint: keypoints){
                keyPtsSigma2.emplace_back(mat2f::Identity());
                keyPtsInf.emplace_back(mat2f::Identity());
            }
            return;
        }
        case SIZE:{
            int iKeyPt{0};
            for(const auto& keypoint: keypoints){
                float keypointSigma = keyPtsSize[iKeyPt];
                float keypointSigma2 = keypointSigma * keypointSigma;
                float keypointInf = 1.0f/(keypointSigma2);
                keyPtsSigma2.emplace_back(keypointSigma2 * mat2f::Identity());
                keyPtsInf.emplace_back(keypointInf * mat2f::Identity());
                iKeyPt++;
            }
            return;
        }
    }
}
//
// Created by fontan on 7/06/24.
//

#include "Feature_surf64.h"

#include <opencv2/core/core.hpp>
#include <opencv2/xfeatures2d.hpp>

ANYFEATURE_VSLAM::FeatureExtractor_surf64::FeatureExtractor_surf64(const int &nfeatures_, std::shared_ptr<FeatureExtractorSettings> &settings_):
        FeatureExtractor(nfeatures_,settings_){
}

void ANYFEATURE_VSLAM::FeatureExtractor_surf64::detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){
    std::map<int,std::vector<cv::KeyPoint>> keypoints_level;
    std::map<int,cv::Mat> descriptors_level;
    detectKeypoints(keypoints_level, img, settings->detectTh, settings->nOctaves);
    filterKeypoints(keypoints_level, img.grayImg, img.mask);
    computeDescriptors(descriptors_level,keypoints_level,img);
    mergeKeypointLevels(keypoints,descriptors,descriptors_level,keypoints_level);
}

void ANYFEATURE_VSLAM::FeatureExtractor_surf64::initializeExtractor(const Image& img){
}

void ANYFEATURE_VSLAM::FeatureExtractor_surf64::detectKeypoints(
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img, const float& detectTh, const int& nOctaves) const{

    cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create();
    surf->setHessianThreshold(detectTh);
    std::vector<cv::KeyPoint> keypoints{};
    surf->detect(img.grayImg, keypoints);
    for(auto& keyPt: keypoints)
        keypoints_level[GetKeypointOctave(keyPt)].push_back(keyPt);

}

void ANYFEATURE_VSLAM::FeatureExtractor_surf64::computeDescriptors(
        std::map<int,cv::Mat>& descriptors_level,
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img) const {


    cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create();
    //surf->setHessianThreshold(detectTh);

    for(auto& [level,keypoints]: keypoints_level)
        surf->compute(img.grayImg, keypoints, descriptors_level[level]);
}

int ANYFEATURE_VSLAM::FeatureExtractor_surf64::GetKeypointOctave(const cv::KeyPoint& keypoint) const{
    return keypoint.octave;
}

float ANYFEATURE_VSLAM::FeatureExtractor_surf64::GetKeypointSize(const cv::KeyPoint& keypoint) const{
    return powf(settings->GetDetectorNominalScaleFactor(), float(GetKeypointOctave(keypoint)));
    //return (keypoint.size / 9.0f);
}

void ANYFEATURE_VSLAM::FeatureExtractor_surf64::filterKeypoints(std::map<int,std::vector<cv::KeyPoint>>& keypoints_level, const cv::Mat& image, const cv::Mat& mask) const{
    FeatureExtractor::filterKeypoints_notScaled(keypoints_level,image,mask);
}

float ANYFEATURE_VSLAM::DescriptorDistance_surf64(const cv::Mat &a, const cv::Mat &b){
    return (Descriptor_Distance_Type) cv::norm(a,b,cv::NORM_L2SQR);
}

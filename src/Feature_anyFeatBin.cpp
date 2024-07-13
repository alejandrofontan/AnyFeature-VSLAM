//
// Created by fontan on 7/06/24.
//

#include "Feature_anyFeatBin.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

ANYFEATURE_VSLAM::FeatureExtractor_anyFeatBin::FeatureExtractor_anyFeatBin(const int &nfeatures_, std::shared_ptr<FeatureExtractorSettings> &settings_):
        FeatureExtractor(nfeatures_,settings_){
}

void ANYFEATURE_VSLAM::FeatureExtractor_anyFeatBin::detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){
    std::map<int,std::vector<cv::KeyPoint>> keypoints_level;
    std::map<int,cv::Mat> descriptors_level;
    detectKeypoints(keypoints_level, img, settings->detectTh, settings->nOctaves);
    filterKeypoints(keypoints_level, img.grayImg, img.mask);
    computeDescriptors(descriptors_level,keypoints_level,img);
    scaleKeypoints(keypoints_level);
    mergeKeypointLevels(keypoints,descriptors,descriptors_level,keypoints_level);
}

void ANYFEATURE_VSLAM::FeatureExtractor_anyFeatBin::initializeExtractor(const Image& img){
    ComputePyramid(img.grayImg);
}

void ANYFEATURE_VSLAM::FeatureExtractor_anyFeatBin::detectKeypoints(
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img, const float& detectTh, const int& nOctaves) const{

    for(int level{0}; level < nOctaves; level++){
        cv::Ptr<cv::AgastFeatureDetector> agast = cv::AgastFeatureDetector::create();
        //agast->setThreshold(detectTh);
        //agast->setNonmaxSuppression(1);
        agast->detect(mvImagePyramid[level], keypoints_level[level]);
    }
}

void ANYFEATURE_VSLAM::FeatureExtractor_anyFeatBin::computeDescriptors(
        std::map<int,cv::Mat>& descriptors_level,
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img) const {

    for(int level{0}; level < settings->nOctaves; level++){
        cv::Ptr<cv::xfeatures2d::FREAK> freak = cv::xfeatures2d::FREAK::create();
        //agast->setThreshold(detectTh);
        //agast->setNonmaxSuppression(1);
        freak->compute(mvImagePyramid[level], keypoints_level[level], descriptors_level[level]);
    }
}

int ANYFEATURE_VSLAM::FeatureExtractor_anyFeatBin::GetKeypointOctave(const cv::KeyPoint& keypoint) const{
    return keypoint.octave;
}

float ANYFEATURE_VSLAM::FeatureExtractor_anyFeatBin::GetKeypointSize(const cv::KeyPoint& keypoint) const{
    return powf(settings->GetDetectorNominalScaleFactor(), float(GetKeypointOctave(keypoint)));
}

void ANYFEATURE_VSLAM::FeatureExtractor_anyFeatBin::filterKeypoints(std::map<int,std::vector<cv::KeyPoint>>& keypoints_level, const cv::Mat& image, const cv::Mat& mask) const{
    FeatureExtractor::filterKeypoints_scaled(keypoints_level,image,mask);
}

float ANYFEATURE_VSLAM::DescriptorDistance_anyFeatureBin(const cv::Mat &a, const cv::Mat &b){
    return (Descriptor_Distance_Type) cv::norm(a,b,cv::NORM_HAMMING);
}
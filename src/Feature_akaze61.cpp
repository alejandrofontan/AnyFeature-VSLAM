//
// Created by fontan on 7/06/24.
//

#include "Feature_akaze61.h"

ANYFEATURE_VSLAM::FeatureExtractor_akaze61::FeatureExtractor_akaze61(const int &nfeatures_, std::shared_ptr<FeatureExtractorSettings> &settings_):
        FeatureExtractor(nfeatures_,settings_){
    akazeOptions = AKAZEOptions();
    akazeOptions.omax = settings->GetDetectorNominalNumOctaves() / 4;
    akazeOptions.nsublevels = settings->GetDetectorNominalNumOctaves() / 2;
    akazeOptions.dthreshold = float(settings->detectTh);
}

void ANYFEATURE_VSLAM::FeatureExtractor_akaze61::detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){
    std::map<int,std::vector<cv::KeyPoint>> keypoints_level;
    std::map<int,cv::Mat> descriptors_level;
    detectKeypoints(keypoints_level, img, settings->detectTh, settings->nOctaves);
    filterKeypoints(keypoints_level, img.grayImg, img.mask);
    computeDescriptors(descriptors_level,keypoints_level,img);
    mergeKeypointLevels(keypoints,descriptors,descriptors_level,keypoints_level);
}

void ANYFEATURE_VSLAM::FeatureExtractor_akaze61::initializeExtractor(const Image& img){
    cv::Mat img_32;
    img.grayImg.convertTo(img_32, CV_32F, 1.0/255.0,0);
    akazeOptions.img_width = img_32.cols;
    akazeOptions.img_height = img_32.rows;

    evolution.reset();
    evolution = std::make_shared<libAKAZE::AKAZE>(akazeOptions);
    evolution->Create_Nonlinear_Scale_Space(img_32);
}

void ANYFEATURE_VSLAM::FeatureExtractor_akaze61::detectKeypoints(
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img, const float& detectTh, const int& nOctaves) const{

    std::vector<cv::KeyPoint> keypoints{};
    evolution->Feature_Detection(keypoints);

    for(auto& keyPt: keypoints)
        keypoints_level[GetKeypointOctave(keyPt)].push_back(keyPt);
}

void ANYFEATURE_VSLAM::FeatureExtractor_akaze61::computeDescriptors(
        std::map<int,cv::Mat>& descriptors_level,
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img) const {

    descriptors_level.clear();

    std::vector<cv::KeyPoint> keypoints_0;
    for(auto& [level,keypoints]: keypoints_level)
        keypoints_0.insert(keypoints_0.end(),keypoints.begin(),keypoints.end());

    keypoints_level.clear();
    keypoints_level[0] = keypoints_0;

    evolution->Compute_Descriptors(keypoints_level[0],  descriptors_level[0]);
}

int ANYFEATURE_VSLAM::FeatureExtractor_akaze61::GetKeypointOctave(const cv::KeyPoint& keypoint) const{
    return keypoint.class_id;
}

float ANYFEATURE_VSLAM::FeatureExtractor_akaze61::GetKeypointSize(const cv::KeyPoint& keypoint) const{
    return powf(settings->GetDetectorNominalScaleFactor(), float(GetKeypointOctave(keypoint)));
}

void ANYFEATURE_VSLAM::FeatureExtractor_akaze61::filterKeypoints(std::map<int,std::vector<cv::KeyPoint>>& keypoints_level, const cv::Mat& image, const cv::Mat& mask) const{
    FeatureExtractor::filterKeypoints_notScaled(keypoints_level,image,mask);
}

float ANYFEATURE_VSLAM::DescriptorDistance_akaze61(const cv::Mat &a, const cv::Mat &b){
    return (Descriptor_Distance_Type) cv::norm(a,b,cv::NORM_HAMMING);
}
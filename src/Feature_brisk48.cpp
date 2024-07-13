//
// Created by fontan on 7/06/24.
//

#include "Feature_brisk48.h"

ANYFEATURE_VSLAM::FeatureExtractor_brisk48::FeatureExtractor_brisk48(const int &nfeatures_, std::shared_ptr<FeatureExtractorSettings> &settings_):
        FeatureExtractor(nfeatures_,settings_){
}

void ANYFEATURE_VSLAM::FeatureExtractor_brisk48::detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){
    std::map<int,std::vector<cv::KeyPoint>> keypoints_level;
    std::map<int,cv::Mat> descriptors_level;
    detectKeypoints(keypoints_level, img, settings->detectTh, settings->nOctaves);
    filterKeypoints(keypoints_level, img.grayImg, img.mask);
    computeDescriptors(descriptors_level,keypoints_level,img);
    mergeKeypointLevels(keypoints,descriptors,descriptors_level,keypoints_level);
}

void ANYFEATURE_VSLAM::FeatureExtractor_brisk48::detectKeypoints(
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img, const float& detectTh, const int& nOctaves) const{

    const bool suppressScaleNonmaxima = true;
    cv::Ptr<cv::FeatureDetector> brisk = new brisk::BriskFeatureDetector(int(detectTh), nOctaves / 2, suppressScaleNonmaxima);

    std::vector<cv::KeyPoint> keypoints{};
    brisk->detect(img.grayImg, keypoints);
    for(auto& keyPt: keypoints)
        keypoints_level[GetKeypointOctave(keyPt)].push_back(keyPt);
}

void ANYFEATURE_VSLAM::FeatureExtractor_brisk48::computeDescriptors(
        std::map<int,cv::Mat>& descriptors_level,
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img) const {

    std::vector<cv::KeyPoint> keypoints_0;
    for(auto& [level,keypoints]: keypoints_level)
        keypoints_0.insert(keypoints_0.end(),keypoints.begin(),keypoints.end());
    keypoints_level.clear();
    keypoints_level[0] = keypoints_0;

    cv::Ptr<cv::DescriptorExtractor> brisk = new brisk::BriskDescriptorExtractor(
            true, true, brisk::BriskDescriptorExtractor::Version::briskV2);

    brisk->compute(img.grayImg, keypoints_level[0], descriptors_level[0]);
}

int ANYFEATURE_VSLAM::FeatureExtractor_brisk48::GetKeypointOctave(const cv::KeyPoint& keypoint) const{
    return keypoint.octave;
}

float ANYFEATURE_VSLAM::FeatureExtractor_brisk48::GetKeypointSize(const cv::KeyPoint& keypoint) const{
    return powf(settings->GetDetectorNominalScaleFactor(), float(GetKeypointOctave(keypoint)));
}

void ANYFEATURE_VSLAM::FeatureExtractor_brisk48::filterKeypoints(std::map<int,std::vector<cv::KeyPoint>>& keypoints_level, const cv::Mat& image, const cv::Mat& mask) const{
    FeatureExtractor::filterKeypoints_notScaled(keypoints_level,image,mask);
}

float ANYFEATURE_VSLAM::DescriptorDistance_brisk48(const cv::Mat &a, const cv::Mat &b){
    return (Descriptor_Distance_Type) cv::norm(a,b,cv::NORM_HAMMING);
}

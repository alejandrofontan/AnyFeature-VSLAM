//
// Created by fontan on 7/06/24.
//

#include "Feature_kaze64.h"

ANYFEATURE_VSLAM::FeatureExtractor_kaze64::FeatureExtractor_kaze64(const int &nfeatures_, std::shared_ptr<FeatureExtractorSettings> &settings_):
        FeatureExtractor(nfeatures_,settings_){
}

void ANYFEATURE_VSLAM::FeatureExtractor_kaze64::detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){
    std::map<int,std::vector<cv::KeyPoint>> keypoints_level;
    std::map<int,cv::Mat> descriptors_level;
    detectKeypoints(keypoints_level, img, settings->detectTh, settings->nOctaves);
    filterKeypoints(keypoints_level, img.grayImg, img.mask);
    computeDescriptors(descriptors_level,keypoints_level,img);
    mergeKeypointLevels(keypoints,descriptors,descriptors_level,keypoints_level);
}

void ANYFEATURE_VSLAM::FeatureExtractor_kaze64::detectKeypoints(
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img, const float& detectTh, const int& nOctaves) const{

    cv::Ptr<cv::KAZE> kaze = cv::KAZE::create();
    kaze->setNOctaves(nOctaves/4);
    kaze->setNOctaveLayers(nOctaves/2);
    kaze->setThreshold(detectTh);

    std::vector<cv::KeyPoint> keypoints{};
    kaze->detect(img.grayImg, keypoints);
    for(auto& keyPt: keypoints)
        keypoints_level[GetKeypointOctave(keyPt)].push_back(keyPt);

}

void ANYFEATURE_VSLAM::FeatureExtractor_kaze64::computeDescriptors(
        std::map<int,cv::Mat>& descriptors_level,
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img) const {

    std::vector<cv::KeyPoint> keypoints_0;
    for(auto& [level,keypoints]: keypoints_level)
        keypoints_0.insert(keypoints_0.end(),keypoints.begin(),keypoints.end());
    keypoints_level.clear();
    keypoints_level[0] = keypoints_0;

    cv::Ptr<cv::KAZE> kaze64 = cv::KAZE::create();
    kaze64->setThreshold(settings->detectTh);

    kaze64->compute(img.grayImg, keypoints_level[0], descriptors_level[0]);
}

int ANYFEATURE_VSLAM::FeatureExtractor_kaze64::GetKeypointOctave(const cv::KeyPoint& keypoint) const{
    return keypoint.class_id;
}

float ANYFEATURE_VSLAM::FeatureExtractor_kaze64::GetKeypointSize(const cv::KeyPoint& keypoint) const{
    return powf(settings->GetDetectorNominalScaleFactor(), float(GetKeypointOctave(keypoint)));
}

void ANYFEATURE_VSLAM::FeatureExtractor_kaze64::filterKeypoints(std::map<int,std::vector<cv::KeyPoint>>& keypoints_level, const cv::Mat& image, const cv::Mat& mask) const{
    FeatureExtractor::filterKeypoints_notScaled(keypoints_level,image,mask);
}

float ANYFEATURE_VSLAM::DescriptorDistance_kaze64(const cv::Mat &a, const cv::Mat &b){
    return (Descriptor_Distance_Type) cv::norm(a,b,cv::NORM_L2SQR);
}

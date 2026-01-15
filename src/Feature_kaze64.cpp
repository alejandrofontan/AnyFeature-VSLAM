#include "Feature_kaze64.h"

ANYFEATURE_VSLAM::FeatureExtractor_kaze64::FeatureExtractor_kaze64(std::shared_ptr<FeatureExtractorSettings> &settings_):
        FeatureExtractor(settings_){

        kaze = cv::KAZE::create();
        kaze->setNOctaves(settings->nOctaves/4);
        kaze->setNOctaveLayers(settings->nOctaves/2);
        kaze->setThreshold(settings->detectTh);
}

void ANYFEATURE_VSLAM::FeatureExtractor_kaze64::detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){
    kaze->detect(img.grayImg, keypoints);
    kaze->compute(img.grayImg, keypoints, descriptors);
}


int ANYFEATURE_VSLAM::FeatureExtractor_kaze64::GetKeypointOctave(const cv::KeyPoint& keypoint) const{
    return keypoint.class_id;
}

float ANYFEATURE_VSLAM::FeatureExtractor_kaze64::GetKeypointSize(const cv::KeyPoint& keypoint) const{
    return powf(settings->GetDetectorScaleFactor(), float(GetKeypointOctave(keypoint)));
}


float ANYFEATURE_VSLAM::DescriptorDistance_kaze64(const cv::Mat &a, const cv::Mat &b){
    return (Descriptor_Distance_Type) cv::norm(a,b,cv::NORM_L2);
}

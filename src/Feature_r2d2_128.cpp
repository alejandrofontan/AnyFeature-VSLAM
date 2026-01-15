#include "Feature_r2d2_128.h"
#include "Utils.h"

ANYFEATURE_VSLAM::FeatureExtractor_r2d2_128::FeatureExtractor_r2d2_128(std::shared_ptr<FeatureExtractorSettings> &settings_):
        FeatureExtractor(settings_){
}

void ANYFEATURE_VSLAM::FeatureExtractor_r2d2_128::detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){}




int ANYFEATURE_VSLAM::FeatureExtractor_r2d2_128::GetKeypointOctave(const cv::KeyPoint& keypoint) const{
    return 0;
}

float ANYFEATURE_VSLAM::FeatureExtractor_r2d2_128::GetKeypointSize(const cv::KeyPoint& keypoint) const{
    return 1.0;
}

float ANYFEATURE_VSLAM::DescriptorDistance_r2d2_128(const cv::Mat &a, const cv::Mat &b){
    return (Descriptor_Distance_Type) cv::norm(a,b,cv::NORM_L2);
}

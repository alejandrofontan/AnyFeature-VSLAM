#include "include/Utils.h"
#include "Feature_anyFeatNonBin.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

ANYFEATURE_VSLAM::FeatureExtractor_anyFeatNonBin::FeatureExtractor_anyFeatNonBin(std::shared_ptr<FeatureExtractorSettings> &settings_):
        FeatureExtractor(settings_){
}

void ANYFEATURE_VSLAM::FeatureExtractor_anyFeatNonBin::detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){}

int ANYFEATURE_VSLAM::FeatureExtractor_anyFeatNonBin::GetKeypointOctave(const cv::KeyPoint& keypoint) const{
    return keypoint.octave;
}

float ANYFEATURE_VSLAM::FeatureExtractor_anyFeatNonBin::GetKeypointSize(const cv::KeyPoint& keypoint) const{
    return powf(settings->GetDetectorScaleFactor(), float(GetKeypointOctave(keypoint)));
}


float ANYFEATURE_VSLAM::DescriptorDistance_anyFeatureNonBin(const cv::Mat &a, const cv::Mat &b){
    return (Descriptor_Distance_Type) cv::norm(a,b,cv::NORM_L2SQR);
}
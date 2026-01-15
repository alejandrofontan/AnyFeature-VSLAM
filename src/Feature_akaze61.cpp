#include "Feature_akaze61.h"

ANYFEATURE_VSLAM::FeatureExtractor_akaze61::FeatureExtractor_akaze61(std::shared_ptr<FeatureExtractorSettings> &settings_):
        FeatureExtractor(settings_){

    akazeOptions = AKAZEOptions();
    akazeOptions.omax = settings->GetDetectorNumOctaves() / 4;
    akazeOptions.nsublevels = settings->GetDetectorNumOctaves() / 2;
    akazeOptions.dthreshold = float(settings->detectTh);
}

void ANYFEATURE_VSLAM::FeatureExtractor_akaze61::detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){
    evolution->Feature_Detection(keypoints);
    evolution->Compute_Descriptors(keypoints, descriptors);
    if ((keypoints.size() > 1.05 * settings->maxNumFeatures) && (akazeOptions.dthreshold < 10.0 * settings->detectTh))
        akazeOptions.dthreshold += (10.0 - 0.1f) * akazeOptions.dthreshold * 0.05f;
    if ((keypoints.size() < 0.95 * settings->maxNumFeatures) && (akazeOptions.dthreshold > 0.1 * settings->detectTh))
        akazeOptions.dthreshold -= (10.0 - 0.1f) * akazeOptions.dthreshold * 0.05f;
}

void ANYFEATURE_VSLAM::FeatureExtractor_akaze61::setupImage(const Image& img){
    cv::Mat img_32;
    img.grayImg.convertTo(img_32, CV_32F, 1.0/255.0,0);
    akazeOptions.img_width = img_32.cols;
    akazeOptions.img_height = img_32.rows;

    evolution.reset();
    evolution = std::make_shared<libAKAZE::AKAZE>(akazeOptions);
    evolution->Create_Nonlinear_Scale_Space(img_32);
}

int ANYFEATURE_VSLAM::FeatureExtractor_akaze61::GetKeypointOctave(const cv::KeyPoint& keypoint) const{
    return keypoint.class_id;
}

float ANYFEATURE_VSLAM::FeatureExtractor_akaze61::GetKeypointSize(const cv::KeyPoint& keypoint) const{
    return powf(settings->GetDetectorScaleFactor(), float(GetKeypointOctave(keypoint)));
}

float ANYFEATURE_VSLAM::DescriptorDistance_akaze61(const cv::Mat &a, const cv::Mat &b){
    return (Descriptor_Distance_Type) cv::norm(a,b,cv::NORM_HAMMING);
}
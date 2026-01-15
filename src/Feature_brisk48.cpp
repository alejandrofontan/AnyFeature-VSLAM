#include "Feature_brisk48.h"

ANYFEATURE_VSLAM::FeatureExtractor_brisk48::FeatureExtractor_brisk48(std::shared_ptr<FeatureExtractorSettings> &settings_):
        FeatureExtractor(settings_){
    
    detectTh = settings->detectTh;
    brisk_detector = new brisk::BriskFeatureDetector(int(settings->detectTh), settings->nOctaves / 2, true);
    brisk_extractor = new brisk::BriskDescriptorExtractor(true, true, brisk::BriskDescriptorExtractor::Version::briskV2);
}

void ANYFEATURE_VSLAM::FeatureExtractor_brisk48::detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){
    brisk_detector->detect(img.grayImg, keypoints);
    brisk_extractor->compute(img.grayImg, keypoints, descriptors);
    if ((keypoints.size() > 1.05 * settings->maxNumFeatures) && (detectTh < 70)){
        detectTh += 1;
        brisk_detector = new brisk::BriskFeatureDetector(int(detectTh), settings->nOctaves / 2, true);
    }
    if ((keypoints.size() < 0.95 * settings->maxNumFeatures) && (detectTh > 5)){
        detectTh -= 1;
        brisk_detector = new brisk::BriskFeatureDetector(int(detectTh), settings->nOctaves / 2, true);
    }
}

int ANYFEATURE_VSLAM::FeatureExtractor_brisk48::GetKeypointOctave(const cv::KeyPoint& keypoint) const{
    return keypoint.octave;
}

float ANYFEATURE_VSLAM::FeatureExtractor_brisk48::GetKeypointSize(const cv::KeyPoint& keypoint) const{
    return powf(settings->GetDetectorScaleFactor(), float(GetKeypointOctave(keypoint)));
}

float ANYFEATURE_VSLAM::DescriptorDistance_brisk48(const cv::Mat &a, const cv::Mat &b){
    return (Descriptor_Distance_Type) cv::norm(a,b,cv::NORM_HAMMING);
}

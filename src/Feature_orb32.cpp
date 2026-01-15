#include "Feature_orb32.h"

ANYFEATURE_VSLAM::FeatureExtractor_orb32::FeatureExtractor_orb32(std::shared_ptr<FeatureExtractorSettings> &settings_):
        FeatureExtractor(settings_){

    orb32_extractor = cv::ORB::create();
    orb32_extractor->setMaxFeatures(settings->maxNumFeatures); 
    orb32_extractor->setScaleFactor(settings->scaleFactor); 
    orb32_extractor->setEdgeThreshold(31);
    orb32_extractor->setFirstLevel(0); 
    orb32_extractor->setWTA_K(2); 
    orb32_extractor->setScoreType(cv::ORB::FAST_SCORE); // Default cv::ORB::HARRIS_SCORE
    orb32_extractor->setPatchSize(31); 
    orb32_extractor->setNLevels(settings->nOctaves); 
    orb32_extractor->setFastThreshold(int(settings->detectTh));
}

void ANYFEATURE_VSLAM::FeatureExtractor_orb32::detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){
    orb32_extractor->detectAndCompute(img.grayImg, cv::noArray(), keypoints, descriptors);
}

int ANYFEATURE_VSLAM::FeatureExtractor_orb32::GetKeypointOctave(const cv::KeyPoint& keypoint) const{
    return keypoint.octave;
}

float ANYFEATURE_VSLAM::FeatureExtractor_orb32::GetKeypointSize(const cv::KeyPoint& keypoint) const{
    return powf(settings->GetDetectorScaleFactor(), float(GetKeypointOctave(keypoint)));
}

float ANYFEATURE_VSLAM::DescriptorDistance_orb32(const cv::Mat &a, const cv::Mat &b){
    // Bit set count operation from
    // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return Descriptor_Distance_Type(dist);
}
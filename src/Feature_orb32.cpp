//
// Created by fontan on 7/06/24.
//

#include "Feature_orb32.h"

ANYFEATURE_VSLAM::FeatureExtractor_orb32::FeatureExtractor_orb32(const int &nfeatures_, std::shared_ptr<FeatureExtractorSettings> &settings_):
        FeatureExtractor(nfeatures_,settings_){
}

void ANYFEATURE_VSLAM::FeatureExtractor_orb32::detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){
    std::map<int,std::vector<cv::KeyPoint>> keypoints_level;
    std::map<int,cv::Mat> descriptors_level;
    detectKeypoints(keypoints_level, img, settings->detectTh, settings->nOctaves);
    filterKeypoints(keypoints_level, img.grayImg, img.mask);
    computeDescriptors(descriptors_level,keypoints_level,img);
    mergeKeypointLevels(keypoints,descriptors,descriptors_level,keypoints_level);
}

void ANYFEATURE_VSLAM::FeatureExtractor_orb32::initializeExtractor(const Image& img) {
    orb32_extractor = cv::ORB::create();
    orb32_extractor->setMaxFeatures(nfeatures * 10);
    orb32_extractor->setEdgeThreshold(0);
}

void ANYFEATURE_VSLAM::FeatureExtractor_orb32::detectKeypoints(
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img, const float& detectTh, const int& nOctaves) const{

    orb32_extractor->setFastThreshold(int(detectTh));
    orb32_extractor->setNLevels(nOctaves);

    std::vector<cv::KeyPoint> keypoints{};
    orb32_extractor->detect(img.grayImg, keypoints);
    for(auto& keyPt: keypoints)
        keypoints_level[GetKeypointOctave(keyPt)].push_back(keyPt);

    /*ANYFEATURE_VSLAM::printInfo("FeatureExtractor", "ComputeKeyPointsAndDescriptors(): # " + keypointName(settings->keypointType)
                                                    + " Keypoints = " + std::to_string(keypoints.size()), settings->verbosity, MEDIUM);*/
}

void ANYFEATURE_VSLAM::FeatureExtractor_orb32::computeDescriptors(
        std::map<int,cv::Mat>& descriptors_level,
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img) const {

    for(auto& [level,keypoints]: keypoints_level)
        orb32_extractor->compute(img.grayImg, keypoints, descriptors_level[level]);

    /*ANYFEATURE_VSLAM::printInfo("ComputeKeyPointsAndDescriptors", "# " + descriptorName(settings->descriptorType) +
                                                                  " Descriptors = " + std::to_string(descriptors.rows) + " x " + std::to_string(descriptors.cols) +
                                                                  " , type " +  matType(descriptors.type()),settings->verbosity, MEDIUM);*/
}

int ANYFEATURE_VSLAM::FeatureExtractor_orb32::GetKeypointOctave(const cv::KeyPoint& keypoint) const{
    return keypoint.octave;
}

float ANYFEATURE_VSLAM::FeatureExtractor_orb32::GetKeypointSize(const cv::KeyPoint& keypoint) const{
    return powf(settings->GetDetectorNominalScaleFactor(), float(GetKeypointOctave(keypoint)));
}

void ANYFEATURE_VSLAM::FeatureExtractor_orb32::filterKeypoints(std::map<int,std::vector<cv::KeyPoint>>& keypoints_level, const cv::Mat& image, const cv::Mat& mask) const{
    FeatureExtractor::filterKeypoints_notScaled(keypoints_level,image,mask);
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
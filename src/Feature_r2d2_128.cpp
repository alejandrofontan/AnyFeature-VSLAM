//
// Created by fontan on 7/06/24.
//

#include "Feature_r2d2_128.h"
#include "Utils.h"

ANYFEATURE_VSLAM::FeatureExtractor_r2d2_128::FeatureExtractor_r2d2_128(const int &nfeatures_, std::shared_ptr<FeatureExtractorSettings> &settings_):
        FeatureExtractor(nfeatures_,settings_){
}

void ANYFEATURE_VSLAM::FeatureExtractor_r2d2_128::detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){
    std::map<int,std::vector<cv::KeyPoint>> keypoints_level;
    std::map<int,cv::Mat> descriptors_level;
    detectKeypoints(keypoints_level, img, settings->detectTh, settings->nOctaves);
    filterKeypoints(keypoints_level, img.grayImg, img.mask);
    computeDescriptors(descriptors_level,keypoints_level,img);
    mergeKeypointLevels(keypoints,descriptors,descriptors_level,keypoints_level);
}

void ANYFEATURE_VSLAM::FeatureExtractor_r2d2_128::detectKeypoints(
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img, const float& detectTh, const int& nOctaves) const{

    std::vector<std::vector<float>> keypointFloats = loadBinFile(img.keypointBinFile,3);
    std::vector<std::vector<float>> scoreFloats = loadBinFile(img.scoresBinFile,1);
    for(int iKeyPt{0}; iKeyPt < keypointFloats.size(); iKeyPt++){
        std::vector<float> lineScores    =  scoreFloats[iKeyPt];
        std::vector<float> lineKeypoints =  keypointFloats[iKeyPt];
        auto response = float(lineScores[0]);
        cv::KeyPoint keyPt{};
        keyPt.pt.x = float(lineKeypoints[0]);
        keyPt.pt.y = float(lineKeypoints[1]);
        keyPt.size = float(lineKeypoints[2]);
        keyPt.response = response;
        keyPt.class_id = iKeyPt;
        keyPt.angle = 0.0;
        keypoints_level[0].push_back(keyPt);
    }
}

void ANYFEATURE_VSLAM::FeatureExtractor_r2d2_128::computeDescriptors(
        std::map<int,cv::Mat>& descriptors_level,
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img) const {

    const int descSize{128};
    std::vector<std::vector<float>> descriptorFloats = loadBinFile(img.descriptorsBinFile, descSize);
    descriptors_level[0] = cv::Mat::zeros((int) keypoints_level[0].size(), descSize, CV_32F);
    for (size_t jKeyPt = 0; jKeyPt < keypoints_level[0].size(); jKeyPt++)
        for (size_t iDesc = 0; iDesc < descSize; iDesc++)
            descriptors_level[0].at<float>(jKeyPt,iDesc) = (float) descriptorFloats[keypoints_level[0][jKeyPt].class_id][iDesc];

}

int ANYFEATURE_VSLAM::FeatureExtractor_r2d2_128::GetKeypointOctave(const cv::KeyPoint& keypoint) const{
    return 0;
}

float ANYFEATURE_VSLAM::FeatureExtractor_r2d2_128::GetKeypointSize(const cv::KeyPoint& keypoint) const{
    return 1.0;
}

void ANYFEATURE_VSLAM::FeatureExtractor_r2d2_128::filterKeypoints(std::map<int,std::vector<cv::KeyPoint>>& keypoints_level, const cv::Mat& image, const cv::Mat& mask) const{
    FeatureExtractor::filterKeypoints_notScaled(keypoints_level,image,mask);
}

float ANYFEATURE_VSLAM::DescriptorDistance_r2d2_128(const cv::Mat &a, const cv::Mat &b){
    return (Descriptor_Distance_Type) cv::norm(a,b,cv::NORM_L2SQR);
}

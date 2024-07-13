//
// Created by fontan on 7/06/24.
//

#include "Feature_sift128.h"
#include "GL/glew.h"
#include "math.h"

ANYFEATURE_VSLAM::FeatureExtractor_sift128::FeatureExtractor_sift128(const int &nfeatures_, std::shared_ptr<FeatureExtractorSettings> &settings_):
        FeatureExtractor(nfeatures_,settings_){

    std::vector<std::string> sift_gpu_args;
    sift_gpu_args.push_back("./sift_gpu");

    // Darkness adaptivity (hidden feature). Significantly improves
    // distribution of features. Only available in GLSL version.
    sift_gpu_args.push_back("-da");

    // No verbose logging.
    sift_gpu_args.push_back("-v");
    sift_gpu_args.push_back("0");

    // Keep the highest level features.
    sift_gpu_args.push_back("-tc2");
    sift_gpu_args.push_back(std::to_string(nfeatures));

    // First Octave to start detection (default: 0).
    sift_gpu_args.push_back("-fo");
    sift_gpu_args.push_back(std::to_string(0));

    // Maximum number of octaves (default: not limit).
    sift_gpu_args.push_back("-no");
    sift_gpu_args.push_back(std::to_string(8));

    // DOG levels in an octave (default: 3)
    sift_gpu_args.push_back("-d");
    sift_gpu_args.push_back(std::to_string(3));

    // DOG threshold (default: 0.02/3)
    //sift_gpu_args.push_back("-t");
    //sift_gpu_args.push_back(std::to_string(options.peak_threshold));

    // Edge Threshold (default : 10.0)
    sift_gpu_args.push_back("-e");
    sift_gpu_args.push_back(std::to_string(10.0));

    // Let (0, 0) be center of top-left pixel instead of corner with this
    // parameter. The corner is (0, 0) by default, but Lowe‟s SIFT and
    // sift++ are using the pixel center.
    sift_gpu_args.push_back("-loweo");

    std::vector<const char*> sift_gpu_args_cstr;
    sift_gpu_args_cstr.reserve(sift_gpu_args.size());
    for (const auto& arg : sift_gpu_args)
        sift_gpu_args_cstr.push_back(arg.c_str());

    sift = std::make_shared<SiftGPU>();
    sift->ParseParam((int) sift_gpu_args_cstr.size(), const_cast<char **>(sift_gpu_args_cstr.data()));

    int support = sift->CreateContextGL();
    if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED) std::terminate();
}

void ANYFEATURE_VSLAM::FeatureExtractor_sift128::detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){
    std::map<int,std::vector<cv::KeyPoint>> keypoints_level;
    std::map<int,cv::Mat> descriptors_level;
    detectKeypoints(keypoints_level, img, settings->detectTh, settings->nOctaves);
    filterKeypoints(keypoints_level, img.grayImg, img.mask);
    computeDescriptors(descriptors_level,keypoints_level,img);
    mergeKeypointLevels(keypoints,descriptors,descriptors_level,keypoints_level);
}

void ANYFEATURE_VSLAM::FeatureExtractor_sift128::initializeExtractor(const Image& img){
}

void ANYFEATURE_VSLAM::FeatureExtractor_sift128::detectKeypoints(
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img, const float& detectTh, const int& nOctaves) const{

    sift->RunSIFT(img.grayImg.cols,img.grayImg.rows,img.grayImg.data, GL_LUMINANCE, GL_UNSIGNED_BYTE);
    int numKeypoints = sift->GetFeatureNum();
    std::vector<SiftGPU::SiftKeypoint> keys(numKeypoints);
    sift->GetFeatureVector(&keys[0], nullptr);
    int iKey{0};
    for(const auto& key: keys){
        cv::KeyPoint keyPt{};
        keyPt.pt.x = key.x;
        keyPt.pt.y = key.y;
        keyPt.class_id = iKey;
        keyPt.size = key.s;
        keyPt.angle = key.o;
        keyPt.octave = int(log2(key.s / 1.6454));
        keyPt.response = 1.0;
        keypoints_level[keyPt.octave].push_back(keyPt);
        ++iKey;
        //std::cout << keyPt.size << " " << keyPt.octave << " " << keyPt.angle << " " << keyPt.response << " " <<std::endl;
    }
}

void ANYFEATURE_VSLAM::FeatureExtractor_sift128::computeDescriptors(
        std::map<int,cv::Mat>& descriptors_level,
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img) const {

    int numKeypoints = sift->GetFeatureNum();
    std::vector<SiftGPU::SiftKeypoint> keys(numKeypoints);
    std::vector<float> descriptors(128 * numKeypoints);
    sift->GetFeatureVector(nullptr, &descriptors[0]);

    for(auto& [level,keypoints]: keypoints_level){
        descriptors_level[level] = cv::Mat(int(keypoints.size()), 128, CV_32F);
        for (int i = 0; i < keypoints.size(); ++i) {
            for (int j = 0; j < 128; ++j) {
                descriptors_level[level].at<float>(i, j) = descriptors[keypoints[i].class_id * 128 + j];
            }
        }
    }
}

int ANYFEATURE_VSLAM::FeatureExtractor_sift128::GetKeypointOctave(const cv::KeyPoint& keypoint) const{
    return keypoint.octave;
}

float ANYFEATURE_VSLAM::FeatureExtractor_sift128::GetKeypointSize(const cv::KeyPoint& keypoint) const{
    return powf(settings->GetDetectorNominalScaleFactor(), float(GetKeypointOctave(keypoint)));
}

void ANYFEATURE_VSLAM::FeatureExtractor_sift128::filterKeypoints(std::map<int,std::vector<cv::KeyPoint>>& keypoints_level, const cv::Mat& image, const cv::Mat& mask) const{
    FeatureExtractor::filterKeypoints_notScaled(keypoints_level,image,mask);
}

float ANYFEATURE_VSLAM::DescriptorDistance_sift128(const cv::Mat &a, const cv::Mat &b){
    return (Descriptor_Distance_Type) cv::norm(a,b,cv::NORM_L2SQR);
}
//
// Created by fontan on 7/06/24.
//

#include "Feature_aliked128.h"

#include <opencv2/opencv.hpp>
#include <torch/torch.h>

ANYFEATURE_VSLAM::FeatureExtractor_aliked128::FeatureExtractor_aliked128(const int &nfeatures_, std::shared_ptr<FeatureExtractorSettings> &settings_):
        FeatureExtractor(nfeatures_,settings_){
            
        torch::Device device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;    
        extractor = std::make_shared<ALIKED>("aliked-n16", device.str());
}

static cv::Mat tensorDescToMatCopy(const at::Tensor& desc_in) {
    // Move to CPU, contiguous, float32
    at::Tensor desc = desc_in;
    if (desc.is_cuda()) desc = desc.to(at::kCPU);
    desc = desc.contiguous();
    if (desc.scalar_type() != at::kFloat) desc = desc.to(at::kFloat);

    // Handle [1,N,D] -> [N,D]
    if (desc.dim() == 3 && desc.size(0) == 1) {
        desc = desc.squeeze(0);
    }

    TORCH_CHECK(desc.dim() == 2, "Expected descriptors [N,D] or [1,N,D]");
    const int N = (int)desc.size(0);
    const int D = (int)desc.size(1);

    // Create independent cv::Mat and copy
    cv::Mat out(N, D, CV_32F);
    std::memcpy(out.data, desc.data_ptr<float>(), (size_t)N * (size_t)D * sizeof(float));
    return out;
}

void ANYFEATURE_VSLAM::FeatureExtractor_aliked128::detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){
    std::map<int,std::vector<cv::KeyPoint>> keypoints_level;
    std::map<int,cv::Mat> descriptors_level;
    //detectKeypoints(keypoints_level, img, settings->detectTh, settings->nOctaves);
    //filterKeypoints(keypoints_level, img.grayImg, img.mask);
    //computeDescriptors(descriptors_level,keypoints_level,img);
    //torch::Device device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;
    //auto extractor = std::make_shared<ALIKED>("aliked-n16", device.str());
    cv::Mat gray = img.img.clone();   
    auto feats0 = extractor->run(gray);
    const auto& kpts = feats0.at("keypoints");
    //const auto kpts_cpu = kpts.cpu();

    at::Tensor kpts_cpu = kpts.cpu().contiguous().to(at::kFloat);

    TORCH_CHECK(kpts_cpu.dim() == 2 && kpts_cpu.size(1) >= 2, "Expected keypoints [N,2] (or more)");

    int N = (int)kpts_cpu.size(0);
    auto acc = kpts_cpu.accessor<float, 2>();
    int iKey{0};
    for (int i = 0; i < N; ++i) {
        float x = acc[i][0];
        float y = acc[i][1];
        cv::KeyPoint keyPt{};
        keyPt.pt.x = x;
        keyPt.pt.y = y;
        keyPt.class_id = iKey;
        keyPt.size = 1;
        keyPt.angle = 0;
        keyPt.octave = 0;
        keyPt.response = 1.0;
        keypoints_level[keyPt.octave].push_back(keyPt);
        ++iKey;
    }

    const auto& desc_t = feats0.at("descriptors");
    descriptors_level[0] = tensorDescToMatCopy(desc_t);

    mergeKeypointLevels(keypoints,descriptors,descriptors_level,keypoints_level);
}

void ANYFEATURE_VSLAM::FeatureExtractor_aliked128::initializeExtractor(const Image& img){
}

void ANYFEATURE_VSLAM::FeatureExtractor_aliked128::detectKeypoints(
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img, const float& detectTh, const int& nOctaves) const{

    torch::Device device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;
    auto extractor = std::make_shared<ALIKED>("aliked-n16", device.str());
    cv::Mat gray = img.img.clone();   
    auto feats0 = extractor->run(gray);
    const auto& kpts = feats0.at("keypoints");

    at::Tensor kpts_cpu = kpts.cpu().contiguous().to(at::kFloat);

    TORCH_CHECK(kpts_cpu.dim() == 2 && kpts_cpu.size(1) >= 2, "Expected keypoints [N,2] (or more)");

    int N = (int)kpts_cpu.size(0);
    auto acc = kpts_cpu.accessor<float, 2>();
    int iKey{0};
    for (int i = 0; i < N; ++i) {
        float x = acc[i][0];
        float y = acc[i][1];
        cv::KeyPoint keyPt{};
        keyPt.pt.x = x;
        keyPt.pt.y = y;
        keyPt.class_id = iKey;
        keyPt.size = 1;
        keyPt.angle = 0;
        keyPt.octave = 0;
        keyPt.response = 1.0;
        keypoints_level[keyPt.octave].push_back(keyPt);
        ++iKey;
    } 
}

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    FeatureDescriptorsFloat;

void ANYFEATURE_VSLAM::FeatureExtractor_aliked128::computeDescriptors(
        std::map<int,cv::Mat>& descriptors_level,
        std::map<int,std::vector<cv::KeyPoint>>& keypoints_level,
        const Image& img) const {
    return;
}

int ANYFEATURE_VSLAM::FeatureExtractor_aliked128::GetKeypointOctave(const cv::KeyPoint& keypoint) const{
    return keypoint.octave;
}

float ANYFEATURE_VSLAM::FeatureExtractor_aliked128::GetKeypointSize(const cv::KeyPoint& keypoint) const{
    return powf(settings->GetDetectorNominalScaleFactor(), float(GetKeypointOctave(keypoint)));
}

void ANYFEATURE_VSLAM::FeatureExtractor_aliked128::filterKeypoints(std::map<int,std::vector<cv::KeyPoint>>& keypoints_level, const cv::Mat& image, const cv::Mat& mask) const{
    FeatureExtractor::filterKeypoints_notScaled(keypoints_level,image,mask);
}

float ANYFEATURE_VSLAM::DescriptorDistance_aliked128(const cv::Mat &a, const cv::Mat &b){
    return (Descriptor_Distance_Type) cv::norm(a, b, cv::NORM_L2);
}
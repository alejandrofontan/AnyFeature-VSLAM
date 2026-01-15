//
// Created by fontan on 7/06/24.
//

#include "Feature_sift128.h"
#include "GL/glew.h"
#include "math.h"

ANYFEATURE_VSLAM::FeatureExtractor_sift128::FeatureExtractor_sift128(std::shared_ptr<FeatureExtractorSettings> &settings_):
        FeatureExtractor(settings_){

    // settings_->detectTh = (1.0f / settings_->detectTh);

    std::vector<std::string> sift_gpu_args;
    sift_gpu_args.push_back("./sift_gpu");

    // Use CUDA version       
    sift_gpu_args.push_back("-cuda");
    sift_gpu_args.push_back(std::to_string(0));

    // Darkness adaptivity (hidden feature). Significantly improves
    // distribution of features. Only available in GLSL version.
    //sift_gpu_args.push_back("-da");

    // No verbose logging.
    sift_gpu_args.push_back("-v");
    sift_gpu_args.push_back("0");

    // Keep the highest level features.
    sift_gpu_args.push_back("-tc2");
    sift_gpu_args.push_back(std::to_string(settings->maxNumFeatures));

    // First Octave to start detection (default: 0).
    sift_gpu_args.push_back("-fo");
    sift_gpu_args.push_back(std::to_string(-1));

    // Maximum number of octaves (default: not limit).
    sift_gpu_args.push_back("-no");
    sift_gpu_args.push_back(std::to_string(settings->nOctaves));

    // DOG levels in an octave (default: 3)
    sift_gpu_args.push_back("-d");
    sift_gpu_args.push_back(std::to_string(3));

    // DOG threshold (default: 0.02/3)
    sift_gpu_args.push_back("-t");
    sift_gpu_args.push_back(std::to_string(settings->detectTh));

    // Edge Threshold (default : 10.0)
    sift_gpu_args.push_back("-e");
    sift_gpu_args.push_back(std::to_string(10.0));

     // Maximum number of orientations.
    sift_gpu_args.push_back("-mo");
    sift_gpu_args.push_back(std::to_string(2));
    // Let (0, 0) be center of top-left pixel instead of corner with this
    // parameter. The corner is (0, 0) by default, but Lowe‟s SIFT and
    // sift++ are using the pixel center.
    sift_gpu_args.push_back("-loweo");

    std::vector<const char*> sift_gpu_args_cstr;
    sift_gpu_args_cstr.reserve(sift_gpu_args.size());
    for (const auto& arg : sift_gpu_args)
        sift_gpu_args_cstr.push_back(arg.c_str());

    sift = std::make_shared<SiftGPU>();
    sift->ParseParam(sift_gpu_args_cstr.size(), sift_gpu_args_cstr.data());

    int support = sift->CreateContextGL();
    if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED) std::terminate();
}

void ANYFEATURE_VSLAM::FeatureExtractor_sift128::detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){
    detectKeypoints(keypoints, img, settings->detectTh, settings->nOctaves);
    computeDescriptors(descriptors, img);
}

void ANYFEATURE_VSLAM::FeatureExtractor_sift128::detectKeypoints(
        std::vector<cv::KeyPoint>& keypoints,
        const Image& img, const float& detectTh, const int& nOctaves) const{

    sift->RunSIFT(img.grayImg.cols, img.grayImg.rows, img.grayImg.data, GL_LUMINANCE, GL_UNSIGNED_BYTE);
    std::vector<SiftGPU::SiftKeypoint> keys(sift->GetFeatureNum());
    sift->GetFeatureVector(&keys[0], nullptr);
    int iKey{0};
    for(const auto& key: keys){
        cv::KeyPoint keyPt{};
        keyPt.pt.x = key.x;
        keyPt.pt.y = key.y;
        keyPt.class_id = iKey;
        keyPt.size = 1;//key.s;
        keyPt.angle = 0;//key.o;
        keyPt.octave = 0;//int(log2(key.s / 1.6454));
        keyPt.response = 1.0;
        keypoints.push_back(keyPt);
        ++iKey;
        //std::cout << keyPt.size << " " << keyPt.octave << " " << keyPt.angle << " " << keyPt.response << " " <<std::endl;
    }
}

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    FeatureDescriptorsFloat;

void ANYFEATURE_VSLAM::FeatureExtractor_sift128::computeDescriptors(
    cv::Mat& descriptors, const Image& /*img*/) const
{
    const int numKeypoints = sift->GetFeatureNum();
    if (numKeypoints <= 0) {
        descriptors.release();
        return;
    }

    // Allocate output and write SiftGPU descriptors directly into it
    descriptors.create(numKeypoints, 128, CV_32F);
    sift->GetFeatureVector(nullptr, descriptors.ptr<float>());

    // Map cv::Mat memory as an Eigen matrix (row-major to match cv::Mat layout)
    using RowMajorMatXf =
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    Eigen::Map<RowMajorMatXf> D(descriptors.ptr<float>(), numKeypoints, 128);

    constexpr float kEps = 1e-12f;

    // L1 normalize each row, then apply elementwise sqrt (RootSIFT-style)
    for (int i = 0; i < numKeypoints; ++i) {
        const float l1 = D.row(i).lpNorm<1>();
        D.row(i) /= (l1 + kEps);
        D.row(i) = D.row(i).cwiseSqrt();
    }
}

int ANYFEATURE_VSLAM::FeatureExtractor_sift128::GetKeypointOctave(const cv::KeyPoint& keypoint) const{
    return keypoint.octave;
}

float ANYFEATURE_VSLAM::FeatureExtractor_sift128::GetKeypointSize(const cv::KeyPoint& keypoint) const{
    return powf(settings->GetDetectorScaleFactor(), float(GetKeypointOctave(keypoint)));
}

float ANYFEATURE_VSLAM::DescriptorDistance_sift128(const cv::Mat &a, const cv::Mat &b){
    return (Descriptor_Distance_Type) cv::norm(a,b,cv::NORM_L2);
}
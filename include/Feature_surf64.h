//
// Created by fontan on 7/06/24.
//

#ifndef ANYFEATURE_VSLAM_FEATURE_SURF64_H
#define ANYFEATURE_VSLAM_FEATURE_SURF64_H

#include "FeatureExtractor.h"
//#include "opencv2/xfeatures2d/cudafeatures2d.hpp"
#include <opencv2/xfeatures2d/cuda.hpp>

namespace ANYFEATURE_VSLAM {

    class FeatureExtractor_surf64 : public FeatureExtractor {
    public:

        std::shared_ptr<cv::cuda::SURF_CUDA> surf;

        FeatureExtractor_surf64(const int &nfeatures_, std::shared_ptr<FeatureExtractorSettings> &settings_);

        void detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) override;

        void initializeExtractor(const Image& img) override;

        void detectKeypoints(std::map<int, std::vector<cv::KeyPoint>> &keypoints_level, const Image &img,
                             const float &detectTh, const int &nOctaves) const override;

        void computeDescriptors(std::map<int, cv::Mat> &descriptors_level,
                                std::map<int, std::vector<cv::KeyPoint>> &keypoints_level, const Image &img) const override;

        void filterKeypoints(std::map<int,std::vector<cv::KeyPoint>>& keypoints_level, const cv::Mat& image, const cv::Mat& mask) const override;

        [[nodiscard]] int GetKeypointOctave(const cv::KeyPoint &keypoint) const override;
        [[nodiscard]] float GetKeypointSize(const cv::KeyPoint &keypoint) const override;
    };

    float DescriptorDistance_surf64(const cv::Mat &a, const cv::Mat &b);
}


#endif //ANYFEATURE_VSLAM_FEATURE_SURF64_H
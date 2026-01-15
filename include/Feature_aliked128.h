#ifndef ANYFEATURE_VSLAM_FEATURE_ALIKED28_H
#define ANYFEATURE_VSLAM_FEATURE_ALIKED128_H

#include "FeatureExtractor.h"
#include "feature/ALIKED.hpp"

namespace ANYFEATURE_VSLAM {

    class FeatureExtractor_aliked128 : public FeatureExtractor {
    public:

        std::shared_ptr<ALIKED> extractor;

        FeatureExtractor_aliked128(std::shared_ptr<FeatureExtractorSettings> &settings_);
        ~FeatureExtractor_aliked128() {}

        void detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) override;

        [[nodiscard]] int GetKeypointOctave(const cv::KeyPoint &keypoint) const override;
        [[nodiscard]] float GetKeypointSize(const cv::KeyPoint &keypoint) const override;
    };

    float DescriptorDistance_aliked128(const cv::Mat &a, const cv::Mat &b);
    
}

#endif //ANYFEATURE_VSLAM_FEATURE_ALIKED128_H

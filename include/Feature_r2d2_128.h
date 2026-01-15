#ifndef ANYFEATURE_VSLAM_FEATURE_R2D2_128_H
#define ANYFEATURE_VSLAM_FEATURE_R2D2_128_H

#include "FeatureExtractor.h"

namespace ANYFEATURE_VSLAM {

    class FeatureExtractor_r2d2_128 : public FeatureExtractor {
    public:
        FeatureExtractor_r2d2_128(std::shared_ptr<FeatureExtractorSettings> &settings_);

        void detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) override;

        [[nodiscard]] int GetKeypointOctave(const cv::KeyPoint &keypoint) const override;
        [[nodiscard]] float GetKeypointSize(const cv::KeyPoint &keypoint) const override;
    };

    float DescriptorDistance_r2d2_128(const cv::Mat &a, const cv::Mat &b);
}

#endif //ANYFEATURE_VSLAM_FEATURE_R2D2_128_H

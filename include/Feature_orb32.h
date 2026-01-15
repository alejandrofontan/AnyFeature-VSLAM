#ifndef ANYFEATURE_VSLAM_FEATURE_ORB32_H
#define ANYFEATURE_VSLAM_FEATURE_ORB32_H

#include "FeatureExtractor.h"

namespace ANYFEATURE_VSLAM {

    class FeatureExtractor_orb32 : public FeatureExtractor {
    public:

        cv::Ptr<cv::ORB> orb32_extractor;

        FeatureExtractor_orb32(std::shared_ptr<FeatureExtractorSettings> &settings_);

        void detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) override;

        [[nodiscard]] int GetKeypointOctave(const cv::KeyPoint &keypoint) const override;
        [[nodiscard]] float GetKeypointSize(const cv::KeyPoint &keypoint) const override;
    };

    float DescriptorDistance_orb32(const cv::Mat &a, const cv::Mat &b);
}

#endif //ANYFEATURE_VSLAM_FEATURE_ORB32_H

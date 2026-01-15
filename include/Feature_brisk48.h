#ifndef ANYFEATURE_VSLAM_FEATURE_BRISK48_H
#define ANYFEATURE_VSLAM_FEATURE_BRISK48_H

#include "FeatureExtractor.h"
#include "brisk/brisk.h"

namespace ANYFEATURE_VSLAM {

    class FeatureExtractor_brisk48 : public FeatureExtractor {
    public:

        cv::Ptr<cv::FeatureDetector> brisk_detector;
        cv::Ptr<cv::DescriptorExtractor> brisk_extractor;
        float detectTh;

        FeatureExtractor_brisk48(std::shared_ptr<FeatureExtractorSettings> &settings_);

        void detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) override;

        [[nodiscard]] int GetKeypointOctave(const cv::KeyPoint &keypoint) const override;
        [[nodiscard]] float GetKeypointSize(const cv::KeyPoint &keypoint) const override;
    };

    float DescriptorDistance_brisk48(const cv::Mat &a, const cv::Mat &b);

}

#endif //ANYFEATURE_VSLAM_FEATURE_BRISK48_H

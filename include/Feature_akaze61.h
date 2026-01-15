#ifndef ANYFEATURE_VSLAM_FEATURE_AKAZE61_H
#define ANYFEATURE_VSLAM_FEATURE_AKAZE61_H

#include "FeatureExtractor.h"
#include "akaze/AKAZE.h"

namespace ANYFEATURE_VSLAM {

    class FeatureExtractor_akaze61 : public FeatureExtractor {
    public:

        std::shared_ptr<libAKAZE::AKAZE> evolution{};
        AKAZEOptions akazeOptions{};
        
        FeatureExtractor_akaze61(std::shared_ptr<FeatureExtractorSettings> &settings_);

        void detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) override;

        void setupImage(const Image& img) override;

        [[nodiscard]] int GetKeypointOctave(const cv::KeyPoint &keypoint) const override;
        [[nodiscard]] float GetKeypointSize(const cv::KeyPoint &keypoint) const override;

    };

    float DescriptorDistance_akaze61(const cv::Mat &a, const cv::Mat &b);
}

#endif //ANYFEATURE_VSLAM_FEATURE_AKAZE61_H

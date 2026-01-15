#ifndef ANYFEATURE_VSLAM_FEATURE_ANYFEATNONBIN_H
#define ANYFEATURE_VSLAM_FEATURE_ANYFEATNONBIN_H

#include "FeatureExtractor.h"
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace ANYFEATURE_VSLAM {

    class FeatureExtractor_anyFeatNonBin : public FeatureExtractor {
    public:

        FeatureExtractor_anyFeatNonBin(std::shared_ptr<FeatureExtractorSettings> &settings_);

        void detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) override;

        [[nodiscard]] int GetKeypointOctave(const cv::KeyPoint &keypoint) const override;
        [[nodiscard]] float GetKeypointSize(const cv::KeyPoint &keypoint) const override;

    };

    float DescriptorDistance_anyFeatureNonBin(const cv::Mat &a, const cv::Mat &b);
}

#endif //ANYFEATURE_VSLAM_FEATURE_ANYFEATNONBIN_H
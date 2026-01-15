#ifndef ANYFEATURE_VSLAM_FEATURE_SIFT128_H
#define ANYFEATURE_VSLAM_FEATURE_SIFT128_H

#include "FeatureExtractor.h"
#include "SiftGPU.h"

namespace ANYFEATURE_VSLAM {

    class FeatureExtractor_sift128 : public FeatureExtractor {
    public:

        std::shared_ptr<SiftGPU> sift;

        FeatureExtractor_sift128(std::shared_ptr<FeatureExtractorSettings> &settings_);
        ~FeatureExtractor_sift128() {
            sift.reset();
        }

        void detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) override;

        void detectKeypoints(std::vector<cv::KeyPoint> &keypoints, const Image &img,
                             const float &detectTh, const int &nOctaves) const;

        void computeDescriptors(cv::Mat &descriptors, const Image &img) const;

        [[nodiscard]] int GetKeypointOctave(const cv::KeyPoint &keypoint) const override;
        [[nodiscard]] float GetKeypointSize(const cv::KeyPoint &keypoint) const override;
    };

    float DescriptorDistance_sift128(const cv::Mat &a, const cv::Mat &b);
}

#endif //ANYFEATURE_VSLAM_FEATURE_SIFT128_H

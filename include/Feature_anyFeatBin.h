//
// Created by fontan on 7/06/24.
//

#ifndef ANYFEATURE_VSLAM_FEATURE_ANYFEATBIN_H
#define ANYFEATURE_VSLAM_FEATURE_ANYFEATBIN_H

#include "FeatureExtractor.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace ANYFEATURE_VSLAM {

    class FeatureExtractor_anyFeatBin : public FeatureExtractor {
    public:

        FeatureExtractor_anyFeatBin(const int &nfeatures_, std::shared_ptr<FeatureExtractorSettings> &settings_);

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

    float DescriptorDistance_anyFeatureBin(const cv::Mat &a, const cv::Mat &b);

}

#endif //ANYFEATURE_VSLAM_FEATURE_ANYFEATBIN_H
#ifndef ANYFEATURE_VSLAM_FEATUREEXTRACTOR_H
#define ANYFEATURE_VSLAM_FEATUREEXTRACTOR_H

#include "Types.h"
#include "Definitions.h"
#include "Image.h"

#include <opencv2/opencv.hpp>

namespace ANYFEATURE_VSLAM {

    class FeatureExtractor;
    class ExtractorNode;

    enum CovarianceMethod {
        SIZE = 1,
        NONE = 0,
    };

    class FeatureExtractorSettings
    {
    public:

        float scaleFactor;
        int nOctaves;
        float detectTh;
        int maxNumFeatures;

        FeatureExtractorSettings(const KeypointType& keypointType_,
                                 const DescriptorType& descriptorType_,
                                 const std::string &settingsYamlFile);

        float GetDetectorScaleFactor() const {return scaleFactor;};
        int GetDetectorNumOctaves()  {return nOctaves;};
        float GetDetectorThreshold()  {return detectTh;};

        VerbosityLevel verbosity{LOW};

        KeypointType keypointType;
        DescriptorType descriptorType;
        float maxKeyPtSize{0.0};
        float minKeyPtSize{std::numeric_limits<float>::max()};

        float maxKeyPtSize0{};
        float maxKeyPtSigma0{};
        const float scaleFactorOrb{1.2f};
        const int nOctavesOrb{8};

    };

    class FeatureExtractor {

    public:

        FeatureExtractor(std::shared_ptr<FeatureExtractorSettings>& settings_): settings(settings_){};
        ~FeatureExtractor() = default;

        std::shared_ptr<FeatureExtractorSettings> settings{};

        void operator()( const Image & img,
                         std::vector<cv::KeyPoint>& keypoints,
                         cv::Mat& descriptors,
                         std::vector<mat2f>& keyPtsSigma2,
                         std::vector<mat2f>& keyPtsInf,
                         std::vector<float>& keyPtsSize);

        int inline GetLevels(){return settings->nOctaves;}
        float inline GetScaleFactor(){return settings->scaleFactor;}
        [[nodiscard]] float inline GetMaxKeyPtSize() const{return settings->maxKeyPtSize0;}
        [[nodiscard]] float inline GetMaxKeyPtSigma() const{return settings->maxKeyPtSigma0;}

    protected:
        void computeSize(std::vector<float>& keyPtsSize, const std::vector<cv::KeyPoint>& keypoints);
        void computeSigma(std::vector<mat2f>& keyPtsSigma2, std::vector<mat2f>& keyPtsInf,
                          const std::vector<float>& keyPtsSize, const std::vector<cv::KeyPoint>& keypoints,
                          const Image& img, const CovarianceMethod& method);

        virtual void setupImage(const Image& img){};

    protected:
        [[nodiscard]] virtual int GetKeypointOctave(const cv::KeyPoint& keypoint) const{};
        [[nodiscard]] virtual float GetKeypointSize(const cv::KeyPoint& keypoint) const{};

        virtual void detectAndCompute(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){};
    };
}
#endif //ANYFEATURE_VSLAM_FEATUREEXTRACTOR_H

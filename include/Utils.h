//
// Created by fontan on 20/02/24.
//

#ifndef ANYFEATURE_VSLAM_UTILS_H
#define ANYFEATURE_VSLAM_UTILS_H

#include "Types.h"

#include<opencv2/core/core.hpp>

#include <vector>
#include <random>

namespace ANYFEATURE_VSLAM{
    void printInfo(const std::string& function, const std::string& message,
                   const VerbosityLevel& verbosityLevel, const VerbosityLevel& verbosityLevelRequired,
                   std::string color = "\x1b[0m");
    void printError(const std::string& function, const std::string& message);

    std::string keypointName(const KeypointType & keypointType);
    std::string descriptorName(const DescriptorType& descriptorType);
    std::string featureName(const FeatureType& featureType);

    std::string matType(const int& matTypeIndex);
    cv::Scalar getFeatureColor(const FeatureType& featureType, const int& format);
    void linearRegression(float& a, float& b, float& R2, const std::vector<float>& x, const std::vector<float>& y);
    std::vector<std::vector<float>> loadBinFile(const std::string& filename, const int& numFloats);
    std::string replaceAllOccurrences(std::string str, const std::string& from, const std::string& to);

    class RandomIntegerGenerator
    {
    private:

        static std::mt19937 randomIntGenerator;

    public:
        static int GetRandomInteger(const int& minNumber, const int& maxNumber);
    };
}



#endif //ANYFEATURE_VSLAM_UTILS_H

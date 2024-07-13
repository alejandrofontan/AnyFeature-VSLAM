//
// Created by fontan on 30/03/24.
//

#ifndef ANYFEATURE_VSLAM_IMAGE_H
#define ANYFEATURE_VSLAM_IMAGE_H

#include<opencv2/core/core.hpp>

namespace ANYFEATURE_VSLAM {

    class Image {
    public:
        explicit Image(const std::string& imagePath);
        cv::Mat img{};
        cv::Mat grayImg{};
        cv::Mat mask{};

        std::string imageFile{};
        std::string keypointBinFile{};
        std::string scoresBinFile{};
        std::string descriptorsBinFile{};

        void LoadMask(const std::string& maskPath);
        void GetGrayImage(const bool& rgb);
        void FixImageSize(const int& new_width, const int& new_height);
    };

}

#endif //ANYFEATURE_VSLAM_IMAGE_H

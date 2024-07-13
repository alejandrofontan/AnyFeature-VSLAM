//
// Created by fontan on 30/03/24.
//

#include "Image.h"
#include "Utils.h"

#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>


ANYFEATURE_VSLAM::Image::Image(const std::string &imagePath): imageFile{imagePath} {

    img = cv::imread(imageFile,cv::IMREAD_UNCHANGED);

    keypointBinFile = replaceAllOccurrences(imageFile, "/rgb/", "/r2d2/keypoints/");
    keypointBinFile = replaceAllOccurrences(keypointBinFile, "png", "bin");
    scoresBinFile = replaceAllOccurrences(imageFile, "/rgb/", "/r2d2/scores/");
    scoresBinFile = replaceAllOccurrences(scoresBinFile, "png", "bin");
    descriptorsBinFile = replaceAllOccurrences(imageFile, "/rgb/", "/r2d2/descriptors/");
    descriptorsBinFile = replaceAllOccurrences(descriptorsBinFile, "png", "bin");
}

void ANYFEATURE_VSLAM::Image::LoadMask(const std::string &maskPath) {
    mask = cv::imread( maskPath.substr(0, 43) + "mask" + maskPath.substr(46, 50),cv::IMREAD_UNCHANGED);
}

void ANYFEATURE_VSLAM::Image::GetGrayImage(const bool& rgb) {
    if(img.channels() == 3){
        if(rgb){
            cvtColor(img,grayImg,CV_RGB2GRAY);
            return;
        }
        else{
            cvtColor(img,grayImg,CV_BGR2GRAY);
            return;
        }

    }
    else if(img.channels() == 4){
        if(rgb){
            cvtColor(img,grayImg,CV_RGBA2GRAY);
            return;
        }
        else{
            cvtColor(img,grayImg,CV_BGRA2GRAY);
            return;
        }
    }
    grayImg = img;
}

void ANYFEATURE_VSLAM::Image::FixImageSize(const int& new_width, const int& new_height){
    cv::Size new_size(new_width, new_height);
    cv::Mat resized_gray;
    cv::resize(grayImg, grayImg, new_size);
}
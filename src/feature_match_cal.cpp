#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<opencv2/core/core.hpp>
#include "sys/sysinfo.h"

#include<System.h>
#include<Types.h>
#include <yaml-cpp/yaml.h>

#include "include/FeatureExtractor.h"
#include "include/Feature_orb32.h"
int main(int argc, char **argv)
{
    
    // const KeypointType keypointType = KEYP_ORB;
    // const DescriptorType descriptorType = DESC_ORB;
    // std::string settingsYamlFile = "/home/alejandro/VSLAM-LAB/Baselines/AnyFeature-VSLAM-DEV/settings/orb32_settings.yaml";

    const KeypointType keypointType = KEYP_SIFT;
    const DescriptorType descriptorType = DESC_SIFT128;
    std::string settingsYamlFile = "/home/alejandro/VSLAM-LAB/Baselines/AnyFeature-VSLAM-DEV/settings/sift128_settings.yaml";

    std::shared_ptr<ANYFEATURE_VSLAM::FeatureExtractorSettings> extractorSettings = 
        std::make_shared<ANYFEATURE_VSLAM::FeatureExtractorSettings>(keypointType, descriptorType, settingsYamlFile);

    std::shared_ptr<ANYFEATURE_VSLAM::FeatureExtractor> featureExtractor;
    switch (keypointType) {
        case KEYP_ORB:{
            featureExtractor = std::make_shared<ANYFEATURE_VSLAM::FeatureExtractor_orb32>(10000, extractorSettings);
            break;
        }
        case KEYP_SIFT:{
            featureExtractor = std::make_shared<ANYFEATURE_VSLAM::FeatureExtractor_sift128>(10000, extractorSettings);
            break;
        }
    }

    std::vector<std::string> imageFilenames;
    imageFilenames.push_back("/home/alejandro/VSLAM-LAB-Benchmark/ETH/table_3/rgb_0/2989.023949.png");
    imageFilenames.push_back("/home/alejandro/VSLAM-LAB-Benchmark/ETH/table_3/rgb_0/2989.355719.png");
    imageFilenames.push_back("/home/alejandro/VSLAM-LAB-Benchmark/ETH/table_3/rgb_0/2989.687489.png");
    imageFilenames.push_back("/home/alejandro/VSLAM-LAB-Benchmark/ETH/table_3/rgb_0/2990.019259.png");
    imageFilenames.push_back("/home/alejandro/VSLAM-LAB-Benchmark/ETH/table_3/rgb_0/2990.351030.png");
    imageFilenames.push_back("/home/alejandro/VSLAM-LAB-Benchmark/ETH/table_3/rgb_0/2990.682800.png");
    imageFilenames.push_back("/home/alejandro/VSLAM-LAB-Benchmark/ETH/table_3/rgb_0/2991.014570.png");
    std::vector<ANYFEATURE_VSLAM::Image> im{};
    for (int i{0}; i < imageFilenames.size(); i++){
        ANYFEATURE_VSLAM::Image im_i(imageFilenames[i]);
        im_i.GetGrayImage(true);
        im.push_back(im_i);
    }
    
    std::map<int, std::vector<cv::KeyPoint>> mvKeys;
    std::map<int,cv::Mat> mDescriptors;
    std::map<int,std::vector<mat2f>> keyPtsSigma2{}, keyPtsInf{};
    std::map<int,std::vector<float>> keyPtsSize{};
    for (int i{0}; i < imageFilenames.size(); i++){
        std::cout << "Image " << i << ": " << imageFilenames[i] << std::endl;
        featureExtractor->operator()(im[i],mvKeys[i],mDescriptors[i],keyPtsSigma2[i],keyPtsInf[i],keyPtsSize[i]);
        std::cout << "DetectTh: " << extractorSettings->detectTh << " - Keypoints: " << mvKeys[i].size() << std::endl;
    }

    std::vector<cv::DMatch> matches;
    //cv::BFMatcher bf_matcher{cv::NORM_HAMMING, false};
    cv::BFMatcher bf_matcher{cv::NORM_L2, false};

    std::ofstream outFile("keypoint_stats.txt");
    std::map<float,std::vector<float>> perc_matches{};
    for (int j{1}; j < imageFilenames.size(); j++){
        bf_matcher.match(mDescriptors[0], mDescriptors[j], matches);
        //std::cout << "Matches found: " << matches.size() << std::endl;
        for (int i{0}; i < 50; i++){
            float threshold = 0.05f + i * 0.02f; // 15 + i * 5; //
            std::vector<cv::DMatch> good_matches;
            for (const auto& match : matches) {
                //std::cout << "Match distance: " << match.distance << std::endl;
                if (match.distance <= threshold) {
                    good_matches.push_back(match);
                }
            }
            if (matches.size() < 8){
                std::cout << "Threshold: " << threshold << " - Good Matches: " << good_matches.size() 
                    << " - RANSAC Inliers: " << 0 << std::endl;
                perc_matches[threshold].push_back(0.0 * 100.0f);  
                continue;
            }
            std::vector<cv::Point2f> points1, points2;
            for (const auto& match : matches) {
                points1.push_back(mvKeys[0][match.queryIdx].pt);
                points2.push_back(mvKeys[1][match.trainIdx].pt);
            }
            std::vector<uchar> inliers_mask; 
            cv::Mat F = cv::findFundamentalMat(
                points1,           // Points from image 1
                points2,           // Points from image 2
                cv::FM_RANSAC,     // RANSAC algorithm
                3.0,               // Distance threshold (max distance from epipolar line)
                0.95,              // Confidence level
                inliers_mask       // Output mask (1 for inlier, 0 for outlier)
            );
            std::vector<cv::DMatch> ransac_matches;
            for (size_t i = 0; i < inliers_mask.size(); ++i) {
                if (inliers_mask[i]) {
                    ransac_matches.push_back(matches[i]);
                }
            }
            perc_matches[threshold].push_back(static_cast<float>(good_matches.size()) / static_cast<float>(ransac_matches.size()) * 100.0f);  
            std::cout << "Threshold: " << threshold << " - Good Matches: " << good_matches.size() 
                << " - RANSAC Inliers: " << ransac_matches.size() << std::endl;       
        }
    }

    std::cout << "Writing median percentages to file." << std::endl;
    if (outFile.is_open()) {
        for (auto& [threshold, perc]: perc_matches){
            std::sort(perc.begin(), perc.end());
            float median_perc = perc[perc.size() / 2];
            outFile << threshold << ", " << median_perc << std::endl;
        }
    }
    outFile.close();


    return 0;
}

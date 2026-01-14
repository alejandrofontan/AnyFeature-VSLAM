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

    
    std::string imageFilename = "/home/alejandro/VSLAM-LAB-Benchmark/ETH/table_3/rgb_0/2989.023949.png";
    ANYFEATURE_VSLAM::Image im(imageFilename);
    im.GetGrayImage(true);

    std::ofstream outFile("keypoint_stats.txt");
    std::shared_ptr<ANYFEATURE_VSLAM::FeatureExtractor> featureExtractor;
    if (outFile.is_open()) {
        for(int i = 0; i < 100; i++){
            extractorSettings->detectTh = 1.0f + i * 1.0f; 
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
            
            std::vector<cv::KeyPoint> mvKeys;
            cv::Mat mDescriptors;
            vector<mat2f> keyPtsSigma2{};
            vector<mat2f> keyPtsInf{};
            vector<float> keyPtsSize{};

            featureExtractor->operator()(im,mvKeys,mDescriptors,keyPtsSigma2,keyPtsInf,keyPtsSize);
            std::cout << "DetectTh: " << extractorSettings->detectTh << " - Keypoints: " << mvKeys.size() << std::endl;
            outFile << extractorSettings->detectTh << ", " << mvKeys.size() << std::endl;
        }
        outFile.close();
    }
    // cv::Mat imgKeypoints;
    // cv::drawKeypoints(
    //     im.img,               
    //     mvKeys,                  
    //     imgKeypoints,           
    //     cv::Scalar(0, 255, 0),   
    //     cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS 
    // );

    // cv::namedWindow("SIFT Keypoints", cv::WINDOW_NORMAL);
    // cv::imshow("SIFT Keypoints", imgKeypoints);

    // // 4. Save or Wait
    // cv::waitKey(0);

    return 0;
}

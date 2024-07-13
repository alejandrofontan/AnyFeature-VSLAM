//
// Created by fontan on 20/02/24.
//

#include "FeatureExtractor.h"
#include "Utils.h"
#include "MathFunctions.h"

#include <opencv2/core/core.hpp>
#include <vector>

#include <random>

using namespace cv;
using namespace std;

int ANYFEATURE_VSLAM::FeatureExtractorSettings::numOctaves0;
float ANYFEATURE_VSLAM::FeatureExtractorSettings::scaleFactor0;
float ANYFEATURE_VSLAM::FeatureExtractorSettings::th0;

ANYFEATURE_VSLAM::FeatureExtractorSettings::FeatureExtractorSettings(
        const KeypointType& keypointType_, const DescriptorType& descriptorType_,
        const std::string &settingsYamlFile):
        keypointType(keypointType_), descriptorType(descriptorType_){

    if (settingsYamlFile != "none"){
        cv::FileStorage fSettings(settingsYamlFile, cv::FileStorage::READ);
        cout << endl  << "Loading Feature Extractor Settings from : " << settingsYamlFile << endl;
        numOctaves0 = fSettings["FeatureExtractor.numOctaves"];
        scaleFactor0 = fSettings["FeatureExtractor.scaleFactor"];
        th0 = fSettings["FeatureExtractor.detectionTh"];
        cout <<  "- numOctaves: " << numOctaves0 << endl;
        cout <<  "- scaleFactor: " << scaleFactor0 << endl;
        cout <<  "- detectionTh: " << th0 << endl;
    }
    scaleFactor = GetDetectorNominalScaleFactor();
    nOctaves = GetDetectorNominalNumOctaves();
    detectTh = GetDetectorNominalThreshold();

#ifdef VANILLA_ORB_SLAM2
    ON_automaticTuning = false;
    scaleFactor = 1.2f;
    nOctaves = 8;
    iniThFAST = 20;
    minThFAST = 7;
    detectTh = float(iniThFAST);
#else
    ON_automaticTuning = true;
    iniThFAST = 20;
    minThFAST = 7;
#endif
    maxKeyPtSize0  = pow(scaleFactorOrb,float(nOctavesOrb - 1.0));
    maxKeyPtSigma0 = pow(scaleFactorOrb,float(nOctavesOrb - 1.0));
    maxKeyPtSize = pow(scaleFactorOrb,float(nOctavesOrb - 1.0));
    minKeyPtSize = 1.0f;
}

ANYFEATURE_VSLAM::FeatureExtractorSettings::FeatureExtractorSettings(
        const float& scaleFactor_, const int& nOctaves_,
        const int& iniThFAST_, const int& minThFAST_,
        const KeypointType& keypointType_, const DescriptorType& descriptorType_):
        scaleFactor(scaleFactor_),nOctaves(nOctaves_),
        iniThFAST(iniThFAST_),minThFAST(minThFAST_),
        keypointType(keypointType_), descriptorType(descriptorType_){

    /*ON_automaticTuning = false;
    detectTh = float(iniThFAST);

    maxKeyPtSize0  = pow(scaleFactor,float(nOctaves - 1.0));
    maxKeyPtSigma0 = pow(scaleFactor,float(nOctaves - 1.0));*/
}

#ifndef VANILLA_ORB_SLAM2
ANYFEATURE_VSLAM::FeatureExtractor::FeatureExtractor(const int& nfeatures_, shared_ptr<FeatureExtractorSettings>& settings_):
        settings(settings_), nfeatures(nfeatures_){

    mvScaleFactor.resize(settings->nOctaves);
    mvLevelSigma2.resize(settings->nOctaves);
    mvScaleFactor[0]=1.0f;
    mvLevelSigma2[0]=1.0f;
    for(int i=1; i<settings->nOctaves; i++)
    {
        mvScaleFactor[i]=mvScaleFactor[i-1]*settings->scaleFactor;
        mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
    }

    mvInvScaleFactor.resize(settings->nOctaves);
    mvInvLevelSigma2.resize(settings->nOctaves);
    for(int i=0; i<settings->nOctaves; i++)
    {
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
        mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
    }

    mvImagePyramid.resize(settings->nOctaves);

    mnFeaturesPerLevel.resize(settings->nOctaves);
    float factor = 1.0f / settings->scaleFactor;
    float nDesiredFeaturesPerScale = float(nfeatures) * (1 - factor)/(1 - (float)pow((double)factor, (double)settings->nOctaves));

    int sumFeatures = 0;
    for( int level = 0; level < settings->nOctaves-1; level++ )
    {
        mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
        sumFeatures += mnFeaturesPerLevel[level];
        nDesiredFeaturesPerScale *= factor;
    }
    mnFeaturesPerLevel[settings->nOctaves-1] = std::max(nfeatures - sumFeatures, 0);
}

void ANYFEATURE_VSLAM::FeatureExtractor::operator()(const Image& img,
                                             std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                                             std::vector<mat2f>& keyPtsSigma2, std::vector<mat2f>& keyPtsInf, std::vector<float>& keyPtsSize)
{
    initializeExtractor(img);
    if(settings->ON_automaticTuning)
        automaticTuning(img);
    detectAndCompute(img, keypoints, descriptors);
    computeSize(keyPtsSize,keypoints);
    computeSigma(keyPtsSigma2, keyPtsInf,keyPtsSize,keypoints,img,CovarianceMethod::SIZE);
}

void ANYFEATURE_VSLAM::FeatureExtractor::operator()(const Image& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
    initializeExtractor(img);
    if(settings->ON_automaticTuning)
        automaticTuning(img);
    detectAndCompute(img, keypoints, descriptors);
}
#endif

void ANYFEATURE_VSLAM::FeatureExtractor::computeSize(std::vector<float>& keyPtsSize, const std::vector<cv::KeyPoint>& keypoints){
    keyPtsSize.clear();
    keyPtsSize.reserve(keypoints.size());
    for(auto& keyPt: keypoints){
        float keyPtSize = GetKeypointSize(keyPt);
        float keyPtSize_norm{settings->maxKeyPtSize};
        if(settings->maxKeyPtSize > settings->minKeyPtSize)
            keyPtSize_norm =  1.0f +  (keyPtSize - settings->minKeyPtSize) * (settings->maxKeyPtSize0 - 1.0f)/(settings->maxKeyPtSize - settings->minKeyPtSize);
        keyPtsSize.push_back(keyPtSize_norm);
    }
}

void ANYFEATURE_VSLAM::FeatureExtractor::computeSigma(std::vector<mat2f>& keyPtsSigma2, std::vector<mat2f>& keyPtsInf,
                                                      const std::vector<float>& keyPtsSize, const std::vector<cv::KeyPoint>& keypoints,
                                                      const Image& img, const CovarianceMethod& method){
    keyPtsSigma2.clear();
    keyPtsInf.clear();
    keyPtsSigma2.reserve(keypoints.size());
    keyPtsInf.reserve(keypoints.size());
    switch (method) {
        case NONE:{
            for(const auto& keypoint: keypoints){
                keyPtsSigma2.emplace_back(mat2f::Identity());
                keyPtsInf.emplace_back(mat2f::Identity());
            }
            return;
        }
        case SIZE:{
            int iKeyPt{0};
            for(const auto& keypoint: keypoints){
                float keypointSigma = keyPtsSize[iKeyPt];
                float keypointSigma2 = keypointSigma * keypointSigma;
                float keypointInf = 1.0f/(keypointSigma2);
                keyPtsSigma2.emplace_back(keypointSigma2 * mat2f::Identity());
                keyPtsInf.emplace_back(keypointInf * mat2f::Identity());
                iKeyPt++;
            }
            return;
        }
    }
}

float ANYFEATURE_VSLAM::FeatureExtractor::computeStep(const float& th_0, const Image& img){

    std::map<int,std::vector<cv::KeyPoint>> keypoints_level_tmp;

    int numKeyPts_sup{0};
    keypoints_level_tmp.clear();
    detectKeypoints(keypoints_level_tmp, img, 1.1f * th_0, 1);
    numKeyPts_sup = (int) keypoints_level_tmp[0].size();
    cout << "numKeyPts_sup =" << numKeyPts_sup << endl;

    int numKeyPts_inf{0};
    keypoints_level_tmp.clear();
    detectKeypoints(keypoints_level_tmp, img, 0.9f * th_0, 1);
    numKeyPts_inf = (int) keypoints_level_tmp[0].size();
    cout << "numKeyPts_inf =" << numKeyPts_inf << endl;

    float dnum_dth = float(numKeyPts_sup - numKeyPts_inf) / (0.2f * th_0 );
    //float step = 50.0 / dnum_dth;
    return dnum_dth;
}

void ANYFEATURE_VSLAM::FeatureExtractor::automaticTuning(const Image& img) {
    switch (settings->keypointType) {
        case KEYP_R2D2:
            return;
    }

    int numKeypoints_th{mnFeaturesPerLevel[0]};
    const int numSteps = 200;
    float th_0_0 =  settings->GetDetectorNominalThreshold();
    float th_0 = 1.0f * th_0_0;
    /*if(settings->descriptorType == DESC_SIFT128)
        th_0 = 0.25f * th_0_0;*/

    float bestTh{th_0};
    /*float dnum_dth;
    vector<float> thresholds{};
    vector<float> numKeyPts{};
    int minError{1000000};
    for (int it{0}; it < numSteps; ++it) {
        int numKeyPts_it{0};
        std::map<int,std::vector<cv::KeyPoint>> keypoints_level_tmp;
        detectKeypoints(keypoints_level_tmp, img, th_0, 1);

        numKeyPts_it = keypoints_level_tmp[0].size();
        cout << "th_0 =" << th_0  << " numKeyPts_it = "<< numKeyPts_it <<endl;

        if ((!numKeyPts.empty()) and (numKeyPts_it == numKeyPts.back())) {
            th_0 += 50.0/dnum_dth;
            continue;
        }
        cout << th_0 << " " << numKeyPts_it << " " << numKeypoints_th << endl;
        if(numKeyPts_it > numKeypoints_th)
            break;

        thresholds.push_back(th_0);
        numKeyPts.push_back(float(numKeyPts_it));

        int error = abs(numKeypoints_th - numKeyPts_it);
        if (error < minError){
            minError = error;
            bestTh = th_0;
        }
        if(minError < 50)
            break;

        dnum_dth = computeStep(th_0, img);
        cout << dnum_dth  << endl;

        //if(abs(50.0/dnum_dth) > 0.5 * th_0)
            //break;
        th_0 += 50.0/dnum_dth;
        //th_0 = 1.0f / (step + (1.0f / th_0));
    }*/

    settings->detectTh = bestTh;
    std::vector<cv::KeyPoint> keypoints_tmp;
    //detectKeypoints(keypoints_tmp, img, settings->detectTh, settings->nOctaves);

    /*for(auto& keyPt: keypoints_tmp){

        int octave = GetKeypointOctave(keyPt);
        if((settings->nOctaves - 1) < octave){
            settings->nOctaves = octave + 1;
        }
        float keyPtSize = GetKeypointSize(keyPt);
        if(keyPtSize > settings->maxKeyPtSize)
            settings->maxKeyPtSize = keyPtSize;
        if(keyPtSize < settings->minKeyPtSize)
            settings->minKeyPtSize = keyPtSize;
    }*/

    /*cout << "automaticTuning : " << endl;
    cout << "    - settings->detectTh = " << settings->detectTh << endl;
    cout << "    - keypoints_tmp.size() = " << keypoints_tmp.size() << endl;
    cout << "    - settings->maxKeyPtSize = " << settings->maxKeyPtSize << endl;
    cout << "    - settings->minKeyPtSize = " << settings->minKeyPtSize << endl;
    cout << "    - settings->nOctaves = " << settings->nOctaves << endl;*/

    settings->ON_automaticTuning = false;
}

void ANYFEATURE_VSLAM::FeatureExtractor::filterKeypoints_notScaled(std::map<int,std::vector<cv::KeyPoint>>& keypoints_level, const cv::Mat& image, const cv::Mat& mask) const{
    const int w{image.cols};
    const int h{image.rows};
    for(auto& [level,keypoints_] : keypoints_level) {
        std::vector<cv::KeyPoint> keypoints_filtered = DistributeOctTree(keypoints_, 0, w, 0, h, mnFeaturesPerLevel[level], level);
        //cout << "nFeaturesPerLevel[level] = " << keypoints_filtered.size() << " / " << mnFeaturesPerLevel[level] << " / " << keypoints_.size() << endl;
        keypoints_level[level] = keypoints_filtered;
    }
}

void ANYFEATURE_VSLAM::FeatureExtractor::filterKeypoints_scaled(std::map<int,std::vector<cv::KeyPoint>>& keypoints_level, const cv::Mat& image, const cv::Mat& mask) const{
    for(auto& [level,keypoints_] : keypoints_level) {
        const int w{mvImagePyramid[level].cols};
        const int h{mvImagePyramid[level].rows};
        std::vector<cv::KeyPoint> keypoints_filtered = DistributeOctTree(keypoints_, 0, w, 0, h, mnFeaturesPerLevel[level], level);
        //cout << "nFeaturesPerLevel[level] = " << keypoints_filtered.size() << " / " << mnFeaturesPerLevel[level] << " / " << keypoints_.size() << endl;
        keypoints_level[level] = keypoints_filtered;
    }
}

void ANYFEATURE_VSLAM::FeatureExtractor::mergeKeypointLevels(std::vector<cv::KeyPoint>& keypoints,
                                                            cv::Mat& descriptors,
                                                            std::map<int,cv::Mat>& descriptors_level,
                                                            std::map<int,std::vector<cv::KeyPoint>>& keypoints_level) const {
    keypoints.clear();
    descriptors.release();
    std::vector<cv::Mat> descriptorsVector {};
    for(auto& [level, keypoints_]: keypoints_level){
        keypoints.insert(keypoints.end(),keypoints_.begin(),keypoints_.end());
        descriptorsVector.push_back(descriptors_level[level]);
    }
    cv::vconcat(descriptorsVector,descriptors);
}

void ANYFEATURE_VSLAM::FeatureExtractor::scaleKeypoints(std::map<int,std::vector<cv::KeyPoint>>& keypoints_level) const {
    for(auto& [level, keypoints_]: keypoints_level){
        float scale = powf(settings->scaleFactor,float(level));
        for(auto& keyPt: keypoints_level[level]){
            keyPt.pt.x *= scale;
            keyPt.pt.y *= scale;
            keyPt.octave = level;
            keyPt.size = scale;
        }
    }
}
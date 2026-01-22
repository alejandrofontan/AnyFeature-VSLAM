/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef FEATUREMATCHER_H
#define FEATUREMATCHER_H

#include <vector>
#include <opencv2/core/core.hpp>

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Feature_sift128.h"
#include "matcher/lightglue/matcher.hpp"


namespace ANYFEATURE_VSLAM
{

class FeatureMatcher
{    
public:

    FeatureMatcher(const int& imageWidth, const int& imageHeight, float nnratio=0.6, bool checkOri=true);

    // Computes the Hamming distance between two ORB descriptors
    static Descriptor_Distance_Type DescriptorDistance(const cv::Mat &a, const cv::Mat &b, const DescriptorType& descriptorType_);
    cv::NormTypes getNormType(const FeatureType& featureType_);
    std::vector<cv::DMatch> featureMatching(const cv::Mat& desc1, const cv::Mat& desc2, const FeatureType& ft);

    std::vector<cv::DMatch> featureMatching(const cv::Mat& desc1, const cv::Mat& desc2,  const std::vector<cv::KeyPoint>& kps1, const std::vector<cv::KeyPoint>& kps2, const FeatureType& ft, 
       bool lightglue = true , bool robustMatching = true, int outlierMehod = cv::FM_RANSAC);

    std::vector<cv::DMatch> lightglueMatching(
            const std::vector<cv::KeyPoint>& kps1, const cv::Mat& desc1,
            const std::vector<cv::KeyPoint>& kps2, const cv::Mat& desc2,
            float min_score = 0.0f);
    std::vector<cv::DMatch> robustFeatureMatching(std::vector<cv::DMatch>& matches, const std::vector<cv::KeyPoint>& kps1, const std::vector<cv::KeyPoint>& kps2, int outlierMehod = cv::FM_RANSAC);        

    // AllFeature-VSLAM SearchBruteForce         

    int SearchBruteForce(Frame &CurrentFrame, const Frame &LastFrame, 
        const std::vector<FeatureType>& featureTypes);
    
    std::map<FeatureType, int> SearchBruteForce(const Keyframe& keyframe, const Frame &frame, 
        std::map<FeatureType, std::vector<Pt>>& mapPointMatches, 
        const std::vector<FeatureType>& featureTypes);

    void SearchForTriangulation(const Keyframe& keyframe1, const Keyframe& keyframe2,
                                std::map<FeatureType, vector<pair<size_t,size_t>>>& matchedPairs,
                                const std::vector<FeatureType>& featureTypes);

    int SearchForInitialization(const Frame &F1, const Frame &F2, std::vector<cv::Point2f> &pointsPrevMatched, std::vector<int> &matches12, const FeatureType& featureType);

    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    int SearchByProjection(Frame &F, const std::vector<Pt> &vpMapPoints, const float& radiusTh);
    int SearchByProjection(Frame &Frame, const vector<Pt> &mapPoints);

    // Project MapPoints seen in KeyFrame into the Frame and search matches.
    // Used in relocalisation (Tracking)
    int SearchByProjection(Frame &CurrentFrame, Keyframe pKF, const std::set<Pt> &sAlreadyFound, const float& radiusTh, const bool& useHighMatchingThreshold, const FeatureType& featureType);

    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in loop detection (Loop Closing)
     int SearchByProjection(Keyframe pKF, const mat4f& Scw, const std::vector<Pt> &vpPoints, std::vector<Pt> &vpMatched, const float& radiusTh, const FeatureType& featureType);
     
    // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
    // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
    // Used in Relocalisation and Loop Detection
    int SearchByBoW(Keyframe pKF1, Keyframe pKF2, std::vector<Pt> &vpMatches12, const FeatureType& featureType);

    // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
    // In the stereo and RGB-D case, s12=1
    int SearchBySim3(Keyframe pKF1, Keyframe pKF2, std::vector<Pt> &vpMatches12, const float &s12, const mat3f &R12, const vec3f &t12, const float& radiusTh, const FeatureType& featureType);

    // Project MapPoints into KeyFrame and search for duplicated MapPoints.
    int Fuse(Keyframe pKF, const vector<Pt> &vpMapPoints, const float& radiusTh, const FeatureType& featureType);

    // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
    int Fuse(Keyframe pKF, const mat4f& Scw, const std::vector<Pt> &vpPoints, const float& radiusTh, vector<Pt> &vpReplacePoint, const FeatureType& featureType);

    static void setDescriptorDistanceThresholds(const string &feature_settings_yaml_file, const FeatureType& featureType);

public:

    static VerbosityLevel verbosity;
    static std::map<FeatureType, Descriptor_Distance_Type> TH_LOW;
    static std::map<FeatureType, Descriptor_Distance_Type> TH_HIGH;
    static std::map<FeatureType, Descriptor_Distance_Type> descDistTh_high_reloc;
    static std::map<FeatureType, Descriptor_Distance_Type> descDistTh_low_reloc;

    static const int HISTO_LENGTH;
    static float radiusScale;

protected:

    bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const mat3f& F12, const Keyframe pKF, const float& sigma2_kp2);

    float RadiusByViewingCos(const float &viewCos);

    static vector<vector<int>> initRotationHistogram(float& rotFactor, const int& histLength);
    static void updateRotationHistogram(vector<vector<int>>& rotHist,
                                        const KeypointIndex& idx,
                                        const cv::KeyPoint& keyPt, const cv::KeyPoint& refKeyPt,
                                        const float& rotFactor, const int& histLength);
    static void computeThreeMaxima(vector<vector<int>>& rotHist, int &ind1, int &ind2, int &ind3);
    static void filterMatchesWithOrientation(vector<vector<int>>& rotHist, vector<Pt>& points, int& nMatches);
    static void filterMatchesWithOrientation(vector<vector<int>>& rotHist, vector<int>& matches, int& nMatches);

    float mfNNratio;
    bool mbCheckOrientation;

    const Descriptor_Distance_Type highestPossibleDistance{std::numeric_limits<Descriptor_Distance_Type>::max()};

    SiftMatchGPU sift_match_gpu_{};
    cv::BFMatcher bf_matcher_hamming{cv::NORM_HAMMING, true};
    cv::BFMatcher bf_matcher_L2{cv::NORM_L2, true};
    std::shared_ptr<matcher::LightGlue> matcher_lightglue;
    std::shared_ptr<torch::Device> torch_device;
    int imageWidth;
    int imageHeight;

    // Matching options

    // SearchBruteForce Keyframe-Frame
    // Tracking::TrackReferenceKeyframe & Tracking::Relocalization
    static const bool sBF_kf_lightglue = true;
    
        
    // SearchBruteForce Frame-Frame
    // Tracking::TrackWithMotionModel
    static const bool sBF_ff_lightglue = true;

    // SearchForInitialization Frame-Frame
    // Tracking::MonocularInitialization
    static const bool sFI_ff_lightglue = true;
    static const bool sFI_ff_robustMatching = true;
    static const int  sFI_ff_outlierMethod = cv::FM_LMEDS;  
    
    // SearchForTriangulation Keyframe-Keyframe
    // LocalMapping::CreateNewMapPoints
    static const bool sFT_kk_lightglue = true;
    static const bool sFT_kk_robustMatching = true;
    static const int  sFT_kk_outlierMethod = cv::FM_LMEDS;   

};

}// namespace ORB_SLAM

#endif // FEATUREMATCHER_H

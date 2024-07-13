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


namespace ANYFEATURE_VSLAM
{

class FeatureMatcher
{    
public:

    FeatureMatcher(float nnratio=0.6, bool checkOri=true);

    // Computes the Hamming distance between two ORB descriptors
    static Descriptor_Distance_Type DescriptorDistance(const cv::Mat &a, const cv::Mat &b, const DescriptorType& descriptorType_);

    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    int SearchByProjection(Frame &F, const std::vector<Pt> &vpMapPoints, const float& radiusTh);

    // Project MapPoints tracked in last frame into the current frame and search matches.
    // Used to track from previous frame (Tracking)
    int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float& radiusTh, const bool bMono);

    // Project MapPoints seen in KeyFrame into the Frame and search matches.
    // Used in relocalisation (Tracking)
    int SearchByProjection(Frame &CurrentFrame, Keyframe pKF, const std::set<Pt> &sAlreadyFound, const float& radiusTh, const bool& useHighMatchingThreshold);

    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in loop detection (Loop Closing)
     int SearchByProjection(Keyframe pKF, const mat4f& Scw, const std::vector<Pt> &vpPoints, std::vector<Pt> &vpMatched, const float& radiusTh);

    // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
    // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
    // Used in Relocalisation and Loop Detection
    int SearchByBoW(Keyframe pKF, Frame &F, std::vector<Pt> &vpMapPointMatches);
    int SearchByBoW(Keyframe pKF1, Keyframe pKF2, std::vector<Pt> &vpMatches12);

    // Matching for the Map Initialization (only used in the monocular case)
    int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, const int& windowSize, const DescriptorType& descriptorType);

    // Matching to triangulate new MapPoints. Check Epipolar Constraint.
    int SearchForTriangulation(Keyframe pKF1, Keyframe pKF2, const mat3f& F12,
                               std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo, const DescriptorType& descriptorType);

    // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
    // In the stereo and RGB-D case, s12=1
    int SearchBySim3(Keyframe pKF1, Keyframe pKF2, std::vector<Pt> &vpMatches12, const float &s12, const mat3f &R12, const vec3f &t12, const float& radiusTh);

    // Project MapPoints into KeyFrame and search for duplicated MapPoints.
    int Fuse(Keyframe pKF, const vector<Pt> &vpMapPoints, const float& radiusTh);

    // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
    int Fuse(Keyframe pKF, const mat4f& Scw, const std::vector<Pt> &vpPoints, const float& radiusTh, vector<Pt> &vpReplacePoint);

    static void setDescriptorDistanceThresholds(const std::vector<Descriptor_Distance_Type>& descriptorDistances_, const std::vector<int>& numCandidates_,const DescriptorType& descriptorType);
    static void setDescriptorDistanceThresholds(const string &feature_settings_yaml_file);

public:

    static VerbosityLevel verbosity;
    static Descriptor_Distance_Type TH_LOW;
    static Descriptor_Distance_Type TH_HIGH;
    static Descriptor_Distance_Type descDistTh_high_reloc;
    static Descriptor_Distance_Type descDistTh_low_reloc;

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

};

}// namespace ORB_SLAM

#endif // FEATUREMATCHER_H

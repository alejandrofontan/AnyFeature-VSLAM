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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"
#include "FeatureVocabulary.h"
#include "FeatureExtractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "Types.h"

#include <mutex>


namespace ANYFEATURE_VSLAM
{

class Map;
class Frame;
class KeyFrameDatabase;

class KeyFrame;
typedef shared_ptr<ANYFEATURE_VSLAM::KeyFrame> Keyframe;
class MapPoint;
typedef shared_ptr<ANYFEATURE_VSLAM::MapPoint> Pt;

class KeyFrame : public std::enable_shared_from_this<KeyFrame>
{
public:
    KeyFrame(Frame &F, shared_ptr<Map> pMap, shared_ptr<KeyFrameDatabase> pKFDB);
    std::shared_ptr<KeyFrame> thisKeyframe() {
        return shared_from_this();
    }

    // Pose functions
    void SetPose(const mat4f &Tcw_);
    mat4f GetPose();
    mat4f GetPoseInverse();
    vec3f GetCameraCenter();
    vec4f GetStereoCenter();
    mat3f GetRotation();
    vec3f GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(Keyframe pKF, const int &weight);
    void EraseConnection(Keyframe pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    map<KeyframeId,Keyframe> GetConnectedKeyFrames();
    std::vector<Keyframe > GetVectorCovisibleKeyFrames();
    std::vector<Keyframe> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<Keyframe> GetCovisiblesByWeight(const int &w);
    int GetWeight(Keyframe pKF);

    // Spanning tree functions
    void AddChild(Keyframe pKF);
    void EraseChild(Keyframe pKF);
    void ChangeParent(Keyframe pKF);
    std::set<Keyframe> GetChilds();
    Keyframe GetParent();
    bool hasChild(Keyframe pKF);

    // Loop Edges
    void AddLoopEdge(Keyframe pKF);
    std::set<Keyframe> GetLoopEdges();

    // MapPoint observation functions
    Pt CreateMonocularMapPoint(const vec3f& worldPos,
                             const KeypointIndex& refIndex,
                             Keyframe projKeyframe, const KeypointIndex& projIndex,
                             const FeatureType& featureType);
    Pt CreateMapPoint(const vec3f& worldPos,
                      const KeypointIndex& refIndex,
                      const FeatureType& featureType);

    void AddMapPoint(Pt pt, const KeypointIndex& index);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(Pt pMP);
    void ReplaceMapPointMatch(const size_t &idx, Pt pMP);
    std::set<Pt> GetMapPoints();
    std::vector<Pt> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    Pt GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    vec3f UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(Keyframe pKF1, Keyframe pKF2){
        return pKF1->keyId < pKF2->keyId;
    }

    [[nodiscard]] float GetKeyPtSize(const KeypointIndex &keyPtIdx) const;
    [[nodiscard]] float GetKeyPt1DSigma2(const KeypointIndex &keyPtIdx) const;
    [[nodiscard]] mat2f GetKeyPt2DSigma2(const KeypointIndex &keyPtIdx) const;
    [[nodiscard]] mat3f GetKeyPt3DSigma2(const KeypointIndex &keyPtIdx) const;
    [[nodiscard]] float GetKeyPt1DInf(const KeypointIndex &keyPtIdx) const;
    [[nodiscard]] mat2f GetKeyPt2DInf(const KeypointIndex &keyPtIdx) const;
    [[nodiscard]] mat3f GetKeyPt3DInf(const KeypointIndex &keyPtIdx) const;
    [[nodiscard]] float GetKeyPt1DSigma(const KeypointIndex &keyPtIdx) const;

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    KeyframeId keyId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    mat4f TcwGBA;
    mat4f TcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    mat4f Tcp;

    // Scale
    float sizeTolerance{};
    vector<mat2f> keyPtsSigma2{};
    vector<mat2f> keyPtsInf{};
    vector<float> keyPtsSize{};
    float maxKeyPtSize{};
    float maxKeyPtSigma{};

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;

    const cv::Mat mK; // Remove ???????????????????????
    mat3f K;

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    mat4f Tcw;
    mat4f Twc;
    vec3f twc;

    vec4f Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<Pt> mvpMapPoints;

    // BoW
    shared_ptr<KeyFrameDatabase> mpKeyFrameDB;
    shared_ptr<Vocabulary> vocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyframeId, Keyframe> connectedKeyFrames;
    std::map<KeyframeId,int> connectedKeyFrameWeights;
    std::vector<Keyframe> orderedConnectedKeyFrames;
    std::vector<int> orderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    Keyframe mpParent;
    std::set<Keyframe> mspChildrens;
    std::set<Keyframe> mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    float mHalfBaseline; // Only for visualization

    shared_ptr<Map> mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

//typedef ANYFEATURE_VSLAM::Keyframe Keyframe;

} //namespace ORB_SLAM

#endif // KEYFRAME_H

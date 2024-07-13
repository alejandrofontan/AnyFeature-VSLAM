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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "Observation.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace ANYFEATURE_VSLAM
{

class KeyFrame;
class Map;
class Frame;
class Observation;

class MapPoint;
typedef shared_ptr<ANYFEATURE_VSLAM::MapPoint> Pt;


class MapPoint: public std::enable_shared_from_this<MapPoint>
{
public:
    MapPoint(const vec3f &XYZ_, Keyframe pRefKF, shared_ptr<Map> pMap, const FeatureType& featureType);
    MapPoint(const vec3f &XYZ_,  shared_ptr<Map> pMap, Frame* pFrame, const int &idxF, const FeatureType& featureType);
    std::shared_ptr<MapPoint> thisPt() {
        return shared_from_this();
    }

    void SetWorldPos(const vec3f &XYZ_);
    vec3f GetWorldPos();

    vec3f GetNormal();
    Keyframe GetReferenceKeyFrame();

    std::map<KeyframeId,shared_ptr<Observation>> GetObservations();
    int NumberOfObservations();
    int GetNumberOfObservations();
    void increasePointObservability(Keyframe projKeyframe, const KeypointIndex& projIndex);
    void decreasePointObservability(Keyframe projKeyframe, const KeypointIndex& projIndex);

    Keyframe GetCurrentRefKeyframe();
    void SetRefIndex(const KeypointIndex& refIndex_);

    void AddObservation(Keyframe projKeyframe, const KeypointIndex& projIndex);
    void EraseObservation(Keyframe projKeyframe);

    int GetIndexInKeyFrame(Keyframe pKF);
    bool IsInKeyFrame(Keyframe keyframe);

    void SetBadFlag();
    bool isBad();

    void Replace(Pt pMP);
    Pt GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    Pt ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();

    float PredictSize(const float &currentDist);
    float PredictSigma(const float &currentDist);

public:
    PtId ptId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;

    float trackSize;
    float trackSigma;
    float trackViewCos;

    FrameId idLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    vec3f PosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;
    FeatureType featureType;
    KeypointType keypointType;
    DescriptorType descriptorType;

protected:    

     // Position in absolute coordinates
     vec3f XYZ;

     // Keyframes observing the point and associated observation
     std::map<KeyframeId,shared_ptr<Observation>> observations;

     // Mean viewing direction
     vec3f normalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;
     Keyframe refKeyframe;
     KeypointIndex refIndex;

     float refDistance;
     float refSize;
     float refSigma;
     float minDistance;
     float maxDistance;

     // Reference KeyFrame
     Keyframe mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     Pt mpReplaced;

     shared_ptr<Map> mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H

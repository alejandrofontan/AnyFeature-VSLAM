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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ANYFEATURE_VSLAM
{

class Tracking;
class LoopClosing;
class Map;

class LocalMapping
{
public:
    LocalMapping(shared_ptr<Map> pMap, const float bMonocular, const vector<FeatureType>& featureTypes);

    void SetLoopCloser(std::shared_ptr<LoopClosing>  loopCloser_);

    void SetTracker(std::shared_ptr<Tracking> tracker_);

    // Main function
    void Run();

    void InsertKeyFrame(Keyframe pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    vector<double> localMappingTime{};
    vector<FeatureType> featureTypes{};

protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();

    mat3f ComputeF12(Keyframe &pKF1, Keyframe &pKF2);

    mat3f SkewSymmetricMatrix(const vec3f &v);

    bool mbMonocular;

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    shared_ptr<Map> mpMap;

    std::shared_ptr<LoopClosing> loopCloser;
    std::shared_ptr<Tracking> tracker;

    std::list<Keyframe> mlNewKeyFrames;

    Keyframe mpCurrentKeyFrame;

    std::list<Pt> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    const float covisibilityThreshold{0.9f};
    const int minNumObservations{3};
    const float minDistance{0.05f};

   /* map<KeyframeId,set<KeyframeId>> keyframes_to_positions; // <Keyframe Id , Position Ids>
    map<KeyframeId,set<KeyframeId>> positions_to_keyframes; // <Position Id , Keyframe Ids>
    map<KeyframeId,vec3f> trajectoryXYZ; // <Keyframe Id , Position Ids>
    */
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H

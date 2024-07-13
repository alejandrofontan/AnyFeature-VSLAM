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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "FeatureVocabulary.h"
#include "Tracking.h"
#include "MapDrawer.h"

#include "KeyFrameDatabase.h"

#include <thread>
#include <mutex>

#include "g2o/types/types_seven_dof_expmap.h"

namespace ANYFEATURE_VSLAM
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class MapDrawer;

class LoopConnections{
public:
    KeyframeId keyframeId{};
    Keyframe keyframe{};
    std::map<KeyframeId,Keyframe> connections{};
    LoopConnections() = default;
    LoopConnections(const KeyframeId & keyframeId, const Keyframe& keyframe, const std::map<KeyframeId,Keyframe>& connections);
};

class LoopClosing
{
public:

    typedef std::pair<std::map<KeyframeId,Keyframe>,int> ConsistentGroup;
    typedef map<Keyframe ,g2o::Sim3,std::less<Keyframe>,
            Eigen::aligned_allocator<std::pair<Keyframe const, g2o::Sim3> > > KeyFrameAndPose;

public:

    LoopClosing(shared_ptr<Map> pMap, shared_ptr<KeyFrameDatabase> pDB, shared_ptr<Vocabulary> vocabulary,const bool bFixScale);

    void SetTracker(std::shared_ptr<Tracking> tracker);

    void SetLocalMapper(std::shared_ptr<LocalMapping> localMapper_);

    void SetMapDrawer(std::shared_ptr<MapDrawer> mapDrawer_);

    // Main function
    void Run();

    void InsertKeyFrame(Keyframe pKF);

    void RequestReset();

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(unsigned long nLoopKF);

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    void RequestFinish();

    bool isFinished();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    vector<double> loopClosingTime{};
    size_t numOfLoopClosures{0};

protected:

    bool CheckNewKeyFrames();

    bool DetectLoop();

    bool ComputeSim3();

    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

    void CorrectLoop();

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    shared_ptr<Map> mpMap;
    std::shared_ptr<Tracking> tracker;
    std::shared_ptr<MapDrawer> mapDrawer;

    shared_ptr<KeyFrameDatabase> mpKeyFrameDB;
    shared_ptr<Vocabulary> vocabulary;

    std::shared_ptr<LocalMapping> localMapper;

    std::list<Keyframe > mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    Keyframe  mpCurrentKF;
    Keyframe  mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<Keyframe> mvpEnoughConsistentCandidates;
    std::vector<Keyframe> mvpCurrentConnectedKFs;
    std::vector<Pt> mvpCurrentMatchedPoints;
    std::vector<Pt> mvpLoopMapPoints;
    mat4f mScw;
    g2o::Sim3 mg2oScw;

    long unsigned int mLastLoopKFid;

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;


    bool mnFullBAIdx;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H

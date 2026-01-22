#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "FeatureMatcher.h"

#include <mutex>


namespace ANYFEATURE_VSLAM
{

class Tracking;
class LoopClosing;
class Map;

class LocalMapping
{
public:
    LocalMapping(shared_ptr<Map> pMap, const float bMonocular, const vector<FeatureType>& featureTypes, const int& imageWidth, const int& imageHeight);

    void SetLoopCloser(std::shared_ptr<LoopClosing>  loopCloser_){loopCloser = loopCloser_;};
    void SetTracker(std::shared_ptr<Tracking> tracker_){tracker = tracker_;};

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


protected:

    vector<FeatureType> featureTypes{};
    int featureProcessNewKeyframe{0};

    // Parameters for local mapping
    const float CHI2_2DOF{5.991f};

    // KeyFrameCulling()
    const float KEYFRAME_CULLING_COVISIBILITY_THRESHOLD{0.9f};
    const int KEYFRAME_CULLING_MIN_NUM_OBSERVATIONS{3};

    // CreateNewMapPoints()
    const int CREATE_NEW_MAP_POINTS_BEST_COVISIBILITY_KEYFRAMES{20};
    const float CREATE_NEW_MAP_POINTS_RATIO_BASELINE_DEPTH{0.01f};
    const float CREATE_NEW_MAP_POINTS_MIN_COS{0.9998f};

    // MapPointCulling()
    const int MAP_POINT_CULLING_MIN_NUM_OBSERVATIONS{2};

    // SearchInNeighbors()
    const int SEARCH_IN_NEIGHBORS_NUM_KEYFRAMES{20};
    const int SEARCH_IN_NEIGHBORS_NUM_KEYFRAMES_SECOND{5};
    const float SEARCH_IN_NEIGHBORS_RADIUS_TH{5.f};

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();
    void MapPointCulling();
    void SearchInNeighbors(const FeatureType& featureType);
    void KeyFrameCulling();
    void ResetIfRequested();
    bool CheckFinish();
    void SetFinish();

    std::mutex mMutexFinish;
    std::mutex mMutexReset;
    std::mutex mMutexNewKFs;
    std::mutex mMutexStop;
    std::mutex mMutexAccept;

    std::shared_ptr<Map> mpMap;
    std::shared_ptr<LoopClosing> loopCloser;
    std::shared_ptr<Tracking> tracker;
    std::shared_ptr<FeatureMatcher> matcher;

    std::list<Keyframe> mlNewKeyFrames;
    Keyframe mpCurrentKeyFrame;
    std::list<Pt> mlpRecentAddedMapPoints;

    bool mbAbortBA;
    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    bool mbAcceptKeyFrames;
    bool mbMonocular;
    bool mbResetRequested;
    bool mbFinishRequested;
    bool mbFinished;

    const int imageWidth;
    const int imageHeight;

};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H

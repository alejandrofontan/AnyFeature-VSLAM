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


#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <mutex>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "FeatureVocabulary.h"
#include "KeyFrameDatabase.h"
#include "FeatureExtractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"


namespace ANYFEATURE_VSLAM
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:
    Tracking(System* pSys, shared_ptr<Vocabulary> vocabulary, std::shared_ptr<FrameDrawer> pFrameDrawer, std::shared_ptr<MapDrawer> pMapDrawer, shared_ptr<Map> pMap,
             shared_ptr<KeyFrameDatabase> pKFDB,
             const string &settingsYamlFile, const string &feature_settings_yaml_file,
             const int sensor,
             const vector<FeatureType>& featureTypes,
             const bool& fixImageSize = false);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    mat4f GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    mat4f GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    mat4f GrabImageMonocular(Image &im, const double &timestamp);

    void SetLocalMapper(std::shared_ptr<LocalMapping> localMapper_);
    void SetLoopClosing(std::shared_ptr<LoopClosing> loopClosing_);
    void SetViewer(shared_ptr<Viewer> viewer_);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


public:
    VerbosityLevel verbosity{LOW};

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame currentFrame;
    cv::Mat mImGray;

    // Initialization Variables (mono)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<vec3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<mat4f> mlRelativeFramePoses;
    list<Keyframe> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool onlyTracking;

    void Reset();

    size_t numTrackedFrames{2};
    vector<FeatureType> featureTypes{};

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    void loadCameraParameters(const string &settingsYamlFile);
    shared_ptr<FeatureExtractor> getFeatureExtractor(const int& scaleNumFeaturesMonocular_,
                                                     const string &featureSettingsYamlFile);
    static void getGrayImage(cv::Mat& im, const bool& rgb);

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    std::shared_ptr<LocalMapping> localMapper;
    std::shared_ptr<LoopClosing> loopClosing;

    // Features
    shared_ptr<FeatureExtractor> featureExtractorLeft, featureExtractorRight;
    shared_ptr<FeatureExtractor> initFeatureExtractor;

    //BoW
    shared_ptr<Vocabulary> vocabulary;
    shared_ptr<KeyFrameDatabase> keyFrameDB;

    // Initalization (only for monocular)
    shared_ptr<Initializer> mpInitializer;

    //Local Map
    Keyframe refKeyframe;
    std::vector<Keyframe> localKeyframes;
    std::vector<Pt> localPts;
    
    // System
    System* mpSystem;
    
    //Drawers
    std::shared_ptr<Viewer> viewer;
    std::shared_ptr<FrameDrawer> frameDrawer;
    std::shared_ptr<MapDrawer> mapDrawer;

    // Map
    shared_ptr<Map> map;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;
    int w{};
    int h{};

    // New KeyFrame rules (according to fps)
    size_t minFrames;
    size_t maxFrames;
    float fps;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    // Last Frame, KeyFrame and Relocalisation Info
    Keyframe lastKeyFrame;
    Frame lastFrame;
    KeyframeId lastKeyFrameId;
    FrameId lastRelocFrameId;

    //Motion Model
    mat4f mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB{true};

    list<Pt> mlpTemporalPoints;

    // Fix image size to nominal size 307200 pixels
    bool fixImageSize{false};

    //////////////////////////////////////////////// Heuristics
    // Tracking()
    const int scaleNumFeaturesMonocular{2};

    // UpdateLocalKeyFrames()
    const int _maxNumKey_{80};
    const int _bestCovKey_{10};

    // Matching
    const float nnratio_monoInit{0.9f};
    const float nnratio_trackRefKey{0.7f};
    const float nnratio_trackMotModel{0.9f};
    const float nnratio_slp{0.8f};
    const float nnratio_low_reloc{0.75f};
    const float nnratio_high_reloc{0.9f};

    const float radiusTh_high_trackMotModel{15.0f};
    const float radiusTh_low_trackMotModel{7.0f};
    const float radiusTh_scale_trackMotModel{2.0f};
    const float radiusTh_high_slp{5.0f};
    const float radiusTh_medium_slp{3.0f};
    const float radiusTh_low_slp{1.0f};
    const float radiusTh_high_reloc{10.0f};
    const float radiusTh_low_reloc{3.0f};

    const float viewingCosLimit_slp{0.5f};

    //////////////////////////////////////////////// Constants

    // MonocularInitialization()
    const int minKeypointsMonocular{100};
    const float sigmaInitializer{1.0};
    const int minMatches_monoInit{100};

    // CreateInitialMapMonocular()
    const int keyframeTrackedMapPoints{100};

    // TrackReferenceKeyFrame()
    const int minMatches_trackRefKey_high{15};
    const int minMatches_trackRefKey_low{10};

    // UpdateLastFrame()
    const int minNumPoints{100};

    // TrackWithMotionModel()
    const int minMatches_trackMotModel_high{20};
    const int minMatches_trackMotModel_low{10};

    // TrackLocalMap()
    const int minMatches_trackLocalMap_high{50};
    const int minMatches_trackLocalMap_low{30};

    // NeedNewKeyFrame()
    const float refRatio_high_needNewKey{0.9f};
    const float refRatio_medium_needNewKey{0.75f};
    const float refRatio_low_needNewKey{0.4f};
    const int nMinObs_high{3};
    const int nMinObs_low{2};
    const int minNKFs{2};
    const int minTrackedClose{100};
    const int minNonTrackedClose{70};
    const float nRefMatchesDrop{0.25f};
    const int minMatchesInliers{15};
    const int minKeyframesInQueue{3};

    // CreateNewKeyFrame()
    const int minNumPoints_createNewKey{100};

    // SearchLocalPoints()
    const int idSum{2};

    // Relocalization()
    const int minNmatches{15};
    const int nGood_high{50};
    const int nGood_medium{30};
    const int nGood_low{10};

    // # Iterations
    const int numItGBA{20}; // CreateInitialMapMonocular()
    const int numItInitializer{200}; // MonocularInitialization()
    const int numItpSolver{5}; // Relocalization

    // Memory
    const int scaleReserveKey{3}; // UpdateLocalKeyFrames()

    // RANSAC
    const float ransac_probability{0.99};
    const int ransac_minInliers{10};
    const int ransac_maxIterations{300};
    const int ransac_minSet{4};
    const float ransac_epsilon{0.5};
    const float ransac_th2{5.991};

    // loadCameraParameters()
    const float fps0{30.0f};
    const int numFeatures0{1000};

    // GrabImageRGBD()
    const float mDepthMapFactor_th{1e-5};

    // Track()
    const int minKeyframesInMap{5};

    // StereoInitialization()
    const int minKeypointsStereo{500};
};

} //namespace ORB_SLAM

#endif // TRACKING_H

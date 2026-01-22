


#include "Tracking.h"

#include<opencv2/core/core.hpp>
//#include<opencv2/features2d/features2d.hpp>

#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"
#include"Utils.h"

#include<iostream>

#include<mutex>

#include <yaml-cpp/yaml.h>

#include"Feature_orb32.h"
#include"Feature_brisk48.h"
#include"Feature_akaze61.h"
#include"Feature_surf64.h"
#include"Feature_kaze64.h"
#include"Feature_sift128.h"
#include"Feature_r2d2_128.h"
#include"Feature_anyFeatBin.h"
#include"Feature_anyFeatNonBin.h"
#include"Feature_aliked128.h"

using namespace std;

namespace ANYFEATURE_VSLAM
{

Tracking::Tracking(System *pSys, shared_ptr<Vocabulary> vocabulary,
                   std::shared_ptr<FrameDrawer> frameDrawer, std::shared_ptr<MapDrawer> mapDrawer,
                   shared_ptr<Map> map, shared_ptr<KeyFrameDatabase> pKFDB,
                   const string &strCalibrationPath, const string &strSettingPath,
                   const std::map<FeatureType, string>& feature_settings_yaml_file,
                   const int sensor,
                   const vector<FeatureType>& featureTypes,
                   const bool& fixImageSize):
    mState(NO_IMAGES_YET), mSensor(sensor), onlyTracking(false), mbVO(false), vocabulary(vocabulary),
    keyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(nullptr)), mpSystem(pSys), viewer(static_cast<shared_ptr<Viewer>>(nullptr)),
    frameDrawer(frameDrawer), mapDrawer(mapDrawer), map(map), lastRelocFrameId(0), featureTypes(featureTypes), fixImageSize(fixImageSize)
{
    // Load camera parameters from settings yaml file
    Tracking::loadCameraParameters(strCalibrationPath, strSettingPath);

    // Max/Min Frames to insert keyframes and to check relocalisation
    minFrames = 0;
    maxFrames = size_t(fps);

    // Load feature parameters from settings yaml file
    for (auto& ft: featureTypes){
        featureExtractorLeft[ft] = Tracking::getFeatureExtractor(1, std::string(feature_settings_yaml_file.at(ft)), ft);
    }

    if(sensor==System::MONOCULAR)
        initFeatureExtractor[featureTypes[featureInitialization]] = Tracking::getFeatureExtractor(scaleNumFeaturesMonocular , 
            feature_settings_yaml_file.at(featureTypes[featureInitialization]), featureTypes[featureInitialization]); 
    
    matcher = std::make_shared<FeatureMatcher>(w, h);        
}

void Tracking::SetLocalMapper(std::shared_ptr<LocalMapping> localMapper_)
{
    localMapper = localMapper_;
}

void Tracking::SetLoopClosing(std::shared_ptr<LoopClosing> loopClosing_)
{
    loopClosing = loopClosing_;
}

void Tracking::SetViewer(shared_ptr<Viewer> viewer_)
{
    viewer = viewer_;
}


mat4f Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    std::cout << "This function (Tracking::GrabImageStereo) has not been modified yet to work with AnyFeature-VSLAM"<< endl;
    std::terminate();
}


mat4f Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    std::cout << "This function (Tracking::GrabImageStereo) has not been modified yet to work with AnyFeature-VSLAM"<< endl;
    std::terminate();
}


mat4f Tracking::GrabImageMonocular(Image &im, const double &timestamp)
{   
    im.GetGrayImage(mbRGB);
    //if(fixImageSize)
        // im.FixImageSize(w,h);

    mImGray = im.grayImg;

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        currentFrame = Frame(im,timestamp,initFeatureExtractor,vocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        currentFrame = Frame(im,timestamp,featureExtractorLeft,vocabulary,mK,mDistCoef,mbf,mThDepth);
    
    Track();
    return currentFrame.Tcw;
}

void Tracking::Track()
{       
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(map->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization(featureTypes[featureInitialization]);

        frameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {   
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!onlyTracking)
        {   
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();
                bool trackReferenceKeyframe = (mVelocity(3,3) != 1.0f) || (currentFrame.mnId < lastRelocFrameId+2) 
                    || (lastFrame.mnId == refKeyframe->mnFrameId);

                if(trackReferenceKeyframe)
                    bOK = TrackReferenceKeyFrame();
                else
                {
                    bOK = TrackWithMotionModel();
                    bool optimizePose = !bOK;
                    bOK = TrackReferenceKeyFrame(optimizePose);
                }
            }
            else
            {   
                bOK = Relocalization(featureTypes[featureRelocalization]);
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated

            if(mState==LOST)
            {
                bOK = Relocalization(featureTypes[featureRelocalization]);
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(mVelocity(3,3) == 1.0f)
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    std::map<FeatureType, vector<Pt>> vpMPsMM;
                    std::map<FeatureType, vector<bool>> vbOutMM;
                    mat4f TcwMM;
                    if(mVelocity(3,3) == 1.0f)
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = currentFrame.pts;
                        vbOutMM = currentFrame.mvbOutlier;
                        TcwMM = currentFrame.Tcw;
                    }
                    bOKReloc = Relocalization(featureTypes[featureRelocalization]);

                    if(bOKMM && !bOKReloc)
                    {
                        currentFrame.SetPose(TcwMM);
                        currentFrame.pts = vpMPsMM;
                        currentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {   
                            for (auto& [ft, N] : currentFrame.N) {
                                for(int i = 0; i < N; i++)
                                {
                                    if(currentFrame.pts.at(ft)[i] && !currentFrame.mvbOutlier.at(ft)[i])
                                    {
                                        currentFrame.pts.at(ft)[i]->IncreaseFound();
                                    }
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        currentFrame.refKeyframe = refKeyframe;

        // If we have an initial estimation of the camera pose and matching. Track the local map.

        if(!onlyTracking)
        {   
            if(bOK)
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and, therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        frameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            ++numTrackedFrames;

            // Update motion model
            if(lastFrame.Tcw(3,3) == 1.0f)
            {
                mat4f LastTwc{mat4f::Identity()};
                LastTwc.block<3,3>(0,0) = lastFrame.GetRotationInverse();
                LastTwc.block<3,1>(0,3) = lastFrame.GetCameraCenter();
                mVelocity = currentFrame.Tcw * LastTwc;
            }
            else
                mVelocity = mat4f::Zero();

            mapDrawer->SetCurrentCameraPose(currentFrame.Tcw);

            // Clean VO matches
            for (auto& [ft, N] : currentFrame.N) {
                for(int i = 0; i < N; i++)
                {
                    Pt pMP = currentFrame.pts.at(ft)[i];
                    if(pMP)
                        if(pMP->NumberOfObservations() < 1)
                        {
                            currentFrame.mvbOutlier.at(ft)[i] = false;
                            currentFrame.pts.at(ft)[i]=static_cast<Pt>(nullptr);
                        }
                }
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe

            if(NeedNewKeyFrame())
                CreateNewKeyFrame();
            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so, we discard them in the frame.
            for (auto& [ft, N] : currentFrame.N) {
                for(int i =0; i < N; i++)
                {
                    if(currentFrame.pts.at(ft)[i] && currentFrame.mvbOutlier.at(ft)[i])
                        currentFrame.pts.at(ft)[i]=static_cast<Pt>(nullptr);
                }
            }
        }
        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(map->KeyFramesInMap() <= minKeyframesInMap)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!currentFrame.refKeyframe)
            currentFrame.refKeyframe = refKeyframe;

        lastFrame = Frame(currentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterward.
    if(currentFrame.Tcw(3,3) == 1.0f)
    {
        mat4f Tcr = currentFrame.Tcw * currentFrame.refKeyframe->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(refKeyframe);
        mlFrameTimes.push_back(currentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

    // std::cout << "Tracking::Track: end" << std::endl;
    if (emergencyKeyframe){
        std::cout << "Tracking::Track: emergency keyframe triggered, waiting for local mapping to be idle..." << std::endl;    
        lock.unlock();
        bool localMappingIdle = localMapper->AcceptKeyFrames();
        while(!localMappingIdle)
        {
            // Wait until Local Mapping is idle
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            localMappingIdle = localMapper->AcceptKeyFrames();
        }
        emergencyKeyframe = false;
        std::cout << "Tracking::Track: local mapping is idle, inserting emergency keyframe..." << std::endl;
    }


}


void Tracking::StereoInitialization()
{
    std::cout << "This function (Tracking::StereoInitialization) has not been modified yet to work with AnyFeature-VSLAM"<< endl;
    std::terminate();
}

void Tracking::MonocularInitialization(const FeatureType& featureType)
{   
    const DescriptorType descriptorType = GetDescriptorType(featureType);
    if(!mpInitializer)
    {   
        // Set Reference Frame
        if(currentFrame.mvKeys[featureType].size() > minKeypointsMonocular)
        {
            mInitialFrame = Frame(currentFrame);
            lastFrame = Frame(currentFrame);
            mvbPrevMatched.resize(currentFrame.mvKeysUn[featureType].size());
            for(size_t i = 0; i < currentFrame.mvKeysUn[featureType].size(); i++)
                mvbPrevMatched[i] = currentFrame.mvKeysUn.at(featureType)[i].pt;

            mpInitializer =  make_shared<Initializer>(currentFrame, sigmaInitializer, numItInitializer, featureType);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)currentFrame.mvKeys[featureType].size() <= minKeypointsMonocular)
        {
            mpInitializer = nullptr;
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }
        // Find correspondences

        int nmatches = matcher->SearchForInitialization(mInitialFrame, currentFrame, mvbPrevMatched, mvIniMatches, featureType);

             // Check if there are enough correspondences
        if(nmatches < minMatches_monoInit)
        {
            mpInitializer = nullptr;
            return;
        }
        mat3f Rcw{}; // Current Camera Rotation
        vec3f tcw {}; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
        if(mpInitializer->Initialize(currentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(mat4f::Identity());
            mat4f Tcw{mat4f::Identity()};
            Tcw.block<3,3>(0,0) = Rcw;
            Tcw.block<3,1>(0,3) = tcw;
            currentFrame.SetPose(Tcw);
            CreateInitialMapMonocular(featureType);
        }
    }
}

void Tracking::CreateInitialMapMonocular(const FeatureType& featureType)
{   
    // Create KeyFrames
    Keyframe pKFini = make_shared<KeyFrame>(mInitialFrame, map, keyFrameDB);
    Keyframe pKFcur = make_shared<KeyFrame>(currentFrame, map, keyFrameDB);


    pKFini->ComputeBoW(featureType);
    pKFcur->ComputeBoW(featureType);

    // Insert KFs in the map
    map->AddKeyFrame(pKFini);
    map->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        Pt pMP = pKFcur->CreateMonocularMapPoint(mvIniP3D[i],KeypointIndex(mvIniMatches[i]),
                                                 pKFini,KeypointIndex(i), featureType);
        //Fill Current Frame structure
        currentFrame.pts[featureType][mvIniMatches[i]] = pMP;
        currentFrame.mvbOutlier[featureType][mvIniMatches[i]] = false;

        //Add to Map
        map->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << map->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(map,numItGBA);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1) < keyframeTrackedMapPoints)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    mat4f Tc2w = pKFcur->GetPose();
    Tc2w.block<3,1>(0,3) *= invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<Pt> vpAllMapPoints = pKFini->GetMapPointMatches(featureType);
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            Pt pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    localMapper->InsertKeyFrame(pKFini);
    localMapper->InsertKeyFrame(pKFcur);

    currentFrame.SetPose(pKFcur->GetPose());
    lastKeyFrameId=currentFrame.mnId;
    lastKeyFrame = pKFcur;

    localKeyframes.push_back(pKFcur);
    localKeyframes.push_back(pKFini);
    localPts = map->GetAllMapPoints();
    refKeyframe = pKFcur;
    currentFrame.refKeyframe = pKFcur;

    lastFrame = Frame(currentFrame);

    map->SetReferenceMapPoints(localPts);

    mapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    map->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{   
    for (auto& [ft, pts] : lastFrame.pts) {
        for(int i = 0; i<lastFrame.N.at(ft); i++)
        {
            Pt pMP = lastFrame.pts.at(ft)[i];

            if(pMP)
            {
                Pt pRep = pMP->GetReplaced();
                if(pRep)
                {
                    lastFrame.pts.at(ft)[i] = pRep;
                }
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame(const bool& optimizePose)
{   
    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we set up a PnP solver
    std::map<FeatureType, std::vector<Pt>> vpMapPointMatches;

    std::map<FeatureType, int> nmatches_ft = matcher->SearchBruteForce(refKeyframe, currentFrame, vpMapPointMatches, refKeyframe->featureTypes);
    int nmatches = 0;
    for (auto& [ft, N] : currentFrame.N) {
        if (nmatches_ft[ft] == 0)
            continue;

        nmatches += nmatches_ft[ft];
        currentFrame.pts[ft] = vpMapPointMatches[ft];   
        currentFrame.N[ft] = static_cast<int>(vpMapPointMatches[ft].size());
        currentFrame.mvbOutlier[ft] = vector<bool>(vpMapPointMatches[ft].size(), false);
    }
    
    if (!optimizePose)
        return true;

    if(nmatches < minMatches_trackRefKey_high)
            return false;
         
    currentFrame.SetPose(lastFrame.Tcw);
    Optimizer::PoseOptimization(&currentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (auto& [ft, N] : currentFrame.N) {
        for(int i =0; i<currentFrame.N.at(ft); i++)
        {
            if(currentFrame.pts.at(ft)[i])
            {
                if(currentFrame.mvbOutlier.at(ft)[i])
                {
                    Pt pMP = currentFrame.pts.at(ft)[i];

                    currentFrame.pts.at(ft)[i]=static_cast<Pt>(nullptr);
                    currentFrame.mvbOutlier.at(ft)[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->idLastFrameSeen = currentFrame.mnId;
                    nmatches--;
                }
                else if(currentFrame.pts.at(ft)[i]->NumberOfObservations() > 0)
                    nmatchesMap++;
            }
        }
    }
    return nmatchesMap >= minMatches_trackRefKey_low;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    Keyframe pRef = lastFrame.refKeyframe;
    mat4f Tlr = mlRelativeFramePoses.back();

    lastFrame.SetPose(Tlr * pRef->GetPose());
}

bool Tracking::TrackWithMotionModel()
{

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    currentFrame.SetPose(mVelocity * lastFrame.Tcw);

    for (auto& [ft, N] : currentFrame.N) {
        fill(currentFrame.pts.at(ft).begin(),currentFrame.pts.at(ft).end(),static_cast<Pt>(nullptr));
    }

    int nmatches= matcher->SearchBruteForce(currentFrame, lastFrame, currentFrame.featureTypes);
    
    if(nmatches < minMatches_trackMotModel_high)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&currentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (auto& [ft, N_] : currentFrame.N) {
        for(int i{0}; i < N_; i++)
        {
            if(currentFrame.pts.at(ft)[i])
            {
                if(currentFrame.mvbOutlier.at(ft)[i])
                {
                    Pt pMP = currentFrame.pts.at(ft)[i];

                    currentFrame.pts.at(ft)[i] = static_cast<Pt>(nullptr);
                    currentFrame.mvbOutlier.at(ft)[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->idLastFrameSeen = currentFrame.mnId;
                    nmatches--;
                }
                else if(currentFrame.pts.at(ft)[i]->NumberOfObservations() > 0)
                    nmatchesMap++;
            }
        }    
    }
    
    if(onlyTracking)
    {
        mbVO = nmatchesMap < minMatches_trackMotModel_low;
        return nmatches > minMatches_trackMotModel_high;
    }

    return nmatchesMap >= minMatches_trackMotModel_low;
}

bool Tracking::TrackLocalMap()
{

    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.
    UpdateLocalMap();
    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&currentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for (auto& [ft, pts] : currentFrame.pts) {
        for(int i = 0; i < pts.size(); i++)
        {   
            if(currentFrame.pts.at(ft)[i])
            {
                if(!currentFrame.mvbOutlier.at(ft)[i])
                {
                    currentFrame.pts.at(ft)[i]->IncreaseFound();
                    if(!onlyTracking)
                    {
                        if(currentFrame.pts.at(ft)[i]->NumberOfObservations() > 0)
                            mnMatchesInliers++;
                    }
                    else
                        mnMatchesInliers++;
                }
                else if(mSensor==System::STEREO)
                    currentFrame.pts.at(ft)[i] = static_cast<Pt>(nullptr);

            }
        }
    }
    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(currentFrame.mnId < lastRelocFrameId + maxFrames && mnMatchesInliers < minMatches_trackLocalMap_high)
        return false;

    if(mnMatchesInliers < minMatches_trackLocalMap_low)
        return false;
    else
        return true;
}

    bool Tracking::NeedNewKeyFrame()
    {
        if(onlyTracking)
            return false;

        // If Local Mapping is freezed by a Loop Closure do not insert keyframes
        if(localMapper->isStopped() || localMapper->stopRequested())
            return false;

        const size_t numKeyframesInMap = map->KeyFramesInMap();

        // Do not insert keyframes if not enough frames have passed from last relocalisation
        if((currentFrame.mnId < lastRelocFrameId + maxFrames) && (numKeyframesInMap > maxFrames))
            return false;

        // Tracked MapPoints in the reference keyframe
        int nMinObs = nMinObs_high;
        if(numKeyframesInMap <= size_t(minNKFs))
            nMinObs = nMinObs_low;
        int nRefMatches = refKeyframe->TrackedMapPoints(nMinObs);

        // Local Mapping accept keyframes?
        bool localMappingIdle = localMapper->AcceptKeyFrames();
        // if(localMappingIdle)
        //      return true;

        // Check how many "close" points are being tracked and how many could be potentially created.
        int nNonTrackedClose = 0;
        int nTrackedClose= 0;

        bool bNeedToInsertClose = (nTrackedClose < minTrackedClose) && (nNonTrackedClose > minNonTrackedClose);

        // Thresholds
        float thRefRatio = refRatio_medium_needNewKey;
        if(numKeyframesInMap < size_t(minNKFs))
            thRefRatio = refRatio_low_needNewKey;

        if(mSensor == System::MONOCULAR)
            thRefRatio = refRatio_high_needNewKey;

        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        const bool c1a = currentFrame.mnId >= lastKeyFrameId + maxFrames;
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        const bool c1b = (currentFrame.mnId >= lastKeyFrameId + minFrames && localMappingIdle);
        // Condition 1c: tracking is weak
        // int numInliers = 0;
        // int numTotal = 0;
        // for (const auto& [ft, N] : currentFrame.N) {
        //     numTotal += N;
        //     for (const auto& pt: currentFrame.pts.at(ft)) {
        //         if (pt && (!pt->isBad()))
        //             numInliers++;
        //     }
        // }
        // std::cout << "Inlier ratio: " << float(numInliers)/float(numTotal) << std::endl;
        // std::cout << "numInliers: " << numInliers << ", numTotal: " << numTotal << std::endl;

        const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers < nRefMatches * nRefMatchesDrop || bNeedToInsertClose) ;
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose) && mnMatchesInliers > minMatchesInliers);
        // if(c2)
        //     std::cout << "mnMatchesInliers: " << mnMatchesInliers << ", nRefMatches: " << nRefMatches << ", thRefRatio: " 
        //         << mnMatchesInliers / float(nRefMatches) << std::endl;
        if(bNeedToInsertClose)
            std::terminate();
        if(c1c)
            std::terminate();

        if(c2)
        //if((c1a||c1b||c1c)&&c2)
        {
            // If the mapping accepts keyframes, insert keyframe.
            // Otherwise, send a signal to interrupt BA
            if(localMappingIdle)
            {
                return true;
            }
            else
            {   
                if(mnMatchesInliers < nRefMatches * 0.5f)
                    emergencyKeyframe = true;
                return false;   
            }
        }
        else
            return false;
    }

    void Tracking::CreateNewKeyFrame(){
        if(!localMapper->SetNotStop(true))
            return;

        Keyframe keyframe = make_shared<KeyFrame>(currentFrame,map,keyFrameDB);

        refKeyframe = keyframe;
        currentFrame.refKeyframe = keyframe;

        localMapper->InsertKeyFrame(keyframe);
        localMapper->SetNotStop(false);
        lastKeyFrameId = currentFrame.mnId;
        lastKeyFrame = keyframe;
    }

    void Tracking::SearchLocalPoints()
    {   
        // Do not search map points already matched
        for (const auto& [ft, N] : currentFrame.N) {
            for(auto& pt: currentFrame.pts.at(ft)){
                if(pt && !pt->isBad()){
                    pt->IncreaseVisible();
                    pt->idLastFrameSeen = currentFrame.mnId;
                    pt->mbTrackInView = false;
                }
                else
                    pt = nullptr;
            }
        }

        // Project points in frame and check its visibility
        int nToMatch=0;
        for(auto& pt: localPts){
            if(pt->idLastFrameSeen == currentFrame.mnId)
                continue;
            if(pt->isBad())
                continue;

            // Project (this fills MapPoint variables for matching)
            if(currentFrame.isInFrustum(pt,viewingCosLimit_slp)){
                pt->IncreaseVisible();
                nToMatch++;
            }
        }

        if(nToMatch > 0){
            float radiusTh = radiusTh_low_slp;
            if(mSensor == System::RGBD)
                radiusTh = radiusTh_medium_slp;

            // If the camera has been relocalised recently, perform a coarser search
            if(currentFrame.mnId < lastRelocFrameId + idSum)
                radiusTh = radiusTh_high_slp;

            matcher->SearchByProjection(currentFrame, localPts);//, radiusTh);

        }
    }

    void Tracking::UpdateLocalMap()
    {
        // This is for visualization
        map->SetReferenceMapPoints(localPts);
        // Update
        UpdateLocalKeyFrames();
        UpdateLocalPoints();
    }

    void Tracking::UpdateLocalPoints()
    {
        localPts.clear();
        set<PtId> ptIds{};
        for(const auto& keyframe : localKeyframes){
            for(const auto& ft: featureTypes){
                const vector<Pt> pts = keyframe->GetMapPointMatches(ft);
                for(const auto& pt : pts){
                    if(!pt)
                        continue;
                    if (ptIds.find(pt->ptId) != ptIds.end())
                        continue;
                    if(!pt->isBad()){
                        localPts.push_back(pt);
                        ptIds.insert(pt->ptId);
                    }
                }
            }
        }
    }

    void Tracking::UpdateLocalKeyFrames()
    {
        // Each map point vote for the keyframes in which it has been observed
        set<KeyframeId> keyframeIds{};
        {
            int maxObs = 0;
            auto keyframeMaxObs = static_cast<Keyframe>(nullptr);

            std::map<KeyframeId,int> keyframeCounter;
            std::map<KeyframeId,Keyframe> keyframes;

            for (auto& [ft, pts] : currentFrame.pts) {
                for(auto& pt: pts){
                    if(pt && !pt->isBad()) {
                        const std::map<KeyframeId, Obs> observations = pt->GetObservations();
                        for (const auto &[keyId, obs]: observations) {
                            keyframeCounter[keyId]++;
                            keyframes[keyId] = obs->projKeyframe;
                        }
                    }
                    else
                        pt = nullptr;
                }
            }
            if(keyframeCounter.empty())
                return;

            // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
            localKeyframes.clear();
            localKeyframes.reserve(scaleReserveKey * keyframeCounter.size());
            for (const auto &[keyId, keyframe]: keyframes) {
                if(keyframe->isBad())
                    continue;

                int keyFrameCount = keyframeCounter[keyframe->keyId];
                if(keyFrameCount > maxObs){
                    maxObs = keyFrameCount;
                    keyframeMaxObs = keyframe;
                }

                localKeyframes.push_back(keyframe);
                keyframeIds.insert(keyframe->keyId);
            }

            if(keyframeMaxObs){
                refKeyframe = keyframeMaxObs;
                currentFrame.refKeyframe = refKeyframe;
            }
        }

        // Include also some not-already-included keyframes that are neighbors to already-included keyframes
        for(const auto& keyframe: localKeyframes){

            // Limit the number of keyframes
            if(int(localKeyframes.size()) > _maxNumKey_)
                break;

            const vector<Keyframe> neighbors = keyframe->GetBestCovisibilityKeyFrames(_bestCovKey_);
            for(const auto& neighbor: neighbors){
                if(!neighbor->isBad()){
                    if (keyframeIds.find(neighbor->keyId) == keyframeIds.end()){
                        localKeyframes.push_back(neighbor);
                        keyframeIds.insert(neighbor->keyId);
                        break;
                    }
                }
            }

            const set<Keyframe> childs = keyframe->GetChilds();
            for(const auto& child: childs){
                if(!child->isBad()){
                    if (keyframeIds.find(child->keyId) == keyframeIds.end()){
                        localKeyframes.push_back(child);
                        keyframeIds.insert(child->keyId);
                        break;
                    }
                }
            }

            Keyframe parent = keyframe->GetParent();
            if(parent and !parent->isBad()){
                if (keyframeIds.find(parent->keyId) == keyframeIds.end()){
                    localKeyframes.push_back(parent);
                    keyframeIds.insert(parent->keyId);
                    break;
                }
            }
        }
    }

bool Tracking::Relocalization(const FeatureType& featureType)
{
    // Compute Bag of Words Vector
    currentFrame.ComputeBoW(featureType);

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<Keyframe> vpCandidateKFs = keyFrameDB->DetectRelocalizationCandidates(&currentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we set up a PnP solver

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<std::map<FeatureType, vector<Pt>>> vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        Keyframe pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            std::map<FeatureType, int> nmatches_ft = matcher->SearchBruteForce(pKF, currentFrame, vvpMapPointMatches[i], std::vector<FeatureType>{featureType});
            if(nmatches_ft[featureType] < minNmatches)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(currentFrame,vvpMapPointMatches[i][featureType], featureType);
                pSolver->SetRansacParameters(ransac_probability,ransac_minInliers,ransac_maxIterations,ransac_minSet,ransac_epsilon,ransac_th2);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw_tmp = pSolver->iterate(numItpSolver,bNoMore,vbInliers,nInliers);
            mat4f Tcw{mat4f::Zero()};
            if(!Tcw_tmp.empty())
                Tcw = Converter::toMatrix4f(Tcw_tmp);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(Tcw(3,3) == 1.0f)
            {
                currentFrame.Tcw = Tcw;
                set<Pt> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        currentFrame.pts.at(featureType)[j]=vvpMapPointMatches[i][featureType][j];
                        sFound.insert(vvpMapPointMatches[i][featureType][j]);
                    }
                    else
                        currentFrame.pts.at(featureType)[j]=nullptr;
                }

                int nGood = Optimizer::PoseOptimization(&currentFrame);

                if(nGood < nGood_low)
                    continue;

                for(int io =0; io<currentFrame.N.at(featureType); io++)
                    if(currentFrame.mvbOutlier.at(featureType)[io])
                        currentFrame.pts.at(featureType)[io]=static_cast<Pt>(nullptr);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood < nGood_high)
                {
                    int nadditional = matcher->SearchByProjection(currentFrame,vpCandidateKFs[i],sFound,radiusTh_high_reloc, true, featureType);

                    if(nadditional+nGood >= nGood_high)
                    {
                        nGood = Optimizer::PoseOptimization(&currentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood > nGood_medium && nGood < nGood_high)
                        {
                            sFound.clear();
                            for(int ip =0; ip<currentFrame.N.at(featureType); ip++)
                                if(currentFrame.pts.at(featureType)[ip])
                                    sFound.insert(currentFrame.pts.at(featureType)[ip]);
                            nadditional =matcher->SearchByProjection(currentFrame,vpCandidateKFs[i],sFound,radiusTh_low_reloc,false, featureType);

                            // Final optimization
                            if(nGood+nadditional >= nGood_high)
                            {
                                nGood = Optimizer::PoseOptimization(&currentFrame);

                                for(int io =0; io < currentFrame.N.at(featureType); io++)
                                    if(currentFrame.mvbOutlier.at(featureType)[io])
                                        currentFrame.pts.at(featureType)[io]=nullptr;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood >= nGood_high)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        lastRelocFrameId = currentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(viewer)
    {
        viewer->RequestStop();
        while(!viewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    localMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    loopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    keyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    map->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        mpInitializer = nullptr;
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(viewer)
        viewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    onlyTracking = flag;
}

void Tracking::loadCameraParameters(const string &strCalibrationPath, const string &strSettingPath){

    YAML::Node settings = YAML::LoadFile(strSettingPath);
    YAML::Node calibration = YAML::LoadFile(strCalibrationPath);
    const YAML::Node& cameras = calibration["cameras"];

    std::string cam_name;
    cam_name = settings["cam_mono"].as<std::string>();
    YAML::Node cam{};
    for (size_t i{0}; i < cameras.size(); ++i){
        if (cameras[i]["cam_name"].as<std::string>() == cam_name){
            cam = cameras[i];
            break;
        }
    }
    
    mK = (cv::Mat_<float>(3, 3) << cam["focal_length"][0].as<float>(), 0.0f, cam["principal_point"][0].as<float>(),
            0.0f, cam["focal_length"][1].as<float>(), cam["principal_point"][1].as<float>(),
            0.0f, 0.0f, 1.0f);
                
    w = cam["image_dimension"][0].as<int>();
    h = cam["image_dimension"][1].as<int>();

    // if(fixImageSize){
        // float ratio = float(w) / float(h);
        // int new_h = (int) sqrt(307200.f / ratio);
        // int new_w = (int) (float(new_h) * ratio);
        // float conv_ratio_h = float(new_h)/float(h);
        // float conv_ratio_w = float(new_w)/float(w);

        // w = new_w;     
        // h = new_h;

        // mK.at<float>(0,0) *= conv_ratio_w;
        // mK.at<float>(1,1) *= conv_ratio_h;
        // mK.at<float>(0,2) *= conv_ratio_w;
        // mK.at<float>(1,2) *= conv_ratio_h;
    // }

    // Distortion coefficients
    mDistCoef = cv::Mat::zeros(4,1,CV_32F);
    if (cam["distortion_type"] && cam["distortion_coefficients"]) {
        std::vector<float> dist_coeffs_vec = cam["distortion_coefficients"].as<std::vector<float>>();
        mDistCoef = cv::Mat(dist_coeffs_vec.size(), 1, CV_32F, dist_coeffs_vec.data()).clone(); 
    }

    // Camera frequence (hz)
    fps = cam["fps"].as<float>();
    if(fps == 0)
        fps = fps0;

    // RGB order
    bool mbRGB = cam["cam_type"].as<std::string>() != "bgr";

    // Load settings file
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    // Stereo baseline
    // if(mSensor==System::STEREO || mSensor==System::RGBD)
    // {
    //     cout << endl  << "[Tracking.cc] RGB-D/STEREO Parameters: " << strSettingPath << endl;

    //     mbf = float(fCalibration["Stereo.bf"]) * fx;
    //     cout << "- bf: " << mbf << endl;

    //     mThDepth = mbf *(float)fSettings["ThDepth"] / fx;
    //     cout << "- Depth Threshold: " << mThDepth << endl;

    // }

    // if(mSensor==System::RGBD)
    //     {
    //         mDepthMapFactor = float(fCalibration["Depth0.factor"]);
    //         cout << "- Depth Map Factor: " << mDepthMapFactor << endl;
    //         if(fabs(mDepthMapFactor)<1e-5)
    //             mDepthMapFactor=1;
    //         else
    //             mDepthMapFactor = 1.0f/mDepthMapFactor;
    //     }

    // Print camera parameters
    cout << endl << "Camera Parameters: " << endl;
    cout << "- cam_name: " << cam["cam_name"].as<std::string>() << endl;
    cout << "- cam_type: " << cam["cam_type"].as<std::string>() << endl;
    cout << "- cam_model: " << cam["cam_model"].as<std::string>() << endl;
    if (cam["distortion_type"] && cam["distortion_coefficients"])
        cout << "- distortion_type: " << cam["distortion_type"].as<std::string>() << endl;
    cout << "- fx: " << mK.at<float>(0,0) << endl;
    cout << "- fy: " << mK.at<float>(1,1) << endl;
    cout << "- cx: " << mK.at<float>(0,2) << endl;
    cout << "- cy: " << mK.at<float>(1,2) << endl;
    if (cam["distortion_type"] && cam["distortion_coefficients"]){
        cout << "- distortion_coefficients: " << mDistCoef.t() << endl;
    }
    cout << "- fps: " << cam["fps"].as<float>() << endl;
    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // if(mSensor == System::STEREO || mSensor == System::RGBD) {
    //     cout << "mbf: " << mbf << endl;
    //     cout << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    // }
    // if(mSensor == System::RGBD)
    // {
    //     cout << "mDepthMapFactor: " << mDepthMapFactor << endl;
    // }
}

shared_ptr<FeatureExtractor> Tracking::getFeatureExtractor(const int& scaleNumFeaturesMonocular_,
                                                           const string &featureSettingsYamlFile,
                                                            const FeatureType& featureType){

    const KeypointType keypointType = GetKeypointType(featureType);
    const DescriptorType descriptorType = GetDescriptorType(featureType);

    shared_ptr<FeatureExtractorSettings> extractorSettings = make_shared<FeatureExtractorSettings>(keypointType, descriptorType, featureSettingsYamlFile);
    extractorSettings->maxNumFeatures *= scaleNumFeaturesMonocular_;

    switch (keypointType) {
        case KEYP_ALIKED128:{
            return std::make_shared<FeatureExtractor_aliked128>(extractorSettings);
        }
        case KEYP_ANYFEATNONBIN:{
            return std::make_shared<FeatureExtractor_anyFeatNonBin>(extractorSettings);
        }
        case KEYP_ANYFEATBIN:{
            return std::make_shared<FeatureExtractor_anyFeatBin>(extractorSettings);
        }
        case KEYP_R2D2:{
            return std::make_shared<FeatureExtractor_r2d2_128>(extractorSettings);
        }
        case KEYP_SIFT:{
            return std::make_shared<FeatureExtractor_sift128>(extractorSettings);
        }
        case KEYP_KAZE:{
            return std::make_shared<FeatureExtractor_kaze64>(extractorSettings);
        }
        case KEYP_SURF:{
            return std::make_shared<FeatureExtractor_surf64>(extractorSettings);
        }
        case KEYP_AKAZE:{
            return std::make_shared<FeatureExtractor_akaze61>(extractorSettings);
        }
        case KEYP_BRISK:{
            return std::make_shared<FeatureExtractor_brisk48>(extractorSettings);
        }
        case KEYP_ORB: {
            return std::make_shared<FeatureExtractor_orb32>(extractorSettings);
        }
    }
    return nullptr;
}

void Tracking::getGrayImage(cv::Mat& im , const bool& rgb){
    if(im.channels() == 3)
    {
        if(rgb)
            cvtColor(im,im,CV_RGB2GRAY);
        else
            cvtColor(im,im,CV_BGR2GRAY);
    }
    else if(im.channels() == 4)
    {
        if(rgb)
            cvtColor(im,im,CV_RGBA2GRAY);
        else
            cvtColor(im,im,CV_BGRA2GRAY);
    }
}

} //namespace ORB_SLAM

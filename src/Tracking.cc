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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
//#include<opencv2/features2d/features2d.hpp>

#include"FeatureMatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"
#include"Utils.h"

#include<iostream>

#include<mutex>

#include"Feature_orb32.h"
#include"Feature_brisk48.h"
#include"Feature_akaze61.h"
#include"Feature_surf64.h"
#include"Feature_kaze64.h"
#include"Feature_sift128.h"
#include"Feature_r2d2_128.h"
#include"Feature_anyFeatBin.h"
#include"Feature_anyFeatNonBin.h"

using namespace std;

namespace ANYFEATURE_VSLAM
{

Tracking::Tracking(System *pSys, shared_ptr<Vocabulary> vocabulary,
                   std::shared_ptr<FrameDrawer> frameDrawer, std::shared_ptr<MapDrawer> mapDrawer,
                   shared_ptr<Map> map, shared_ptr<KeyFrameDatabase> pKFDB,
                   const string &settingsYamlFile, const string &feature_settings_yaml_file,
                   const int sensor,
                   const vector<FeatureType>& featureTypes,
                   const bool& fixImageSize):
    mState(NO_IMAGES_YET), mSensor(sensor), onlyTracking(false), mbVO(false), vocabulary(vocabulary),
    keyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(nullptr)), mpSystem(pSys), viewer(static_cast<shared_ptr<Viewer>>(nullptr)),
    frameDrawer(frameDrawer), mapDrawer(mapDrawer), map(map), lastRelocFrameId(0), featureTypes(featureTypes), fixImageSize(fixImageSize)
{
    // Load camera parameters from settings yaml file
    Tracking::loadCameraParameters(settingsYamlFile);

    // Max/Min Frames to insert keyframes and to check relocalisation
    minFrames = 0;
    maxFrames = size_t(fps);

    // Load feature parameters from settings yaml file
    featureExtractorLeft = Tracking::getFeatureExtractor(1, feature_settings_yaml_file);

    //if(sensor==System::STEREO)
        //featureExtractorRight = std::make_shared<FeatureExtractor>(numFeatures,extractorSettings);

    if(sensor==System::MONOCULAR)
        initFeatureExtractor = Tracking::getFeatureExtractor(scaleNumFeaturesMonocular , "none");
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
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;
    getGrayImage(mImGray,mbRGB);
    getGrayImage(imGrayRight,mbRGB);

    currentFrame = Frame(mImGray,imGrayRight,timestamp,featureExtractorLeft,featureExtractorRight,vocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return currentFrame.Tcw;
}


mat4f Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    getGrayImage(mImGray,mbRGB);

    if((fabs(mDepthMapFactor-1.0f) > mDepthMapFactor_th) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    currentFrame = Frame(mImGray,imDepth,timestamp,featureExtractorLeft,vocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return currentFrame.Tcw;
}


mat4f Tracking::GrabImageMonocular(Image &im, const double &timestamp)
{
    im.GetGrayImage(mbRGB);
    if(fixImageSize)
        im.FixImageSize(w,h);

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
            MonocularInitialization();

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

                if((mVelocity(3,3) != 1.0f) || currentFrame.mnId<lastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                bOK = Relocalization();
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated

            if(mState==LOST)
            {
                bOK = Relocalization();
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
                    vector<Pt> vpMPsMM;
                    vector<bool> vbOutMM;
                    mat4f TcwMM;
                    if(mVelocity(3,3) == 1.0f)
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = currentFrame.pts;
                        vbOutMM = currentFrame.mvbOutlier;
                        TcwMM = currentFrame.Tcw;
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        currentFrame.SetPose(TcwMM);
                        currentFrame.pts = vpMPsMM;
                        currentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<currentFrame.N; i++)
                            {
                                if(currentFrame.pts[i] && !currentFrame.mvbOutlier[i])
                                {
                                    currentFrame.pts[i]->IncreaseFound();
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
            for(int i=0; i<currentFrame.N; i++)
            {
                Pt pMP = currentFrame.pts[i];
                if(pMP)
                    if(pMP->NumberOfObservations() < 1)
                    {
                        currentFrame.mvbOutlier[i] = false;
                        currentFrame.pts[i]=static_cast<Pt>(nullptr);
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
            for(int i=0; i<currentFrame.N;i++)
            {
                if(currentFrame.pts[i] && currentFrame.mvbOutlier[i])
                    currentFrame.pts[i]=static_cast<Pt>(nullptr);
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
}


void Tracking::StereoInitialization()
{
    if(currentFrame.N > minKeypointsStereo)
    {
        // Set Frame pose to the origin
        currentFrame.SetPose(mat4f::Identity());

        // Create KeyFrame
        Keyframe pKFini = make_shared<KeyFrame>(currentFrame,map,keyFrameDB);

        // Insert KeyFrame in the map
        map->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<currentFrame.N;i++)
        {
            float z = currentFrame.mvDepth[i];
            if(z>0)
            {
                vec3f x3D = currentFrame.UnprojectStereo(i);
                Pt pNewMP = pKFini->CreateMapPoint(x3D,KeypointIndex (i),featureTypes[0]);
                currentFrame.pts[i]=pNewMP;
            }
        }

        cout << "New map created with " << map->MapPointsInMap() << " points" << endl;

        localMapper->InsertKeyFrame(pKFini);

        lastFrame = Frame(currentFrame);
        lastKeyFrameId=currentFrame.mnId;
        lastKeyFrame = pKFini;

        localKeyframes.push_back(pKFini);
        localPts = map->GetAllMapPoints();
        refKeyframe = pKFini;
        currentFrame.refKeyframe = pKFini;

        map->SetReferenceMapPoints(localPts);

        map->mvpKeyFrameOrigins.push_back(pKFini);

        mapDrawer->SetCurrentCameraPose(currentFrame.Tcw);

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{
    const FeatureType featureType = featureTypes[0];
    const DescriptorType descriptorType = GetDescriptorType(featureType);
    if(!mpInitializer)
    {
        // Set Reference Frame
        if(currentFrame.mvKeys.size() > minKeypointsMonocular)
        {
            mInitialFrame = Frame(currentFrame);
            lastFrame = Frame(currentFrame);
            mvbPrevMatched.resize(currentFrame.mvKeysUn.size());
            for(size_t i=0; i<currentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=currentFrame.mvKeysUn[i].pt;


            mpInitializer =  make_shared<Initializer>(currentFrame,sigmaInitializer,numItInitializer);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)currentFrame.mvKeys.size() <= minKeypointsMonocular)
        {
            mpInitializer = nullptr;
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        FeatureMatcher matcher(nnratio_monoInit, true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,currentFrame,mvbPrevMatched,mvIniMatches,100, descriptorType);

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

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    Keyframe pKFini = make_shared<KeyFrame>(mInitialFrame,map,keyFrameDB);
    Keyframe pKFcur = make_shared<KeyFrame>(currentFrame,map,keyFrameDB);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

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
                                                 pKFini,KeypointIndex(i), featureTypes[0]);
        //Fill Current Frame structure
        currentFrame.pts[mvIniMatches[i]] = pMP;
        currentFrame.mvbOutlier[mvIniMatches[i]] = false;

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
    vector<Pt> vpAllMapPoints = pKFini->GetMapPointMatches();
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
    for(int i =0; i<lastFrame.N; i++)
    {
        Pt pMP = lastFrame.pts[i];

        if(pMP)
        {
            Pt pRep = pMP->GetReplaced();
            if(pRep)
            {
               lastFrame.pts[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    currentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we set up a PnP solver
    FeatureMatcher matcher(nnratio_trackRefKey, true);
    vector<Pt> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(refKeyframe,currentFrame,vpMapPointMatches);

    if(nmatches < minMatches_trackRefKey_high)
        return false;

    currentFrame.pts = vpMapPointMatches;
    currentFrame.SetPose(lastFrame.Tcw);

    Optimizer::PoseOptimization(&currentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<currentFrame.N; i++)
    {
        if(currentFrame.pts[i])
        {
            if(currentFrame.mvbOutlier[i])
            {
                Pt pMP = currentFrame.pts[i];

                currentFrame.pts[i]=static_cast<Pt>(nullptr);
                currentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->idLastFrameSeen = currentFrame.mnId;
                nmatches--;
            }
            else if(currentFrame.pts[i]->NumberOfObservations() > 0)
                nmatchesMap++;
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

    if(lastKeyFrameId==lastFrame.mnId || mSensor==System::MONOCULAR || !onlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(lastFrame.N);
    for(int i=0; i<lastFrame.N;i++)
    {
        float z = lastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        Pt pMP = lastFrame.pts[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->NumberOfObservations() < 1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            vec3f x3D = lastFrame.UnprojectStereo(i);
            Pt pNewMP = make_shared<MapPoint>(x3D,map,&lastFrame,i, featureTypes[0]);

            lastFrame.pts[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints > minNumPoints)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    FeatureMatcher matcher(nnratio_trackMotModel, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    currentFrame.SetPose(mVelocity * lastFrame.Tcw);

    fill(currentFrame.pts.begin(),currentFrame.pts.end(),static_cast<Pt>(nullptr));

    // Project points seen in previous frame
    float radiusTh;
    if(mSensor!=System::STEREO)
        radiusTh = radiusTh_high_trackMotModel;
    else
        radiusTh = radiusTh_low_trackMotModel;
    int nmatches = matcher.SearchByProjection(currentFrame,lastFrame,radiusTh,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches < minMatches_trackMotModel_high)
    {
        fill(currentFrame.pts.begin(),currentFrame.pts.end(),static_cast<Pt>(nullptr));
        nmatches = matcher.SearchByProjection(currentFrame,lastFrame, radiusTh_scale_trackMotModel * radiusTh, mSensor==System::MONOCULAR);
    }

    if(nmatches < minMatches_trackMotModel_high)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&currentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<currentFrame.N; i++)
    {
        if(currentFrame.pts[i])
        {
            if(currentFrame.mvbOutlier[i])
            {
                Pt pMP = currentFrame.pts[i];

                currentFrame.pts[i]=static_cast<Pt>(nullptr);
                currentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->idLastFrameSeen = currentFrame.mnId;
                nmatches--;
            }
            else if(currentFrame.pts[i]->NumberOfObservations() > 0)
                nmatchesMap++;
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
    for(int i=0; i<currentFrame.N; i++)
    {
        if(currentFrame.pts[i])
        {
            if(!currentFrame.mvbOutlier[i])
            {
                currentFrame.pts[i]->IncreaseFound();
                if(!onlyTracking)
                {
                    if(currentFrame.pts[i]->NumberOfObservations() > 0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                currentFrame.pts[i] = static_cast<Pt>(nullptr);

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
        if(numKeyframesInMap <= minNKFs)
            nMinObs = nMinObs_low;
        int nRefMatches = refKeyframe->TrackedMapPoints(nMinObs);

        // Local Mapping accept keyframes?
        bool localMappingIdle = localMapper->AcceptKeyFrames();

        // Check how many "close" points are being tracked and how many could be potentially created.
        int nNonTrackedClose = 0;
        int nTrackedClose= 0;
        if((mSensor==System::RGBD) or (mSensor==System::STEREO))
        {
            for(int i = 0; i < currentFrame.N; i++)
            {
                if(currentFrame.mvDepth[i] > 0 && currentFrame.mvDepth[i] < mThDepth)
                {
                    if(currentFrame.pts[i] && !currentFrame.mvbOutlier[i])
                        nTrackedClose++;
                    else
                        nNonTrackedClose++;
                }
            }
        }

        bool bNeedToInsertClose = (nTrackedClose < minTrackedClose) && (nNonTrackedClose > minNonTrackedClose);

        // Thresholds
        float thRefRatio = refRatio_medium_needNewKey;
        if(numKeyframesInMap < minNKFs)
            thRefRatio = refRatio_low_needNewKey;

        if(mSensor == System::MONOCULAR)
            thRefRatio = refRatio_high_needNewKey;

        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        const bool c1a = currentFrame.mnId >= lastKeyFrameId + maxFrames;
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        const bool c1b = (currentFrame.mnId >= lastKeyFrameId + minFrames && localMappingIdle);
        //Condition 1c: tracking is weak
        const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers < nRefMatches * nRefMatchesDrop || bNeedToInsertClose) ;
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers > minMatchesInliers);

        if((c1a||c1b||c1c)&&c2)
        {
            // If the mapping accepts keyframes, insert keyframe.
            // Otherwise, send a signal to interrupt BA
            if(localMappingIdle)
            {
                return true;
            }
            else
            {
                localMapper->InterruptBA();
                if(mSensor!=System::MONOCULAR)
                {
                    if(localMapper->KeyframesInQueue() < minKeyframesInQueue)
                        return true;
                    else
                        return false;
                }
                else
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

        if((mSensor == System::RGBD) or (mSensor == System::STEREO)){
            currentFrame.UpdatePoseMatrices();

            // We sort points by the measured depth by the stereo/RGBD sensor.
            // We create all those MapPoints whose depth < mThDepth.
            // If there are less than 100 close points we create the 100 closest.
            vector<pair<float,int> > vDepthIdx;
            vDepthIdx.reserve(currentFrame.N);
            for(int i=0; i<currentFrame.N; i++){
                float z = currentFrame.mvDepth[i];
                if(z>0){
                    vDepthIdx.push_back(make_pair(z,i));
                }
            }

            if(!vDepthIdx.empty()){
                sort(vDepthIdx.begin(),vDepthIdx.end());

                int nPoints = 0;
                for(size_t j=0; j<vDepthIdx.size();j++){
                    int i = vDepthIdx[j].second;

                    bool bCreateNew = false;

                    Pt pMP = currentFrame.pts[i];
                    if(!pMP)
                        bCreateNew = true;
                    else if(pMP->NumberOfObservations() < 1){
                        bCreateNew = true;
                        currentFrame.pts[i] = static_cast<Pt>(nullptr);
                    }

                    if(bCreateNew){
                        vec3f x3D = currentFrame.UnprojectStereo(i);
                        Pt pNewMP = keyframe->CreateMapPoint(x3D,KeypointIndex (i), featureTypes[0]);

                        currentFrame.pts[i]=pNewMP;
                        nPoints++;
                    }
                    else{
                        nPoints++;
                    }

                    if(vDepthIdx[j].first>mThDepth && nPoints > minNumPoints_createNewKey)
                        break;
                }
            }
        }

        localMapper->InsertKeyFrame(keyframe);
        localMapper->SetNotStop(false);
        lastKeyFrameId = currentFrame.mnId;
        lastKeyFrame = keyframe;
    }

    void Tracking::SearchLocalPoints()
    {
        // Do not search map points already matched
        for(auto& pt: currentFrame.pts){
            if(pt && !pt->isBad()){
                pt->IncreaseVisible();
                pt->idLastFrameSeen = currentFrame.mnId;
                pt->mbTrackInView = false;
            }
            else
                pt = nullptr;
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
            FeatureMatcher matcher(nnratio_slp);
            float radiusTh = radiusTh_low_slp;
            if(mSensor == System::RGBD)
                radiusTh = radiusTh_medium_slp;

            // If the camera has been relocalised recently, perform a coarser search
            if(currentFrame.mnId < lastRelocFrameId + idSum)
                radiusTh = radiusTh_high_slp;

            matcher.SearchByProjection(currentFrame,localPts, radiusTh);
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
            const vector<Pt> pts = keyframe->GetMapPointMatches();
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

    void Tracking::UpdateLocalKeyFrames()
    {
        // Each map point vote for the keyframes in which it has been observed
        set<KeyframeId> keyframeIds{};
        {
            int maxObs = 0;
            auto keyframeMaxObs = static_cast<Keyframe>(nullptr);

            std::map<KeyframeId,int> keyframeCounter;
            std::map<KeyframeId,Keyframe> keyframes;
            for(auto& pt: currentFrame.pts){
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
            if(localKeyframes.size() > _maxNumKey_)
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

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    currentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<Keyframe> vpCandidateKFs = keyFrameDB->DetectRelocalizationCandidates(&currentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we set up a PnP solver
    FeatureMatcher matcher(nnratio_low_reloc, true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<Pt> > vvpMapPointMatches;
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
            int nmatches = matcher.SearchByBoW(pKF,currentFrame,vvpMapPointMatches[i]);
            if(nmatches < minNmatches)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(currentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(ransac_probability,ransac_minInliers,ransac_maxIterations,ransac_minSet,ransac_epsilon,ransac_th2);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    FeatureMatcher matcher2(nnratio_high_reloc, true);

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
                        currentFrame.pts[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        currentFrame.pts[j]=nullptr;
                }

                int nGood = Optimizer::PoseOptimization(&currentFrame);

                if(nGood < nGood_low)
                    continue;

                for(int io =0; io<currentFrame.N; io++)
                    if(currentFrame.mvbOutlier[io])
                        currentFrame.pts[io]=static_cast<Pt>(nullptr);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood < nGood_high)
                {
                    int nadditional =matcher2.SearchByProjection(currentFrame,vpCandidateKFs[i],sFound,radiusTh_high_reloc, true);

                    if(nadditional+nGood >= nGood_high)
                    {
                        nGood = Optimizer::PoseOptimization(&currentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood > nGood_medium && nGood < nGood_high)
                        {
                            sFound.clear();
                            for(int ip =0; ip<currentFrame.N; ip++)
                                if(currentFrame.pts[ip])
                                    sFound.insert(currentFrame.pts[ip]);
                            nadditional =matcher2.SearchByProjection(currentFrame,vpCandidateKFs[i],sFound,radiusTh_low_reloc,false);

                            // Final optimization
                            if(nGood+nadditional >= nGood_high)
                            {
                                nGood = Optimizer::PoseOptimization(&currentFrame);

                                for(int io =0; io<currentFrame.N; io++)
                                    if(currentFrame.mvbOutlier[io])
                                        currentFrame.pts[io]=nullptr;
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

void Tracking::loadCameraParameters(const string &settingsYamlFile){

    // Load camera parameters from settings yaml file
    cv::FileStorage fSettings(settingsYamlFile, cv::FileStorage::READ);

    if (fSettings["Camera.fx"].empty() || fSettings["Camera.fy"].empty() ||
        fSettings["Camera.cx"].empty() || fSettings["Camera.cy"].empty() ||
        fSettings["Camera.k1"].empty() || fSettings["Camera.k2"].empty() ||
        fSettings["Camera.p1"].empty() || fSettings["Camera.p2"].empty() ||
        fSettings["Camera.w"].empty() || fSettings["Camera.h"].empty()
        ){

        printError("Tracking", "Undefined intrinsics (fx, fy, cx, cy, k1 ,k2, p1, p2, w, h) in " + settingsYamlFile);
        terminate();
    }

    // Calibration matrix
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    w = fSettings["Camera.w"];
    h = fSettings["Camera.h"];

    if(fixImageSize){
        float ratio = float(w) / float(h);
        int new_h = (int) sqrt(307200.f / ratio);
        int new_w = (int) (float(new_h) * ratio);
        float conv_ratio_h = float(new_h)/float(h);
        float conv_ratio_w = float(new_w)/float(w);

        w = new_w;
        h = new_h;
        fx *= conv_ratio_w;
        fy *= conv_ratio_h;
        cx *= conv_ratio_w;
        cy *= conv_ratio_h;
    }

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);


    // Distortion coefficients
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

    // Camera frequence (hz)
    fps = fSettings["Camera.fps"];
    if(fps == 0)
        fps = fps0;

    // RGB order
    if (!fSettings["Camera.RGB"].empty())
        mbRGB = bool(int(fSettings["Camera.RGB"]));

    // Stereo baseline
    if(mSensor == System::STEREO || mSensor == System::RGBD)
    {
        mbf = fSettings["Camera.bf"];
        mThDepth = mbf * (float)fSettings["ThDepth"]/fx;
    }

    if(mSensor == System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor) < mDepthMapFactor_th)
            mDepthMapFactor = 1.0f;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

    // Print camera parameters
    cout << "\nCamera Parameters: " << endl;
    cout << "- fx: " << fx << " , fy: " << fy << " , cx: " << cx << " , cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << " , k2: " << DistCoef.at<float>(1) << " , p1: " <<
            DistCoef.at<float>(2) << " , p2: " << DistCoef.at<float>(3);
    if(DistCoef.rows==5)
        cout << " , k3: " << DistCoef.at<float>(4);
    cout << endl;

    cout << "- w: " << w << " , h: " << h << endl;
    cout << "- fps: " << fps << endl;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    if(mSensor == System::STEREO || mSensor == System::RGBD) {
        cout << "mbf: " << mbf << endl;
        cout << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }
    if(mSensor == System::RGBD)
    {
        cout << "mDepthMapFactor: " << mDepthMapFactor << endl;
    }
}

shared_ptr<FeatureExtractor> Tracking::getFeatureExtractor(const int& scaleNumFeaturesMonocular_,
                                                           const string &featureSettingsYamlFile){
    //cv::FileStorage fSettings(featureSettingsYamlFile, cv::FileStorage::READ);
    cout << endl  << "Feature Extractor Parameters: " << endl;

    const FeatureType featureType = featureTypes[0];
    const KeypointType keypointType = GetKeypointType(featureType);
    const DescriptorType descriptorType = GetDescriptorType(featureType);

    // Scale number of features with the resolution of the image
    int numFeatures = ((2000.0 - 1000.0)/ (1241.0 * 376.0 - 640.0 * 480.0)) * (float(w * h) - 640.0 * 480.0) + numFeatures0;
    if (numFeatures >  2000)
        numFeatures = 2000;
    else if (numFeatures < 1000)
        numFeatures = 1000;
    numFeatures *= scaleNumFeaturesMonocular_;
    cout << "- Number of Features: " << numFeatures << endl;

    shared_ptr<FeatureExtractorSettings> extractorSettings = make_shared<FeatureExtractorSettings>(keypointType, descriptorType, featureSettingsYamlFile);
    switch (keypointType) {
        case KEYP_ANYFEATNONBIN:{
            return std::make_shared<FeatureExtractor_anyFeatNonBin>(numFeatures,extractorSettings);
        }
        case KEYP_ANYFEATBIN:{
            return std::make_shared<FeatureExtractor_anyFeatBin>(numFeatures,extractorSettings);
        }
        case KEYP_R2D2:{
            return std::make_shared<FeatureExtractor_r2d2_128>(numFeatures,extractorSettings);
        }
        case KEYP_SIFT:{
            return std::make_shared<FeatureExtractor_sift128>(numFeatures,extractorSettings);
        }
        case KEYP_KAZE:{
            return std::make_shared<FeatureExtractor_kaze64>(numFeatures,extractorSettings);
        }
        case KEYP_SURF:{
            return std::make_shared<FeatureExtractor_surf64>(numFeatures,extractorSettings);
        }
        case KEYP_AKAZE:{
            return std::make_shared<FeatureExtractor_akaze61>(numFeatures,extractorSettings);
        }
        case KEYP_BRISK:{
            return std::make_shared<FeatureExtractor_brisk48>(numFeatures,extractorSettings);
        }
        case KEYP_ORB: {
            return std::make_shared<FeatureExtractor_orb32>(numFeatures,extractorSettings);
        }
    }
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

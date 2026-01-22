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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "FeatureMatcher.h"
#include "Optimizer.h"
#include "Converter.h"

#include<mutex>
#include <Eigen/Core>
#include <Eigen/SVD>

namespace ANYFEATURE_VSLAM
{

LocalMapping::LocalMapping(shared_ptr<Map> pMap, const float bMonocular, const vector<FeatureType>& featureTypes, const int& imageWidth, const int& imageHeight):
    mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true), featureTypes(featureTypes),
    imageWidth(imageWidth), imageHeight(imageHeight)
{
    matcher = std::make_shared<FeatureMatcher>(imageWidth, imageHeight);
}

void LocalMapping::Run()
{

    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();

            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();

            // Check recent MapPoints
            MapPointCulling();

            // Triangulate new MapPoints
            CreateNewMapPoints();

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                for (const auto& [feat, N_] : mpCurrentKeyFrame->N) {
                    SearchInNeighbors(feat);
                }
            }

            mbAbortBA = false;
            //if(!CheckNewKeyFrames() && !stopRequested())
            if(!CheckNewKeyFrames())
            {
                // Local BA
                if(mpMap->KeyFramesInMap()>2)
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpMap);

                // Check redundant local Keyframes
                KeyFrameCulling();
            }
            loopCloser->InsertKeyFrame(mpCurrentKeyFrame);
            std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
            double t_duration = std::chrono::duration_cast<std::chrono::duration<double> >(t_end - t_start).count();
            localMappingTime.push_back(t_duration);
        }
        else if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())
                break;
        }
        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

    SetFinish();
}

void LocalMapping::InsertKeyFrame(Keyframe pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW(featureTypes[featureProcessNewKeyframe]);

    // Associate MapPoints to the new keyframe and update normal and descriptor
    for(const auto& feat: mpCurrentKeyFrame->featureTypes){
        const vector<Pt> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches(feat);

        for(size_t i=0; i<vpMapPointMatches.size(); i++)
        {
            Pt pMP = vpMapPointMatches[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                    {
                        pMP->AddObservation(mpCurrentKeyFrame, i);
                    }
                    else // this can only happen for new stereo points inserted by the Tracking
                    {
                        mlpRecentAddedMapPoints.push_back(pMP);
                    }
                }
            }
        }    
    }
    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpMap->AddKeyFrame(mpCurrentKeyFrame);

}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<Pt>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->keyId;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        Pt pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->NumberOfObservations() <= MAP_POINT_CULLING_MIN_NUM_OBSERVATIONS)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}

void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    const vector<Keyframe > vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(CREATE_NEW_MAP_POINTS_BEST_COVISIBILITY_KEYFRAMES);

    // Init Current Keyframe
    mat4f Twc1, Tcw1;
    mat3f Rwc1, Rcw1;
    vec3f twc1, tcw1;
    mpCurrentKeyFrame->getFullPose(Twc1, Rwc1, twc1, Tcw1, Rcw1, tcw1);

    float fx1, fy1, cx1, cy1, invfx1, invfy1;
    mpCurrentKeyFrame->getFullIntrinsics(fx1, fy1, cx1, cy1, invfx1, invfy1);

    // Search matches with epipolar restriction and triangulate
    std::map<FeatureType, int> newMapPoints;
    for(size_t i{0}; i < vpNeighKFs.size(); i++)
    {
        if(i > 0 && CheckNewKeyFrames())
            return;

        // Init Second Keyframe
        Keyframe  pKF2 = vpNeighKFs[i];
        mat4f Twc2, Tcw2;
        mat3f Rwc2, Rcw2;
        vec3f twc2, tcw2;
        pKF2->getFullPose(Twc2, Rwc2, twc2, Tcw2, Rcw2, tcw2);

        float fx2, fy2, cx2, cy2, invfx2, invfy2;
        pKF2->getFullIntrinsics(fx2, fy2, cx2, cy2, invfx2, invfy2);

        // Check first that baseline is not too short
        vec3f vBaseline = twc2 - twc1;
        const float baseline = vBaseline.norm();

        const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
        const float ratioBaselineDepth = baseline/medianDepthKF2;

        if(ratioBaselineDepth < CREATE_NEW_MAP_POINTS_RATIO_BASELINE_DEPTH)
            continue;

        // Search matches that fullfil epipolar constraint
        std::map<FeatureType, vector<pair<size_t,size_t>>> vMatchedIndices;

        matcher->SearchForTriangulation(mpCurrentKeyFrame, pKF2, vMatchedIndices, mpCurrentKeyFrame->featureTypes);

        for(auto& [featureType, N_]: pKF2->N){                    
            // Triangulate each match
            const int nmatches = vMatchedIndices.at(featureType).size();
            for(int ikp{0}; ikp < nmatches; ikp++)
            {
                const int &idx1 = vMatchedIndices.at(featureType)[ikp].first;
                const int &idx2 = vMatchedIndices.at(featureType)[ikp].second;

                const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn.at(featureType)[idx1];
                const cv::KeyPoint &kp2 = pKF2->mvKeysUn.at(featureType)[idx2];

                // Check parallax between rays
                vec3f xn1{(kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0f};
                vec3f xn2{(kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0f};

                vec3f ray1 = Rwc1 * xn1;
                vec3f ray2 = Rwc2 * xn2;
                const float cosParallaxRays = ray1.dot(ray2)/(ray1.norm() * ray2.norm());
                const float sinParallaxRays = ray1.cross(ray2).norm() / (ray1.norm() * ray2.norm());

                float cosParallaxStereo = cosParallaxRays+1;

                vec3f x3D;
                const float sinThr = std::sqrt(1.0f - CREATE_NEW_MAP_POINTS_MIN_COS * CREATE_NEW_MAP_POINTS_MIN_COS);
                //if(true)
                if(cosParallaxRays > 0 && (sinParallaxRays > sinThr))
                //if(cosParallaxRays > 0 && (cosParallaxRays < CREATE_NEW_MAP_POINTS_MIN_COS))
                {
                    Eigen::Matrix<float, 4, 4> A;
                        A.row(0) = xn1(0) * Tcw1.row(2) - Tcw1.row(0);
                        A.row(1) = xn1(1) * Tcw1.row(2) - Tcw1.row(1);
                        A.row(2) = xn2(0) * Tcw2.row(2) - Tcw2.row(0);
                        A.row(3) = xn2(1) * Tcw2.row(2) - Tcw2.row(1);

                    Eigen::JacobiSVD<Eigen::Matrix<float,4,4>> svd(
                        A, Eigen::ComputeFullV
                    );  

                    const Eigen::Matrix<float,4,4>& V = svd.matrixV();
                    Eigen::Vector4f x_h = V.col(3); 

                    const float w = x_h(3);
                    if (std::abs(w) < 1e-12f)
                        continue;

                    x3D = x_h.head<3>() / w; 
                }
                else
                    continue; //No stereo and very low parallax

                //Check triangulation in front of cameras
                float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
                if(z1<=0)
                    continue;

                float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
                if(z2<=0)
                    continue;

                //Check reprojection error in first keyframe
                const float &sigmaSquare1 = mpCurrentKeyFrame->GetKeyPt1DSigma2(idx1, featureType);
                const float x1 = Rcw1.row(0).dot(x3D) + tcw1(0);
                const float y1 = Rcw1.row(1).dot(x3D) + tcw1(1);
                const float invz1 = 1.0f / z1;


                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if((errX1*errX1+errY1*errY1) > CHI2_2DOF * sigmaSquare1)
                    continue;
                
                // Check reprojection error in second keyframe
                const float sigmaSquare2 = pKF2->GetKeyPt1DSigma2(idx2, featureType);
                const float x2 = Rcw2.row(0).dot(x3D) + tcw2(0);
                const float y2 = Rcw2.row(1).dot(x3D) + tcw2(1);
                const float invz2 = 1.0f / z2;

                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2) > CHI2_2DOF * sigmaSquare2)
                    continue;
                
                // Check scale consistency
                vec3f normal1 = x3D - twc1;
                float dist1 = normal1.norm();

                vec3f normal2 = x3D - twc2;
                float dist2 = normal2.norm();

                if(dist1 == 0 || dist2 == 0)
                    continue;

                // Triangulation is succesfull
                newMapPoints[featureType]++;   
                Pt pMP = mpCurrentKeyFrame->CreateMonocularMapPoint(x3D, KeypointIndex(idx1),
                                                                    pKF2,  KeypointIndex(idx2),
                                                                    featureType);                                        
                mlpRecentAddedMapPoints.push_back(pMP);
            }
        }
    }
}

void LocalMapping::SearchInNeighbors(const FeatureType& featureType)
{
    // Retrieve neighbor keyframes
    const vector<Keyframe > vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(SEARCH_IN_NEIGHBORS_NUM_KEYFRAMES);
    vector<Keyframe > vpTargetKFs;
    for(vector<Keyframe >::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        Keyframe  pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->keyId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->keyId;

        // Extend to some second neighbors
        const vector<Keyframe > vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(SEARCH_IN_NEIGHBORS_NUM_KEYFRAMES_SECOND);
        for(vector<Keyframe >::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            Keyframe  pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->keyId || pKFi2->keyId == mpCurrentKeyFrame->keyId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }

    // Search matches by projection from current KF in target KFs
    //FeatureMatcher matcher;
    vector<Pt> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches(featureType);
    for(vector<Keyframe >::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        Keyframe  pKFi = *vit;
        if (featureType == FEAT_ORB)
            matcher->Fuse(pKFi,vpMapPointMatches, SEARCH_IN_NEIGHBORS_RADIUS_TH, featureType);
    }

    // Search matches by projection from target KFs in current KF
    vector<Pt> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<Keyframe >::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        Keyframe  pKFi = *vitKF;

        vector<Pt> vpMapPointsKFi = pKFi->GetMapPointMatches(featureType);

        for(vector<Pt>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            Pt pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->keyId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->keyId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    if (featureType == FEAT_ORB)
        matcher->Fuse(mpCurrentKeyFrame,vpFuseCandidates, SEARCH_IN_NEIGHBORS_RADIUS_TH, featureType);

    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches(featureType);
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        Pt pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }
    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{

    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<Keyframe > vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for(vector<Keyframe >::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        Keyframe  pKF = *vit;
        for(const auto feat: pKF->featureTypes){
            if(pKF->keyId == 0)
                continue;
            const vector<Pt> vpMapPoints = pKF->GetMapPointMatches(feat);

            int nObs = KEYFRAME_CULLING_MIN_NUM_OBSERVATIONS;
            const int thObs=nObs;
            int nRedundantObservations=0;
            int nMPs=0;
            for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
            {
                Pt pMP = vpMapPoints[i];

                if(pMP)
                {
                    if(!pMP->isBad())
                    {
                        FeatureType featType = pMP->featureType;
                        nMPs++;
                        if(pMP->NumberOfObservations() > thObs)
                        {
                            //const float keyPtSize = pKF->GetKeyPtSize(KeypointIndex(i), featType);
                            const map<KeyframeId , Obs> observations = pMP->GetObservations();
                            int nObs=0;
                            for(auto& obs: observations)
                            {
                                Keyframe keyframe_i = obs.second->projKeyframe;
                                if(keyframe_i->keyId == pKF->keyId)
                                    continue;
    // #ifndef VANILLA_ORB_SLAM2
                                if(keyframe_i->isBad())
                                    continue;
    // #endif
                                //const float keyPtSize_i = keyframe_i->GetKeyPtSize(obs.second->projIndex, featType);

                                //if(keyPtSize_i <= keyPtSize * keyframe_i->sizeTolerance)
                                //{
                                    nObs++;
                                    if(nObs>=thObs)
                                        break;
                                //}
                            }
                            if(nObs>=thObs)
                            {
                                nRedundantObservations++;
                            }
                        }
                    }
                }
            }
            if(nRedundantObservations > KEYFRAME_CULLING_COVISIBILITY_THRESHOLD * nMPs){  
                if ((int(pKF->mnFrameId) % 10) != 0 && (int(pKF->keyId) % 5) != 0){
                    pKF->SetBadFlag();
                    break;
                }  
            }
    }
    }
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM

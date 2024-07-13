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

namespace ANYFEATURE_VSLAM
{

LocalMapping::LocalMapping(shared_ptr<Map> pMap, const float bMonocular, const vector<FeatureType>& featureTypes):
    mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true), featureTypes(featureTypes)
{
}

void LocalMapping::SetLoopCloser(std::shared_ptr<LoopClosing>  loopCloser_)
{
    loopCloser = loopCloser_;
}

void LocalMapping::SetTracker(std::shared_ptr<Tracking> tracker_)
{
    tracker = tracker_;
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
                SearchInNeighbors();
            }

            mbAbortBA = false;

            if(!CheckNewKeyFrames() && !stopRequested())
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
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<Pt> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

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

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpMap->AddKeyFrame(mpCurrentKeyFrame);

    //
    /*vec3f XYZRef =  mpCurrentKeyFrame->GetCameraCenter();
#ifndef VANILLA_ORB_SLAM2
    for(auto& position : trajectoryXYZ ){
        vec3f XYZ =  position.second;
        float distance3D = (XYZRef - XYZ).norm();
        if(distance3D < minDistance){
            if(keyframes_to_positions.find(position.first) != keyframes_to_positions.end()){
                positions_to_keyframes[mpCurrentKeyFrame->keyId].insert(position.first);
                keyframes_to_positions[mpCurrentKeyFrame->keyId].insert(position.first);
            }
            positions_to_keyframes[position.first].insert(mpCurrentKeyFrame->keyId);
            keyframes_to_positions[position.first].insert(mpCurrentKeyFrame->keyId);
        }
    }

#endif
    trajectoryXYZ[mpCurrentKeyFrame->keyId] = XYZRef;*/
}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<Pt>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->keyId;

    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

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
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->NumberOfObservations() <= cnThObs)
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
    const FeatureType featureType = featureTypes[0];
    const KeypointType keypointType = GetKeypointType(featureType);
    const DescriptorType descriptorType = GetDescriptorType(featureType);

    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<Keyframe > vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    FeatureMatcher matcher(0.6, false);

    mat3f Rcw1 = mpCurrentKeyFrame->GetRotation();
    mat3f Rwc1 = Rcw1.transpose();
    vec3f tcw1 = mpCurrentKeyFrame->GetTranslation();
    mat34f Tcw1{};
    Tcw1.block<3,3>(0,0) = Rcw1;
    Tcw1.block<3,1>(0,3) = tcw1;
    vec3f Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f * mpCurrentKeyFrame->sizeTolerance;

    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        Keyframe  pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        vec3f Ow2 = pKF2->GetCameraCenter();
        vec3f vBaseline = Ow2 - Ow1;
        const float baseline = vBaseline.norm();

        if(!mbMonocular)
        {
            if(baseline<pKF2->mb)
            continue;
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        mat3f F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;

        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false, descriptorType);

        mat3f Rcw2 = pKF2->GetRotation();
        mat3f Rwc2 = Rcw2.transpose();
        vec3f tcw2 = pKF2->GetTranslation();
        mat34f Tcw2{};
        Tcw2.block<3,3>(0,0) = Rcw2;
        Tcw2.block<3,1>(0,3) = tcw2;

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;

            // Check parallax between rays
            vec3f xn1{(kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0f};
            vec3f xn2{(kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0f};

            vec3f ray1 = Rwc1 * xn1;
            vec3f ray2 = Rwc2 * xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(ray1.norm() * ray2.norm());

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            vec3f x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                mat4f A_tmp{};
                A_tmp.row(0) = xn1(0) * Tcw1.row(2) - Tcw1.row(0);
                A_tmp.row(1) = xn1(1) * Tcw1.row(2) - Tcw1.row(1);
                A_tmp.row(2) = xn2(0) * Tcw2.row(2) - Tcw2.row(0);
                A_tmp.row(3) = xn2(1) * Tcw2.row(2) - Tcw2.row(1);

                cv::Mat A(4,4,CV_32F);
                A = Converter::toCvMat(A_tmp).clone();

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                cv::Mat x3D_tmp;
                x3D_tmp = vt.row(3).t();

                if(x3D_tmp.at<float>(3) == 0)
                    continue;

                // Euclidean coordinates
                x3D_tmp = x3D_tmp.rowRange(0,3)/x3D_tmp.at<float>(3);
                x3D = Converter::toVector3f(x3D_tmp);
            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);                
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
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
            const float &sigmaSquare1 = mpCurrentKeyFrame->GetKeyPt1DSigma2(idx1);
            const float x1 = Rcw1.row(0).dot(x3D) + tcw1(0);
            const float y1 = Rcw1.row(1).dot(x3D) + tcw1(1);
            const float invz1 = 1.0f / z1;

            if(!bStereo1)
            {
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->GetKeyPt1DSigma2(idx2);
            const float x2 = Rcw2.row(0).dot(x3D) + tcw2(0);
            const float y2 = Rcw2.row(1).dot(x3D) + tcw2(1);
            const float invz2 = 1.0f / z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            vec3f normal1 = x3D - Ow1;
            float dist1 = normal1.norm();

            vec3f normal2 = x3D-Ow2;
            float dist2 = normal2.norm();

            if(dist1==0 || dist2==0)
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame->GetKeyPtSize(idx1) / pKF2->GetKeyPtSize(idx2);

            if(ratioDist * ratioFactor < ratioOctave || ratioDist > ratioOctave * ratioFactor)
                continue;

            // Triangulation is succesfull
            Pt pMP = mpCurrentKeyFrame->CreateMonocularMapPoint(x3D, KeypointIndex(idx1),
                                                                pKF2,  KeypointIndex(idx2),
                                                                featureType);

            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;
        }
    }
}

void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<Keyframe > vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<Keyframe > vpTargetKFs;
    for(vector<Keyframe >::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        Keyframe  pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->keyId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->keyId;

        // Extend to some second neighbors
        const vector<Keyframe > vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<Keyframe >::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            Keyframe  pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->keyId || pKFi2->keyId == mpCurrentKeyFrame->keyId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    FeatureMatcher matcher;
    vector<Pt> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<Keyframe >::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        Keyframe  pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches,3.0f);
    }

    // Search matches by projection from target KFs in current KF
    vector<Pt> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<Keyframe >::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        Keyframe  pKFi = *vitKF;

        vector<Pt> vpMapPointsKFi = pKFi->GetMapPointMatches();

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

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates,3.0f);


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
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

mat3f LocalMapping::ComputeF12(Keyframe &pKF1, Keyframe &pKF2)
{
    mat3f R1w = pKF1->GetRotation();
    vec3f t1w = pKF1->GetTranslation();
    mat3f R2w = pKF2->GetRotation();
    vec3f t2w = pKF2->GetTranslation();

    mat3f R12 = R1w * R2w.transpose();
    vec3f t12 = -R1w * R2w.transpose() * t2w + t1w;

    mat3f t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return Converter::toMatrix3f(K1.t().inv()) * t12x * R12 * Converter::toMatrix3f(K2.inv());
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
        if(pKF->keyId == 0)
            continue;
        const vector<Pt> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = minNumObservations;
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
                    if(!mbMonocular)
                    {
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    if(pMP->NumberOfObservations() > thObs)
                    {
                        const float keyPtSize = pKF->GetKeyPtSize(KeypointIndex(i));
                        const map<KeyframeId , Obs> observations = pMP->GetObservations();
                        int nObs=0;
                        for(auto& obs: observations)
                        {
                            Keyframe keyframe_i = obs.second->projKeyframe;
                            if(keyframe_i->keyId == pKF->keyId)
                                continue;
#ifndef VANILLA_ORB_SLAM2
                            if(keyframe_i->isBad())
                                continue;
#endif
                            const float keyPtSize_i = keyframe_i->GetKeyPtSize(obs.second->projIndex);

                            if(keyPtSize_i <= keyPtSize * keyframe_i->sizeTolerance)
                            {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }
        if(nRedundantObservations > covisibilityThreshold * nMPs){
//#ifndef VANILLA_ORB_SLAM2
            /*bool cull = true;
            if(keyframes_to_positions[pKF->keyId].empty())
                cull = false;

            for(auto& position: keyframes_to_positions[pKF->keyId]){ // Positions associated with keyframe pKF
                if(positions_to_keyframes[position].size() < 2) // Keyframes associated with position
                {
                    cull = false;
                    continue;
                }
            }

            if(cull){
                for(auto& position: keyframes_to_positions[pKF->keyId]){ // Positions associated with keyframe pKF
                    positions_to_keyframes[position].erase(pKF->keyId); // Keyframes associated with position
                }
                keyframes_to_positions[pKF->keyId].clear();
                pKF->SetBadFlag();
            }*/
//#else
            pKF->SetBadFlag();
//#endif
        }
    }
}

mat3f LocalMapping::SkewSymmetricMatrix(const vec3f &v)
{
    mat3f M;
    M <<     0.0f   , -v(2),  v(1),
          v(2),    0.0f    , -v(0),
         -v(1), v(0) ,     0.0f;

    return M;
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

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

#include "MapPoint.h"
#include "FeatureMatcher.h"

#include<mutex>

namespace ANYFEATURE_VSLAM
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const vec3f &XYZ_, Keyframe pRefKF, shared_ptr<Map> pMap, const FeatureType& featureType):
    mnFirstKFid(pRefKF->keyId), mnFirstFrame(pRefKF->mnFrameId), nObs(0),
    idLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<Pt>(NULL)), minDistance(0), maxDistance(0), mpMap(pMap), featureType(featureType)
{
    keypointType = GetKeypointType(featureType);
    descriptorType = GetDescriptorType(featureType);

    XYZ = XYZ_;
    normalVector = vec3f::Zero();

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    ptId = nNextId++;

    refKeyframe = mpRefKF;
    refIndex = -1;

}

MapPoint::MapPoint(const vec3f &XYZ_, shared_ptr<Map> pMap, Frame* pFrame, const int &idxF, const FeatureType& featureType):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), idLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<Keyframe>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap), featureType(featureType)
{
    keypointType = GetKeypointType(featureType);
    descriptorType = GetDescriptorType(featureType);

    XYZ = XYZ_;
    vec3f twc = pFrame->GetCameraCenter();

    vec3f PC = XYZ - twc;
    const float dist = PC.norm();
    normalVector = PC / dist;

    const float levelScaleFactor =  pFrame->GetKeyPtSize(idxF);

    maxDistance = dist * levelScaleFactor;
    minDistance = maxDistance / pFrame->maxKeyPtSize;

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    ptId = nNextId++;

    refKeyframe = nullptr;
    refIndex = -1;
}

void MapPoint::SetWorldPos(const vec3f &XYZ_)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    XYZ = XYZ_;
}

vec3f MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return XYZ;
}

vec3f MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return normalVector;
}

Keyframe MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::AddObservation(Keyframe projKeyframe,  const KeypointIndex& projIndex)
{
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(observations.count(projKeyframe->keyId))
            return;

        observations[projKeyframe->keyId] = make_shared<Observation>(projKeyframe, projIndex,
                                                                    refKeyframe , refIndex);
        increasePointObservability(projKeyframe,projIndex);
    }
    ComputeDistinctiveDescriptors()->UpdateNormalAndDepth();
}

int MapPoint::GetNumberOfObservations()
{
    return int(observations.size());
}

void MapPoint::increasePointObservability(Keyframe projKeyframe, const KeypointIndex& projIndex){
    if(projKeyframe->mvuRight[projIndex] >= 0)
        nObs += 2;
    else
        nObs++;
}

void MapPoint::decreasePointObservability(Keyframe projKeyframe, const KeypointIndex& projIndex){
    if(projKeyframe->mvuRight[projIndex] >= 0)
        nObs-=2;
    else
        nObs--;
}
Keyframe MapPoint::GetCurrentRefKeyframe(){
    return refKeyframe;
}

void MapPoint::SetRefIndex(const KeypointIndex& refIndex_){
    refIndex = refIndex_;
}

void MapPoint::EraseObservation(Keyframe projKeyframe)
{
    bool removePoint = false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(observations.count(projKeyframe->keyId))
        {
            KeypointIndex projIndex = observations[projKeyframe->keyId]->projIndex;
            decreasePointObservability(projKeyframe,projIndex);

            observations.erase(projKeyframe->keyId);

            if(mpRefKF->keyId == projKeyframe->keyId)
                mpRefKF = observations.begin()->second->projKeyframe;

            // If only 2 observations or less, discard point
            removePoint = (GetNumberOfObservations() <= 2);
        }
    }

    if(removePoint)
        SetBadFlag();
    else
        ComputeDistinctiveDescriptors()->UpdateNormalAndDepth();
}

map<KeyframeId, Obs> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return observations;
}

int MapPoint::NumberOfObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
    map<KeyframeId,Obs> observations_tmp;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad = true;
        observations_tmp = observations;
        observations.clear();
    }
    for(auto& obs: observations_tmp)
    {
        Keyframe keyframe = obs.second->projKeyframe;
        keyframe->EraseMapPointMatch(obs.second->projIndex);
    }

    mpMap->EraseMapPoint(thisPt());
}

Pt MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(Pt pMP)
{
    if(pMP->ptId == this->ptId)
        return;

    int nvisible, nfound;

    map<KeyframeId,Obs> observations_tmp;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        observations_tmp = observations;
        observations.clear();
        mbBad = true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(auto& obs: observations_tmp)
    {
        // Replace measurement in keyframe
        Keyframe keyframe = obs.second->projKeyframe;

        if(!pMP->IsInKeyFrame(keyframe))
        {
            keyframe->ReplaceMapPointMatch(obs.second->projIndex, pMP);
            pMP->AddObservation(keyframe,obs.second->projIndex);
        }
        else
        {
            keyframe->EraseMapPointMatch(obs.second->projIndex);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(thisPt());
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

Pt MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    map<KeyframeId, Obs> observations_tmp;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return thisPt();
    }

    observations_tmp = GetObservations();
    if(observations_tmp.empty())
        return thisPt();

    vector<KeypointIndex> projIndexes{};
    vector<Keyframe> projKeyframes{};
    vector<cv::Mat> descriptors;
    descriptors.reserve(observations_tmp.size());
    for(auto& obs: observations_tmp)
    {
        Keyframe projKeyframe = obs.second->projKeyframe;
        if(!projKeyframe->isBad()){
            descriptors.push_back(projKeyframe->mDescriptors.row(obs.second->projIndex));
            projIndexes.push_back(obs.second->projIndex);
            projKeyframes.push_back(projKeyframe);
        }

    }

    if(descriptors.empty())
        return thisPt();

    // Compute distances between them
    const size_t N = descriptors.size();

    Descriptor_Distance_Type Distances[N][N];
    for(size_t i = 0;i < N; i++)
    {
        Distances[i][i] = Descriptor_Distance_Type(0.0);
        for(size_t j = i + 1;j < N; j++)
        {
            Descriptor_Distance_Type distij = FeatureMatcher::DescriptorDistance(descriptors[i], descriptors[j],descriptorType);
            Distances[i][j] = distij;
            Distances[j][i] = distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    Descriptor_Distance_Type BestMedian = std::numeric_limits<Descriptor_Distance_Type>::max();
    int BestIdx{0};
    for(size_t i = 0;i < N; i++)
    {
        vector<Descriptor_Distance_Type> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        Descriptor_Distance_Type median = vDists[0.5*(N-1)];

        if(median < BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = descriptors[BestIdx].clone();
        refIndex = projIndexes[BestIdx];
        refKeyframe = projKeyframes[BestIdx];
    }
    return thisPt();
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(Keyframe keyframe)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(observations.count(keyframe->keyId))
        return observations[keyframe->keyId]->projIndex;
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(Keyframe keyframe)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (observations.count(keyframe->keyId));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<KeyframeId , Obs> observations_tmp = GetObservations();
    if(observations_tmp.empty())
        return;

    Keyframe refKeyframe_;
    vec3f XYZ_;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;

        refKeyframe_ = mpRefKF;
        XYZ_ = XYZ;
    }

    vec3f normal{vec3f::Zero()};
    int n = 0;
    for(auto& obs: observations_tmp)
    {
        Keyframe projKeyframe = obs.second->projKeyframe;
        vec3f twc_i = projKeyframe->GetCameraCenter();
        vec3f normal_i = XYZ_ - twc_i;
        normal = normal + normal_i / normal_i.norm();
        n++;
    }

    vec3f PC = XYZ_ - refKeyframe_->GetCameraCenter();
    const float dist = PC.norm();
    const float levelScaleFactor =  refKeyframe_->GetKeyPtSize(observations_tmp[refKeyframe_->keyId]->projIndex);

    KeypointIndex keyPtIdx = observations_tmp[refKeyframe_->keyId]->projIndex;
    {
        unique_lock<mutex> lock3(mMutexPos);

        refDistance = dist;
        refSize  = refKeyframe_->GetKeyPtSize(keyPtIdx);;
        refSigma = refKeyframe_->GetKeyPt1DSigma(keyPtIdx);

        maxDistance = dist * levelScaleFactor;
        minDistance = maxDistance / refKeyframe_->maxKeyPtSize ;

        normalVector = normal / n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f * minDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f * maxDistance;
}

float MapPoint::PredictSize(const float &currentDist)
{
    unique_lock<mutex> lock(mMutexPos);
    return refSize * refDistance / currentDist;
}

float MapPoint::PredictSigma(const float &currentDist)
{
    unique_lock<mutex> lock(mMutexPos);
    return refSigma * refDistance / currentDist;
}

} //namespace ORB_SLAM

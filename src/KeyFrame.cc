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

#include "KeyFrame.h"
#include "Converter.h"
#include "FeatureMatcher.h"
#include<mutex>
#include <memory>

namespace ANYFEATURE_VSLAM
{

static bool KeyframeComparison(pair<int , Keyframe> a, pair<int , Keyframe> b){
    return (a.first != b.first) ? (a.first < b.first) : (a.second->keyId < b.second->keyId);
}

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame(Frame &F, shared_ptr<Map> pMap, shared_ptr<KeyFrameDatabase>pKFDB):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec),  sizeTolerance(F.sizeTolerance),
    mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    keyPtsSigma2(F.keyPtsSigma2),keyPtsInf(F.keyPtsInf),keyPtsSize(F.keyPtsSize),
    maxKeyPtSize(F.maxKeyPtSize),maxKeyPtSigma(F.maxKeyPtSigma),
    mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.pts), mpKeyFrameDB(pKFDB),
    vocabulary(F.vocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap)
{
    keyId = nNextId++;

    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.Tcw);
}

void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        vocabulary->transform(mDescriptors,mBowVec,mFeatVec);
    }
}

void KeyFrame::SetPose(const mat4f &Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);

    Tcw = Tcw_;

    mat3f Rcw = Tcw.block<3,3>(0,0);
    vec3f tcw = Tcw.block<3,1>(0,3);
    mat3f Rwc = Rcw.transpose();
    twc = -Rwc * tcw;

    Twc = mat4f::Identity();
    Twc.block<3,3>(0,0) = Rwc;
    Twc.block<3,1>(0,3) = twc;

    vec4f center{mHalfBaseline, 0.0f , 0.0f, 1.0f};
    Cw = Twc * center;
}

mat4f KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw;
}

mat4f KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc;
}

vec3f KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return twc;
}

vec4f KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw;
}


mat3f KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.block<3,3>(0,0);
}

vec3f KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.block<3,1>(0,3);
}

void KeyFrame::AddConnection(Keyframe keyframe, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(!connectedKeyFrameWeights.count(keyframe->keyId)){
            connectedKeyFrameWeights[keyframe->keyId] = weight;
            connectedKeyFrames[keyframe->keyId] = keyframe;
        }
        else if(connectedKeyFrameWeights[keyframe->keyId] != weight)
            connectedKeyFrameWeights[keyframe->keyId] = weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,Keyframe> > vPairs;
    vPairs.reserve(connectedKeyFrameWeights.size());

    for(auto& keyframeWeight: connectedKeyFrameWeights)
        vPairs.push_back(make_pair(keyframeWeight.second,connectedKeyFrames[keyframeWeight.first]));

    sort(vPairs.begin(),vPairs.end(),KeyframeComparison);
    list<Keyframe> keyframes;
    list<int> weights;
    for(size_t i = 0, iend = vPairs.size(); i < iend;i++)
    {
        keyframes.push_front(vPairs[i].second);
        weights.push_front(vPairs[i].first);
    }

    orderedConnectedKeyFrames = vector<Keyframe>(keyframes.begin(),keyframes.end());
    orderedWeights = vector<int>(weights.begin(), weights.end());
}

map<KeyframeId,Keyframe> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    map<KeyframeId,Keyframe> connectedKeyFrames_tmp;
    for(auto connectedKeyFrame: connectedKeyFrames)
        connectedKeyFrames_tmp[connectedKeyFrame.first] = connectedKeyFrame.second;
    return connectedKeyFrames_tmp;
}

vector<Keyframe> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return orderedConnectedKeyFrames;
}

vector<Keyframe> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)orderedConnectedKeyFrames.size()<N)
        return orderedConnectedKeyFrames;
    else
        return vector<Keyframe>(orderedConnectedKeyFrames.begin(),orderedConnectedKeyFrames.begin()+N);

}

vector<Keyframe> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(orderedConnectedKeyFrames.empty())
        return vector<Keyframe>();

    vector<int>::iterator it = upper_bound(orderedWeights.begin(),orderedWeights.end(),w,KeyFrame::weightComp);
    if(it == orderedWeights.end())
        return vector<Keyframe>();
    else
    {
        int n = it - orderedWeights.begin();
        return vector<Keyframe>(orderedConnectedKeyFrames.begin(), orderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(Keyframe keyframe)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(connectedKeyFrameWeights.count(keyframe->keyId))
        return connectedKeyFrameWeights[keyframe->keyId];
    else
        return 0;
}

Pt KeyFrame::CreateMonocularMapPoint(const vec3f& worldPos,
                            const KeypointIndex& refIndex,
                            Keyframe projKeyframe, const KeypointIndex& projIndex,
                            const FeatureType& featureType)
{
    auto pt = make_shared<MapPoint>(worldPos,thisKeyframe(),mpMap, featureType);

    AddMapPoint(pt,refIndex);
    pt->AddObservation(thisKeyframe(), refIndex);

    projKeyframe->AddMapPoint(pt,projIndex);
    pt->AddObservation(projKeyframe, projIndex);

    mpMap->AddMapPoint(pt);
    return pt;
}

Pt KeyFrame::CreateMapPoint(const vec3f& worldPos, const KeypointIndex& refIndex, const FeatureType& featureType){
        auto pt = make_shared<MapPoint>(worldPos,thisKeyframe(),mpMap,featureType);

        AddMapPoint(pt,refIndex);
        pt->AddObservation(thisKeyframe(), refIndex);
        mpMap->AddMapPoint(pt);
        return pt;
}

void KeyFrame::AddMapPoint(Pt pt, const KeypointIndex& index)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[index] = pt;

    if (keyId == pt->GetCurrentRefKeyframe()->keyId)
        pt->SetRefIndex(index);
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<Pt>(NULL);
}

void KeyFrame::EraseMapPointMatch(Pt pMP)
{
    int idx = pMP->GetIndexInKeyFrame(thisKeyframe());
    if(idx>=0)
        mvpMapPoints[idx]=static_cast<Pt>(NULL);
}


void KeyFrame::ReplaceMapPointMatch(const size_t &idx, Pt pMP)
{
    mvpMapPoints[idx]=pMP;
}

set<Pt> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<Pt> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        Pt pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        Pt pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->NumberOfObservations() >= minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<Pt> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

Pt KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections()
{
    map<KeyframeId ,int> KFweightsCounter;
    map<KeyframeId ,Keyframe> KFcounter;

    vector<Pt> pts;
    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        pts = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(auto& pt: pts)
    {
        if(!pt)
            continue;

        if(pt->isBad())
            continue;

        map<KeyframeId , Obs> observations = pt->GetObservations();

        for(auto& obs: observations)
        {
            if(obs.first == keyId)
                continue;
            KFweightsCounter[obs.first]++;
            KFcounter[obs.first] = obs.second->projKeyframe;
        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax = 0;
    Keyframe keyframeMaxObs = nullptr;
    int th = 15;

    vector<pair<int,Keyframe> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(auto& weightCount: KFweightsCounter)
    {
        if(weightCount.second > nmax)
        {
            nmax = weightCount.second;
            keyframeMaxObs = KFcounter[weightCount.first];
        }
        if(weightCount.second >= th)
        {
            vPairs.push_back(make_pair(weightCount.second,KFcounter[weightCount.first]));
            KFcounter[weightCount.first]->AddConnection(thisKeyframe(),weightCount.second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,keyframeMaxObs));
        keyframeMaxObs->AddConnection(thisKeyframe(),nmax);
    }

    sort(vPairs.begin(),vPairs.end(),KeyframeComparison);
    list<Keyframe> keyframes;
    list<int> weights;
    for(size_t i = 0; i < vPairs.size(); i++)
    {
        keyframes.push_front(vPairs[i].second);
        weights.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        connectedKeyFrameWeights = KFweightsCounter;
        connectedKeyFrames = KFcounter;
        orderedConnectedKeyFrames = vector<Keyframe>(keyframes.begin(),keyframes.end());
        orderedWeights = vector<int>(weights.begin(), weights.end());

        if(mbFirstConnection && keyId!=0)
        {
            mpParent = orderedConnectedKeyFrames.front();
            mpParent->AddChild(thisKeyframe());
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(Keyframe pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(Keyframe pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(Keyframe pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(thisKeyframe());
}

set<Keyframe> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

Keyframe KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(Keyframe pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(Keyframe pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<Keyframe> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{   
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(keyId==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(auto& connectedKeyFrame: connectedKeyFrames)
        connectedKeyFrame.second->EraseConnection(thisKeyframe());

    for(size_t i=0; i<mvpMapPoints.size(); i++){
        if(mvpMapPoints[i]){
            mvpMapPoints[i]->EraseObservation(thisKeyframe());
        }
    }

    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        connectedKeyFrameWeights.clear();
        orderedConnectedKeyFrames.clear();
        connectedKeyFrameWeights.clear();
        orderedWeights.clear();

        // Update Spanning Tree
        set<Keyframe> sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            Keyframe pC;
            Keyframe pP;

            for(set<Keyframe>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                Keyframe pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<Keyframe> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<Keyframe>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->keyId == (*spcit)->keyId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
            for(set<Keyframe>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(thisKeyframe());
        Tcp = Tcw * mpParent->GetPoseInverse();
        mbBad = true;
    }

    mpMap->EraseKeyFrame(thisKeyframe());
    mpKeyFrameDB->erase(thisKeyframe());
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(Keyframe keyframe)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(connectedKeyFrameWeights.count(keyframe->keyId))
        {
            connectedKeyFrameWeights.erase(keyframe->keyId);
            connectedKeyFrames.erase(keyframe->keyId);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

vec3f KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        vec3f x3Dc{x, y, z};

        unique_lock<mutex> lock(mMutexPose);
        return Twc.block<3,3>(0,0) * x3Dc + Twc.block<3,1>(0,3);
    }
    else
        return vec3f{0.0f,0.0f,-1.0f};
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<Pt> vpMapPoints;
    mat4f Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw;
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    vec3f Rcw2 = Tcw_.block<1,3>(2,0).transpose();
    float zcw = Tcw_(2,3);
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            Pt pMP = mvpMapPoints[i];
            vec3f x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

    float KeyFrame::GetKeyPtSize(const KeypointIndex &keyPtIdx) const {
        return keyPtsSize[keyPtIdx];
    }

    float KeyFrame::GetKeyPt1DSigma2(const KeypointIndex &keyPtIdx) const
    {
        return 0.5f * (keyPtsSigma2[keyPtIdx](0,0) + keyPtsSigma2[keyPtIdx](1,1));
    }

    mat2f KeyFrame::GetKeyPt2DSigma2(const KeypointIndex &keyPtIdx) const
    {
        return keyPtsSigma2[keyPtIdx];
    }

    mat3f KeyFrame::GetKeyPt3DSigma2(const KeypointIndex &keyPtIdx) const
    {
        mat3f sigma2Matrix{mat3f::Zero()};
        sigma2Matrix.block<2,2>(0,0) = keyPtsSigma2[keyPtIdx];
        sigma2Matrix(2,2) = GetKeyPt1DSigma2(keyPtIdx);
        return sigma2Matrix;
    }

    float KeyFrame::GetKeyPt1DInf(const KeypointIndex &keyPtIdx) const
    {
        return 0.5f * (keyPtsInf[keyPtIdx](0,0) + keyPtsInf[keyPtIdx](1,1));
    }

    mat2f KeyFrame::GetKeyPt2DInf(const KeypointIndex &keyPtIdx) const
    {
        return keyPtsInf[keyPtIdx];
    }

    mat3f KeyFrame::GetKeyPt3DInf(const KeypointIndex &keyPtIdx) const
    {
        mat3f infMatrix{mat3f::Zero()};
        infMatrix.block<2,2>(0,0) = keyPtsInf[keyPtIdx];
        infMatrix(2,2) = GetKeyPt1DInf(keyPtIdx);
        return infMatrix;
    }

    float KeyFrame::GetKeyPt1DSigma(const KeypointIndex &keyPtIdx) const
    {
        return sqrtf(GetKeyPt1DSigma2(keyPtIdx));
    }

} //namespace ORB_SLAM

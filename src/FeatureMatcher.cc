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

#include "FeatureMatcher.h"
#include "Converter.h"
#include "MathFunctions.h"

#include "Feature_orb32.h"
#include "Feature_akaze61.h"
#include "Feature_brisk48.h"
#include "Feature_surf64.h"
#include "Feature_kaze64.h"
#include "Feature_sift128.h"
#include "Feature_r2d2_128.h"
#include "Feature_anyFeatBin.h"
#include "Feature_anyFeatNonBin.h"

#include<limits.h>

#include<opencv2/core/core.hpp>
//#include<opencv2/features2d/features2d.hpp>

#include "DBoW2/FeatureVector.h"

#include<stdint-gcc.h>
#include <memory>

using namespace std;

namespace ANYFEATURE_VSLAM
{

#ifdef VANILLA_ORB_SLAM2
Descriptor_Distance_Type FeatureMatcher::TH_HIGH = Descriptor_Distance_Type(100);
Descriptor_Distance_Type FeatureMatcher::TH_LOW = Descriptor_Distance_Type(50);
Descriptor_Distance_Type FeatureMatcher::descDistTh_high_reloc = Descriptor_Distance_Type(100);
Descriptor_Distance_Type FeatureMatcher::descDistTh_low_reloc = Descriptor_Distance_Type(64);
#else
Descriptor_Distance_Type FeatureMatcher::TH_HIGH = Descriptor_Distance_Type(0.0);
Descriptor_Distance_Type FeatureMatcher::TH_LOW = Descriptor_Distance_Type(0.0);
Descriptor_Distance_Type FeatureMatcher::descDistTh_high_reloc = Descriptor_Distance_Type(0.0);
Descriptor_Distance_Type FeatureMatcher::descDistTh_low_reloc = Descriptor_Distance_Type(0.0);
#endif

VerbosityLevel FeatureMatcher::verbosity{MEDIUM};

const int FeatureMatcher::HISTO_LENGTH = 30;
float FeatureMatcher::radiusScale{1.15f};

FeatureMatcher::FeatureMatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
}

// SearchByProjection 1
// TrackLocalMap
int FeatureMatcher::SearchByProjection(Frame &F, const vector<Pt> &vpMapPoints, const float& radiusTh)
{
    int nmatches=0;

    for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++)
    {
        Pt pMP = vpMapPoints[iMP];
        if(!pMP)
            continue;

        if(!pMP->mbTrackInView)
            continue;

        if(pMP->isBad())
            continue;

        // The size of the window will depend on the viewing direction
        const float predictedSize = pMP->trackSize;
        float r = radiusScale * radiusTh *  RadiusByViewingCos(pMP->trackViewCos) * predictedSize;

        const vector<size_t> vIndices =
                F.GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY, r,
                                    (pMP->trackSize / F.sizeTolerance),(pMP->trackSize * F.sizeTolerance));

        if(vIndices.empty())
            continue;

        const cv::Mat refDescriptor = pMP->GetDescriptor();
        Descriptor_Distance_Type bestDist{highestPossibleDistance},bestDist2{highestPossibleDistance};
        float bestSize{-1.0f},bestSize2{-1.0f};
        int bestIdx{-1};

        // Get best and second matches with near keypoints
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            if(F.pts[idx])
                if(F.pts[idx]->NumberOfObservations() > 0)
                    continue;

            if(F.mvuRight[idx]>0)
            {
                const float er = fabs(pMP->mTrackProjXR-F.mvuRight[idx]);
                if(er > r * pMP->trackSigma)
                    continue;
            }

            const cv::Mat &descriptor = F.mDescriptors.row(idx);
            const Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,pMP->descriptorType);

            if(descDist < bestDist)
            {
                bestDist2 = bestDist;
                bestDist = descDist;
                bestIdx = idx;
                bestSize2 = bestSize;
                bestSize = F.GetKeyPtSize(KeypointIndex(idx));
            }
            else if(descDist < bestDist2)
            {
                bestDist2 = descDist;
                bestSize2 = F.GetKeyPtSize(KeypointIndex(idx));
            }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if(bestDist <= TH_HIGH)
        {
            if((bestSize / bestSize2 < F.sizeTolerance) && (bestSize / bestSize2 > F.invSizeTolerance) && (bestSize2 > 0.0f)){
                if(bestDist > mfNNratio * bestDist2){
                    continue;
                }
            }

            F.pts[bestIdx]=pMP;
            nmatches++;
        }
    }

    return nmatches;
}

float FeatureMatcher::RadiusByViewingCos(const float &viewCos)
{
    if(viewCos>0.998)
        return 2.5;
    else
        return 4.0;
}


bool FeatureMatcher::CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const mat3f& F12 , const Keyframe pKF2, const float& sigma2_kp2)
{
    // Epipolar line in second image l = x1'F12 = [a b c]
    const float a = kp1.pt.x * F12(0,0) + kp1.pt.y * F12(1,0) + F12(2,0);
    const float b = kp1.pt.x * F12(0,1) + kp1.pt.y * F12(1,1) + F12(2,1);
    const float c = kp1.pt.x * F12(0,2) + kp1.pt.y * F12(1,2) + F12(2,2);

    const float num = a*kp2.pt.x+b*kp2.pt.y+c;

    const float den = a*a+b*b;

    if(den==0)
        return false;

    const float dsqr = num*num/den;

    return dsqr < 3.84f * sigma2_kp2;
}

// SearchByBoW 1
// TrackReferenceKeyframe & Relocalization
int FeatureMatcher::SearchByBoW(Keyframe pKF, Frame &F, vector<Pt> &vpMapPointMatches)
{
    const vector<Pt> vpMapPointsKF = pKF->GetMapPointMatches();

    vpMapPointMatches = vector<Pt>(F.N,static_cast<Pt>(NULL));

    const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;

    // Rotation Histogram (to check rotation consistency)
    int nMatches{0};
    float rotFactor{};
    vector<vector<int>> rotHist = initRotationHistogram(rotFactor,HISTO_LENGTH);

    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
    DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
    DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
    DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

    while(KFit != KFend && Fit != Fend)
    {
        if(KFit->first == Fit->first)
        {
            const vector<unsigned int> vIndicesKF = KFit->second;
            const vector<unsigned int> vIndicesF = Fit->second;

            for(size_t iKF=0; iKF<vIndicesKF.size(); iKF++)
            {
                const unsigned int realIdxKF = vIndicesKF[iKF];

                Pt pMP = vpMapPointsKF[realIdxKF];

                if(!pMP)
                    continue;

                if(pMP->isBad())
                    continue;                

                const cv::Mat &refDescriptor= pKF->mDescriptors.row(realIdxKF);
                Descriptor_Distance_Type bestDist1{highestPossibleDistance},bestDist2{highestPossibleDistance};
                int bestIdxF{-1} ;

                for(size_t iF=0; iF<vIndicesF.size(); iF++)
                {
                    const unsigned int realIdxF = vIndicesF[iF];

                    if(vpMapPointMatches[realIdxF])
                        continue;

                    const cv::Mat &descriptor = F.mDescriptors.row(realIdxF);
                    const Descriptor_Distance_Type descDist =  DescriptorDistance(refDescriptor,descriptor,pMP->descriptorType);

                    if(descDist < bestDist1)
                    {
                        bestDist2 = bestDist1;
                        bestDist1 = descDist;
                        bestIdxF = realIdxF;
                    }
                    else if(descDist < bestDist2)
                    {
                        bestDist2 = descDist;
                    }
                }

                if(bestDist1 <= TH_LOW)
                {
                    if(static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2))
                    {
                        vpMapPointMatches[bestIdxF]=pMP;

                        const cv::KeyPoint &kp = pKF->mvKeysUn[realIdxKF];
                        nMatches++;
                        if(mbCheckOrientation)
                            updateRotationHistogram(rotHist,bestIdxF,kp,F.mvKeys[bestIdxF],rotFactor,HISTO_LENGTH);
                    }
                }

            }

            KFit++;
            Fit++;
        }
        else if(KFit->first < Fit->first)
        {
            KFit = vFeatVecKF.lower_bound(Fit->first);
        }
        else
        {
            Fit = F.mFeatVec.lower_bound(KFit->first);
        }
    }


    if(mbCheckOrientation)
        filterMatchesWithOrientation(rotHist,vpMapPointMatches,nMatches);

    return nMatches;
}

// SearchByProjection 2
// Compute Sim3
int FeatureMatcher::SearchByProjection(Keyframe pKF, const mat4f& Scw, const vector<Pt> &vpPoints, vector<Pt> &vpMatched, const float& radiusTh)
{
    // Get Calibration Parameters for later projection
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;

    // Decompose Scw
    mat3f sRcw = Scw.block<3,3>(0,0);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
    mat3f Rcw = sRcw / scw;
    vec3f tcw = Scw.block<3,1>(0,3);
    vec3f Ow = -Rcw.transpose() * tcw;

    // Set of MapPoints already found in the KeyFrame
    set<Pt> spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(static_cast<Pt>(NULL));

    int nmatches=0;

    // For each Candidate MapPoint Project and Match
    for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
    {
        Pt pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        vec3f p3Dw = pMP->GetWorldPos();

        // Transform into Camera Coords.
        vec3f p3Dc = Rcw * p3Dw + tcw;

        // Depth must be positive
        if(p3Dc(2) < 0.0f)
            continue;

        // Project into Image
        const float invz = 1.0f / p3Dc(2);
        const float x = p3Dc(0) * invz;
        const float y = p3Dc(1) * invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        // Depth must be inside the scale invariance region of the point
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        vec3f PO = p3Dw - Ow;
        const float dist3D = PO.norm();

        if(dist3D < minDistance || dist3D > maxDistance)
            continue;

        // Viewing angle must be less than 60 deg
        vec3f Pn = pMP->GetNormal();

        if(PO.dot(Pn) < 0.5f * dist3D)
            continue;


        // Search in a radius
        float predictedSize = pMP->PredictSize(dist3D);
        const float radius = radiusScale * radiusTh * predictedSize;
        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat refDescriptor = pMP->GetDescriptor();
        Descriptor_Distance_Type bestDist{highestPossibleDistance};
        int bestIdx{-1};

        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;
            if(vpMatched[idx])
                continue;

            const float keyPtSize = pKF->GetKeyPtSize(KeypointIndex (idx));
            if((keyPtSize < predictedSize / pKF->sizeTolerance) || (keyPtSize > predictedSize * pKF->sizeTolerance))
                continue;

            const cv::Mat &descriptor = pKF->mDescriptors.row(idx);
            const Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,pMP->descriptorType);

            if(descDist < bestDist)
            {
                bestDist = descDist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_LOW)
        {
            vpMatched[bestIdx]=pMP;
            nmatches++;
        }

    }

    return nmatches;
}

int FeatureMatcher::SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, const int& windowSize,
                                            const DescriptorType& descriptorType)
{

#ifndef VANILLA_ORB_SLAM2
    {

        //static size_t frame_id_0 = 0;
        static vector<Descriptor_Distance_Type> matchDistances;
        static vector<int> numCandidates;
/*
        //if(F1.mnId != frame_id_0){
            frame_id_0 = F1.mnId;
            matchDistances.clear();
            numCandidates.clear();
        //}

        unordered_map<KeypointIndex, Descriptor_Distance_Type> bestMatchDistances{};
        unordered_map<KeypointIndex, int> bestNumCandidates{};

        for(KeypointIndex i1{0}; i1 < F1.mvKeysUn.size(); i1++)
        {

            if(F1.mvKeysUn[i1].octave > 0)
                continue;

            vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,0.0f,F1.maxKeyPtSize);
            if(vIndices2.empty())
                continue;

            cv::Mat refDescriptor = F1.mDescriptors.row(i1);
            Descriptor_Distance_Type bestDist{highestPossibleDistance}, bestDist2{highestPossibleDistance};
            KeypointIndex bestIdx2{-1};

            int numCandidates_i{0};
            for(auto& i2: vIndices2)
            {
                if(F2.mvKeysUn[i2].octave > 0)
                    continue;

                cv::Mat descriptor = F2.mDescriptors.row(i2);
                Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,descriptorType);

                if(descDist < bestDist)
                {
                    bestDist2 = bestDist;
                    bestDist = descDist;
                    bestIdx2 = KeypointIndex(i2);
                }
                else if(descDist < bestDist2)
                {
                    bestDist2 = descDist;
                }
                numCandidates_i++;
            }

            if(float(bestDist) > (float) bestDist2 * mfNNratio)
                continue;

            auto bestMatchDistance = bestMatchDistances.find(bestIdx2);
            if(bestMatchDistance != bestMatchDistances.end()){
                if(bestMatchDistance->second <= bestDist)
                    continue;
            }
            bestMatchDistances[bestIdx2] = bestDist;
            bestNumCandidates[bestIdx2] = numCandidates_i;
        }

        for(int i{0}; i < bestMatchDistances.size(); i++){
            if(bestMatchDistances[i] > 0){
                matchDistances.emplace_back(bestMatchDistances[i]);
                numCandidates.emplace_back(bestNumCandidates[i]);
            }
        }
*/
        //setDescriptorDistanceThresholds(matchDistances,numCandidates,descriptorType);
        //terminate();
    }
#endif

    vnMatches12 = vector<int>(F1.mvKeysUn.size(),-1);

    int nMatches{0};
    float rotFactor{};
    vector<vector<int>> rotHist = initRotationHistogram(rotFactor,HISTO_LENGTH);

    vector<Descriptor_Distance_Type> vMatchedDistance(F2.mvKeysUn.size(),highestPossibleDistance);
    vector<int> vnMatches21(F2.mvKeysUn.size(),-1);

    for(size_t i1=0, iend1=F1.mvKeysUn.size(); i1<iend1; i1++)
    {
        cv::KeyPoint kp1 = F1.mvKeysUn[i1];
        int level1 = kp1.octave;
        if(level1 > 0)
            continue;

        vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,
                                                        0.0f,F1.maxKeyPtSize);
        if(vIndices2.empty())
            continue;

        cv::Mat refDescriptor = F1.mDescriptors.row(i1);
        Descriptor_Distance_Type bestDist{highestPossibleDistance}, bestDist2{highestPossibleDistance};
        int bestIdx2{-1};

        for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
        {
            size_t i2 = *vit;

            cv::Mat descriptor = F2.mDescriptors.row(i2);
            Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,descriptorType);

            if(vMatchedDistance[i2] <= descDist)
                continue;

            if(descDist < bestDist)
            {
                bestDist2 = bestDist;
                bestDist = descDist;
                bestIdx2 = i2;
            }
            else if(descDist < bestDist2)
            {
                bestDist2 = descDist;
            }
        }

        if(bestDist <= TH_LOW)
        {
            if(float(bestDist) < (float) bestDist2 * mfNNratio)
            {
                if(vnMatches21[bestIdx2] >= 0)
                {
                    vnMatches12[vnMatches21[bestIdx2]]= -1;
                    nMatches--;
                }
                vnMatches12[i1] = bestIdx2;
                vnMatches21[bestIdx2] = i1;
                vMatchedDistance[bestIdx2] = bestDist;
                nMatches++;

                if(mbCheckOrientation)
                    updateRotationHistogram(rotHist,i1,F1.mvKeysUn[i1],F2.mvKeysUn[bestIdx2],rotFactor,HISTO_LENGTH);

            }
        }

    }

    if(mbCheckOrientation)
        filterMatchesWithOrientation(rotHist,vnMatches12,nMatches);

    //Update prev matched
    for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
        if(vnMatches12[i1]>=0)
            vbPrevMatched[i1]=F2.mvKeysUn[vnMatches12[i1]].pt;

    return nMatches;
}

// SearchByBoW 2
// ComputeSim3
int FeatureMatcher::SearchByBoW(Keyframe pKF1, Keyframe pKF2, vector<Pt > &vpMatches12)
{
    const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeysUn;
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
    const vector<Pt> vpMapPoints1 = pKF1->GetMapPointMatches();
    const cv::Mat &Descriptors1 = pKF1->mDescriptors;

    const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn;
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
    const vector<Pt> vpMapPoints2 = pKF2->GetMapPointMatches();
    const cv::Mat &Descriptors2 = pKF2->mDescriptors;

    vpMatches12 = vector<Pt>(vpMapPoints1.size(),static_cast<Pt>(NULL));
    vector<bool> vbMatched2(vpMapPoints2.size(),false);

    int nMatches{0};
    float rotFactor{};
    vector<vector<int>> rotHist = initRotationHistogram(rotFactor,HISTO_LENGTH);

    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

    while(f1it != f1end && f2it != f2end)
    {
        if(f1it->first == f2it->first)
        {
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                const size_t idx1 = f1it->second[i1];

                Pt pMP1 = vpMapPoints1[idx1];
                if(!pMP1)
                    continue;
                if(pMP1->isBad())
                    continue;

                const cv::Mat &refDescriptor = Descriptors1.row(idx1);
                Descriptor_Distance_Type bestDist1{highestPossibleDistance},bestDist2{highestPossibleDistance};
                int bestIdx2{-1};

                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    const size_t idx2 = f2it->second[i2];

                    Pt pMP2 = vpMapPoints2[idx2];

                    if(vbMatched2[idx2] || !pMP2)
                        continue;

                    if(pMP2->isBad())
                        continue;

                    const cv::Mat &descriptor = Descriptors2.row(idx2);
                    Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,pMP2->descriptorType);

                    if(descDist < bestDist1)
                    {
                        bestDist2 = bestDist1;
                        bestDist1 = descDist;
                        bestIdx2 = idx2;
                    }
                    else if(descDist < bestDist2)
                    {
                        bestDist2 = descDist;
                    }
                }

                if(bestDist1 < TH_LOW)
                {
                    if(static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2))
                    {
                        vpMatches12[idx1] = vpMapPoints2[bestIdx2];
                        vbMatched2[bestIdx2] = true;
                        nMatches++;
                        if(mbCheckOrientation)
                            updateRotationHistogram(rotHist,idx1,vKeysUn1[idx1],vKeysUn2[bestIdx2],rotFactor,HISTO_LENGTH);
                    }
                }
            }

            f1it++;
            f2it++;
        }
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    if(mbCheckOrientation)
        filterMatchesWithOrientation(rotHist,vpMatches12,nMatches);

    return nMatches;
}

int FeatureMatcher::SearchForTriangulation(Keyframe pKF1, Keyframe pKF2, const mat3f& F12,
                                           vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo, const DescriptorType& descriptorType)
{    
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;

    //Compute epipole in second image
    vec3f Cw = pKF1->GetCameraCenter();
    mat3f R2w = pKF2->GetRotation();
    vec3f t2w = pKF2->GetTranslation();
    vec3f C2 = R2w * Cw + t2w;
    const float invz = 1.0f / C2(2);
    const float ex = pKF2->fx * C2(0) * invz + pKF2->cx;
    const float ey = pKF2->fy * C2(1) * invz + pKF2->cy;

    // Find matches between not tracked keypoints
    // Matching speed-up by ORB Vocabulary
    // Compare only ORB that share the same node

    vector<bool> vbMatched2(pKF2->N,false);
    vector<int> vMatches12(pKF1->N,-1);

    int nMatches{0};

    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

    while(f1it!=f1end && f2it!=f2end)
    {
        if(f1it->first == f2it->first)
        {
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                const size_t idx1 = f1it->second[i1];
                
                Pt pMP1 = pKF1->GetMapPoint(idx1);
                
                // If there is already a MapPoint skip
                if(pMP1)
                    continue;

                const bool bStereo1 = pKF1->mvuRight[idx1]>=0;

                if(bOnlyStereo)
                    if(!bStereo1)
                        continue;
                
                const cv::KeyPoint &kp1 = pKF1->mvKeysUn[idx1];
                
                const cv::Mat &refDescriptor = pKF1->mDescriptors.row(idx1);
                Descriptor_Distance_Type bestDist{TH_LOW};
                int bestIdx2{-1};
                
                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    size_t idx2 = f2it->second[i2];
                    
                    Pt pMP2 = pKF2->GetMapPoint(idx2);
                    
                    // If we have already matched or there is a MapPoint skip
                    if(vbMatched2[idx2] || pMP2)
                        continue;

                    const bool bStereo2 = pKF2->mvuRight[idx2]>=0;

                    if(bOnlyStereo)
                        if(!bStereo2)
                            continue;
                    
                    const cv::Mat &descriptor = pKF2->mDescriptors.row(idx2);
                    const Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,descriptorType);

                    if(descDist > TH_LOW || descDist > bestDist)
                        continue;

                    const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];

                    if(!bStereo1 && !bStereo2)
                    {
                        const float distex = ex-kp2.pt.x;
                        const float distey = ey-kp2.pt.y;

                        if(distex*distex+distey*distey < 100.0f * sqrtf(pKF2->GetKeyPt1DSigma2(KeypointIndex(idx2))))
                            continue;
                    }

                    float sigma2_kp2 = pKF2->GetKeyPt1DSigma2(KeypointIndex(idx2));
                    if(CheckDistEpipolarLine(kp1,kp2,F12,pKF2,sigma2_kp2))
                    {
                        bestIdx2 = idx2;
                        bestDist = descDist;
                    }
                }
                
                if(bestIdx2>=0)
                {
                    const cv::KeyPoint &kp2 = pKF2->mvKeysUn[bestIdx2];
                    vMatches12[idx1] = bestIdx2;
                    nMatches++;
                }
            }

            f1it++;
            f2it++;
        }
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    vMatchedPairs.clear();
    vMatchedPairs.reserve(nMatches);

    for(size_t i=0, iend=vMatches12.size(); i<iend; i++)
    {
        if(vMatches12[i]<0)
            continue;
        vMatchedPairs.push_back(make_pair(i,vMatches12[i]));
    }

    return nMatches;
}

// Fuse 1
// Local Mapping
int FeatureMatcher::Fuse(Keyframe pKF, const vector<Pt> &vpMapPoints, const float& radiusTh)
{
    mat3f Rcw = pKF->GetRotation();
    vec3f tcw = pKF->GetTranslation();

    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;
    const float &bf = pKF->mbf;

    vec3f Ow = pKF->GetCameraCenter();

    int nFused=0;

    const int nMPs = vpMapPoints.size();

    for(int i=0; i<nMPs; i++)
    {
        Pt pMP = vpMapPoints[i];

        if(!pMP)
            continue;

        if(pMP->isBad() || pMP->IsInKeyFrame(pKF))
            continue;

        vec3f p3Dw = pMP->GetWorldPos();
        vec3f p3Dc = Rcw * p3Dw + tcw;

        // Depth must be positive
        if(p3Dc(2) < 0.0f)
            continue;

        const float invz = 1.0f / p3Dc(2);
        const float x = p3Dc(0) * invz;
        const float y = p3Dc(1) * invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        const float ur = u-bf*invz;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        vec3f PO = p3Dw - Ow;
        const float dist3D = PO.norm();

        // Depth must be inside the scale pyramid of the image
        if(dist3D < minDistance || dist3D > maxDistance )
            continue;

        // Viewing angle must be less than 60 deg
        vec3f Pn = pMP->GetNormal();

        if(PO.dot(Pn) < 0.5 * dist3D)
            continue;

        // Search in a radius
        float predictedSize = pMP->PredictSize(dist3D);
        const float radius = radiusScale * radiusTh * predictedSize;

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat refDescriptor = pMP->GetDescriptor();
        Descriptor_Distance_Type bestDist{highestPossibleDistance};
        int bestIdx{-1};

        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF->mvKeysUn[idx];

            const float keyPtSize = pKF->GetKeyPtSize(KeypointIndex (idx));
            if((keyPtSize < predictedSize / pKF->sizeTolerance) || (keyPtSize > predictedSize * pKF->sizeTolerance))
                continue;

            if(pKF->mvuRight[idx]>=0)
            {
                // Check reprojection error in stereo
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float &kpr = pKF->mvuRight[idx];
                const float ex = u-kpx;
                const float ey = v-kpy;
                const float er = ur-kpr;
                const float e2 = ex*ex+ey*ey+er*er;

                if(e2 * pKF->GetKeyPt1DInf(KeypointIndex (idx)) > 7.8)
                    continue;
            }
            else
            {
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float ex = u-kpx;
                const float ey = v-kpy;
                const float e2 = ex*ex+ey*ey;

                if(e2 * pKF->GetKeyPt1DInf(KeypointIndex (idx)) > 5.99)
                    continue;
            }

            const cv::Mat &descriptor = pKF->mDescriptors.row(idx);
            const Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,pMP->descriptorType);

            if(descDist < bestDist)
            {
                bestDist = descDist;
                bestIdx = idx;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if(bestDist <= TH_LOW)
        {
            Pt pMPinKF = pKF->GetMapPoint(bestIdx);
            if(pMPinKF)
            {
                if(!pMPinKF->isBad())
                {
                    if(pMPinKF->NumberOfObservations() > pMP->NumberOfObservations())
                        pMP->Replace(pMPinKF);
                    else
                        pMPinKF->Replace(pMP);
                }
            }
            else
            {
                pMP->AddObservation(pKF,bestIdx);
                pKF->AddMapPoint(pMP,bestIdx);
            }
            nFused++;
        }
    }

    return nFused;
}

// Fuse 2
// Loop Closing
int FeatureMatcher::Fuse(Keyframe pKF, const mat4f& Scw, const vector<Pt> &vpPoints, const float& radiusTh, vector<Pt> &vpReplacePoint)
{

    // Get Calibration Parameters for later projection
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;

    // Decompose Scw
    mat3f sRcw = Scw.block<3,3>(0,0);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
    mat3f Rcw = sRcw / scw;
    vec3f tcw = Scw.block<3,1>(0,3);
    vec3f Ow = -Rcw.transpose() * tcw;

    // Set of MapPoints already found in the KeyFrame
    const set<Pt> spAlreadyFound = pKF->GetMapPoints();

    int nFused=0;

    const int nPoints = vpPoints.size();

    // For each candidate MapPoint project and match
    for(int iMP=0; iMP<nPoints; iMP++)
    {
        Pt pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        vec3f p3Dw = pMP->GetWorldPos();

        // Transform into Camera Coords.
        vec3f p3Dc = Rcw * p3Dw + tcw;

        // Depth must be positive
        if(p3Dc(2) < 0.0f)
            continue;

        // Project into Image
        const float invz = 1.0f / p3Dc(2);
        const float x = p3Dc(0) * invz;
        const float y = p3Dc(1) * invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        // Depth must be inside the scale pyramid of the image
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        vec3f PO = p3Dw-Ow;
        const float dist3D = PO.norm();

        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        // Viewing angle must be less than 60 deg
        vec3f Pn = pMP->GetNormal();

        if(PO.dot(Pn) < 0.5f * dist3D)
            continue;

        // Search in a radius
        const float predictedSize = pMP->PredictSize(dist3D);
        const float radius = radiusScale * radiusTh * predictedSize;

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat refDescriptor = pMP->GetDescriptor();
        Descriptor_Distance_Type bestDist{highestPossibleDistance};
        int bestIdx{-1};

        for(vector<size_t>::const_iterator vit=vIndices.begin(); vit!=vIndices.end(); vit++)
        {
            const size_t idx = *vit;

            const float keyPtSize = pKF->GetKeyPtSize(KeypointIndex (idx));
            if((keyPtSize < predictedSize / pKF->sizeTolerance) || (keyPtSize > predictedSize * pKF->sizeTolerance))
                continue;

            const cv::Mat &descriptor = pKF->mDescriptors.row(idx);
            Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,pMP->descriptorType);

            if(descDist < bestDist)
            {
                bestDist = descDist;
                bestIdx = idx;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if(bestDist <= TH_LOW)
        {
            Pt pMPinKF = pKF->GetMapPoint(bestIdx);
            if(pMPinKF)
            {
                if(!pMPinKF->isBad())
                    vpReplacePoint[iMP] = pMPinKF;
            }
            else
            {
                pMP->AddObservation(pKF,bestIdx);
                pKF->AddMapPoint(pMP,bestIdx);
            }
            nFused++;
        }
    }

    return nFused;
}

int FeatureMatcher::SearchBySim3(Keyframe pKF1, Keyframe pKF2, vector<Pt> &vpMatches12,
                                 const float &s12, const mat3f  &R12, const vec3f &t12, const float& radiusTh)
{

    const float &fx = pKF1->fx;
    const float &fy = pKF1->fy;
    const float &cx = pKF1->cx;
    const float &cy = pKF1->cy;

    // Camera 1 from world
    mat3f R1w = pKF1->GetRotation();
    vec3f t1w = pKF1->GetTranslation();

    //Camera 2 from world
    mat3f R2w = pKF2->GetRotation();
    vec3f t2w = pKF2->GetTranslation();

    //Transformation between cameras
    mat3f sR12 = s12 * R12;
    mat3f sR21 = (1.0/s12) * R12.transpose();
    vec3f t21 = -sR21 * t12;

    const vector<Pt> vpMapPoints1 = pKF1->GetMapPointMatches();
    const int N1 = vpMapPoints1.size();

    const vector<Pt> vpMapPoints2 = pKF2->GetMapPointMatches();
    const int N2 = vpMapPoints2.size();

    vector<bool> vbAlreadyMatched1(N1,false);
    vector<bool> vbAlreadyMatched2(N2,false);

    for(int i=0; i<N1; i++)
    {
        Pt pMP = vpMatches12[i];
        if(pMP)
        {
            vbAlreadyMatched1[i]=true;
            int idx2 = pMP->GetIndexInKeyFrame(pKF2);
            if(idx2>=0 && idx2<N2)
                vbAlreadyMatched2[idx2]=true;
        }
    }

    vector<int> vnMatch1(N1,-1);
    vector<int> vnMatch2(N2,-1);

    // Transform from KF1 to KF2 and search
    for(int i1=0; i1<N1; i1++)
    {
        Pt pMP = vpMapPoints1[i1];

        if(!pMP || vbAlreadyMatched1[i1])
            continue;

        if(pMP->isBad())
            continue;

        vec3f p3Dw = pMP->GetWorldPos();
        vec3f p3Dc1 = R1w * p3Dw + t1w;
        vec3f p3Dc2 = sR21 * p3Dc1 + t21;

        // Depth must be positive
        if(p3Dc2(2) < 0.0f)
            continue;

        const float invz = 1.0f / p3Dc2(2);
        const float x = p3Dc2(0) * invz;
        const float y = p3Dc2(1) * invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF2->IsInImage(u,v))
            continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = p3Dc2.norm();

        // Depth must be inside the scale invariance region
        if(dist3D < minDistance || dist3D > maxDistance )
            continue;

        // Search in a radius
        const float predictedSize = pMP->PredictSize(dist3D);
        const float radius = radiusScale * radiusTh * predictedSize;

        const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat refDescriptor = pMP->GetDescriptor();
        Descriptor_Distance_Type bestDist{highestPossibleDistance};
        int bestIdx{-1};

        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF2->mvKeysUn[idx];

            const float keyPtSize = pKF2->GetKeyPtSize(KeypointIndex (idx));
            if((keyPtSize < predictedSize / pKF2->sizeTolerance) || (keyPtSize > predictedSize * pKF2->sizeTolerance))
                continue;

            const cv::Mat &descriptor = pKF2->mDescriptors.row(idx);
            const Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,pMP->descriptorType);

            if(descDist < bestDist)
            {
                bestDist = descDist;
                bestIdx = idx;
            }
        }

        if(bestDist <= TH_HIGH)
        {
            vnMatch1[i1] = bestIdx;
        }
    }

    // Transform from KF2 to KF2 and search
    for(int i2=0; i2<N2; i2++)
    {
        Pt pMP = vpMapPoints2[i2];

        if(!pMP || vbAlreadyMatched2[i2])
            continue;

        if(pMP->isBad())
            continue;

        vec3f p3Dw = pMP->GetWorldPos();
        vec3f p3Dc2 = R2w * p3Dw + t2w;
        vec3f p3Dc1 = sR12 * p3Dc2 + t12;

        // Depth must be positive
        if(p3Dc1(2) < 0.0f)
            continue;

        const float invz = 1.0f / p3Dc1(2);
        const float x = p3Dc1(0) * invz;
        const float y = p3Dc1(1) * invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF1->IsInImage(u,v))
            continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = p3Dc1.norm();

        // Depth must be inside the scale pyramid of the image
        if(dist3D < minDistance || dist3D > maxDistance)
            continue;

        // Search in a radius of 2.5*sigma(ScaleLevel)
        const float predictedSize = pMP->PredictSize(dist3D);
        const float radius = radiusScale * radiusTh * predictedSize;

        const vector<size_t> vIndices = pKF1->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat refDescriptor = pMP->GetDescriptor();
        Descriptor_Distance_Type bestDist{highestPossibleDistance};
        int bestIdx{-1};

        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF1->mvKeysUn[idx];

            const float keyPtSize = pKF1->GetKeyPtSize(KeypointIndex (idx));
            if((keyPtSize < predictedSize / pKF1->sizeTolerance) || (keyPtSize > predictedSize * pKF1->sizeTolerance))
                continue;

            const cv::Mat &descriptor = pKF1->mDescriptors.row(idx);
            const Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,pMP->descriptorType);

            if(descDist < bestDist)
            {
                bestDist = descDist;
                bestIdx = idx;
            }
        }

        if(bestDist <= TH_HIGH)
        {
            vnMatch2[i2]=bestIdx;
        }
    }

    // Check agreement
    int nFound = 0;

    for(int i1=0; i1<N1; i1++)
    {
        int idx2 = vnMatch1[i1];

        if(idx2>=0)
        {
            int idx1 = vnMatch2[idx2];
            if(idx1==i1)
            {
                vpMatches12[i1] = vpMapPoints2[idx2];
                nFound++;
            }
        }
    }

    return nFound;
}

// SearchByProjection 3
// TrackWithMotionModel
int FeatureMatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float& radiusTh, const bool bMono)
{

    // Rotation Histogram (to check rotation consistency)
    int nMatches{0};
    float rotFactor{};
    vector<vector<int>> rotHist = initRotationHistogram(rotFactor,HISTO_LENGTH);

    const mat3f Rcw = CurrentFrame.Tcw.block<3,3>(0,0);
    const vec3f tcw = CurrentFrame.Tcw.block<3,1>(0,3);

    const vec3f twc = -Rcw.transpose() * tcw;

    const mat3f Rlw = LastFrame.Tcw.block<3,3>(0,0);
    const vec3f tlw = LastFrame.Tcw.block<3,1>(0,3);

    const vec3f tlc = Rlw * twc + tlw;

    const bool bForward = false;//tlc(2) > CurrentFrame.mb && !bMono;
    const bool bBackward = false;//-tlc(2) > CurrentFrame.mb && !bMono;
    int numPmP{0};
    for(int i=0; i<LastFrame.N; i++)
    {
        Pt pMP = LastFrame.pts[i];

        if((pMP) && (!pMP->isBad()))
        {
            ++numPmP;
            if(!LastFrame.mvbOutlier[i])
            {
                // Project
                vec3f x3Dw = pMP->GetWorldPos();
                vec3f x3Dc = Rcw*x3Dw+tcw;

                const float xc = x3Dc(0);
                const float yc = x3Dc(1);
                const float invzc = 1.0f / x3Dc(2);

                if(invzc<0)
                    continue;

                float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
                float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;

                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;

                // Search in a window. Size depends on scale
                float keyPtSize = LastFrame.GetKeyPtSize(i);
                float radius = radiusScale * radiusTh * keyPtSize;

                vector<size_t> vIndices2;

                if(bForward)
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, (keyPtSize / CurrentFrame.sizeTolerance),CurrentFrame.maxKeyPtSize);
                else if(bBackward)
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, 0.0, (keyPtSize * CurrentFrame.sizeTolerance));
                else
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, (keyPtSize / CurrentFrame.sizeTolerance),(keyPtSize * CurrentFrame.sizeTolerance));

                if(vIndices2.empty())
                    continue;

                const cv::Mat refDescriptor = pMP->GetDescriptor();
                Descriptor_Distance_Type bestDist{highestPossibleDistance};
                int bestIdx2{-1};

                for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.pts[i2])
                        if(CurrentFrame.pts[i2]->NumberOfObservations() > 0)
                            continue;

                    if(CurrentFrame.mvuRight[i2]>0)
                    {
                        const float ur = u - CurrentFrame.mbf*invzc;
                        const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
                        if(er>radius)
                            continue;
                    }

                    const cv::Mat &descriptor = CurrentFrame.mDescriptors.row(i2);
                    const Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,pMP->descriptorType);

                    if(descDist < bestDist)
                    {
                        bestDist = descDist;
                        bestIdx2 = i2;
                    }
                }

                if(bestDist <= TH_HIGH)
                {
                    CurrentFrame.pts[bestIdx2] = pMP;
                    nMatches++;

                    if(mbCheckOrientation)
                        updateRotationHistogram(rotHist,bestIdx2,LastFrame.mvKeysUn[i],CurrentFrame.mvKeysUn[bestIdx2],rotFactor,HISTO_LENGTH);
                }
            }
        }
    }

    //Apply rotation consistency
    if(mbCheckOrientation)
        filterMatchesWithOrientation(rotHist,CurrentFrame.pts,nMatches);

    return nMatches;
}

// SearchByProjection 4
// Relocalization
int FeatureMatcher::SearchByProjection(Frame &CurrentFrame, Keyframe pKF, const set<Pt> &sAlreadyFound, const float& radiusTh, const bool& useHighMatchingThreshold)
{


    Descriptor_Distance_Type descDistanceTh = descDistTh_low_reloc;
    if(useHighMatchingThreshold)
        descDistanceTh = descDistTh_high_reloc;

    const mat3f Rcw = CurrentFrame.Tcw.block<3,3>(0,0);
    const vec3f tcw = CurrentFrame.Tcw.block<3,1>(0,3);
    const vec3f Ow = -Rcw.transpose() * tcw;

    // Rotation Histogram (to check rotation consistency)
    int nMatches{0};
    float rotFactor{};
    vector<vector<int>> rotHist = initRotationHistogram(rotFactor,HISTO_LENGTH);

    const vector<Pt> vpMPs = pKF->GetMapPointMatches();

    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        Pt pMP = vpMPs[i];

        if(pMP)
        {
            if(!pMP->isBad() && !sAlreadyFound.count(pMP))
            {
                //Project
                vec3f x3Dw = pMP->GetWorldPos();
                vec3f x3Dc = Rcw * x3Dw + tcw;

                const float xc = x3Dc(0);
                const float yc = x3Dc(1);
                const float invzc = 1.0f / x3Dc(2);

                const float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
                const float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;

                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;

                // Compute predicted scale level
                vec3f PO = x3Dw - Ow;
                float dist3D = PO.norm();

                const float maxDistance = pMP->GetMaxDistanceInvariance();
                const float minDistance = pMP->GetMinDistanceInvariance();

                // Depth must be inside the scale pyramid of the image
                if(dist3D<minDistance || dist3D>maxDistance)
                    continue;

                // Search in a window
                float predictedSize = pMP->PredictSize(dist3D);
                const float radius = radiusScale * radiusTh * predictedSize;

                const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius,
                                                                                (predictedSize / CurrentFrame.sizeTolerance),(predictedSize * CurrentFrame.sizeTolerance));

                if(vIndices2.empty())
                    continue;

                const cv::Mat refDescriptor = pMP->GetDescriptor();
                Descriptor_Distance_Type bestDist{highestPossibleDistance};
                int bestIdx2{-1};

                for(vector<size_t>::const_iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.pts[i2])
                        continue;

                    const cv::Mat &descriptor = CurrentFrame.mDescriptors.row(i2);
                    const Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,pMP->descriptorType);

                    if(descDist < bestDist)
                    {
                        bestDist = descDist;
                        bestIdx2 = i2;
                    }
                }

                if(bestDist <= descDistanceTh)
                {
                    CurrentFrame.pts[bestIdx2]=pMP;
                    nMatches++;

                    if(mbCheckOrientation)
                        updateRotationHistogram(rotHist,bestIdx2, pKF->mvKeysUn[i],CurrentFrame.mvKeysUn[bestIdx2],rotFactor,HISTO_LENGTH);
                }
            }
        }
    }

    if(mbCheckOrientation)
        filterMatchesWithOrientation(rotHist,CurrentFrame.pts,nMatches);

    return nMatches;
}

Descriptor_Distance_Type FeatureMatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b, const DescriptorType& descriptorType_)
{
    switch(descriptorType_) {
        // DescriptorDistance
        case DESC_ANYFEATNONBIN:
            return DescriptorDistance_anyFeatureNonBin(a,b);
        case DESC_ANYFEATBIN:
            return DescriptorDistance_anyFeatureBin(a,b);
        case DESC_R2D2:
            return DescriptorDistance_r2d2_128(a,b);
        case DESC_SIFT128:
            return DescriptorDistance_sift128(a,b);
        case DESC_KAZE64:
            return DescriptorDistance_kaze64(a,b);
        case DESC_SURF64:
            return DescriptorDistance_surf64(a,b);
        case DESC_BRISK:
            return DescriptorDistance_brisk48(a,b);
        case DESC_AKAZE61:
            return DescriptorDistance_akaze61(a,b);
        case DESC_ORB:
            return DescriptorDistance_orb32(a,b);
    }
}

void FeatureMatcher::setDescriptorDistanceThresholds(const string &feature_settings_yaml_file) {
#ifdef VANILLA_ORB_SLAM2
        return;
#endif
    cv::FileStorage fSettings(feature_settings_yaml_file, cv::FileStorage::READ);
    const float matchingTh = fSettings["FeatureMatcher.matchingTh"];
    cout << endl  << "Loading Feature Matcher Settings from : " << feature_settings_yaml_file << endl;
    cout <<  "- matchingTh: " << matchingTh << endl;
    FeatureMatcher::TH_LOW = matchingTh;
    FeatureMatcher::TH_HIGH = FeatureMatcher::TH_LOW;
    FeatureMatcher::descDistTh_low_reloc = FeatureMatcher::TH_LOW;
    FeatureMatcher::descDistTh_high_reloc = FeatureMatcher::TH_LOW;
}
void FeatureMatcher::setDescriptorDistanceThresholds(const std::vector<Descriptor_Distance_Type>& descriptorDistances_,
                                                     const std::vector<int>& numCandidates_,const DescriptorType& descriptorType){
    #ifdef VANILLA_ORB_SLAM2
    return;
    #endif

    /*if(descriptorDistances_.empty())
        return;*/

    //float th{0.0};
    /*int totalNumCandidates{0};
    for(int i{0}; i < descriptorDistances_.size();i++){
        th += descriptorDistances_[i] / float(numCandidates_[i]);
        totalNumCandidates += numCandidates_[i];
        //cout << descriptorDistances_[i] << " " << numCandidates_[i] << endl;
    }

    //th *= float(totalNumCandidates)/(float(descriptorDistances_.size() * descriptorDistances_.size()));*/

    //FeatureMatcher::TH_LOW = GetNominalMatchingThreshold(descriptorType);
    FeatureMatcher::TH_HIGH = FeatureMatcher::TH_LOW;
    FeatureMatcher::descDistTh_low_reloc = FeatureMatcher::TH_LOW;
    FeatureMatcher::descDistTh_high_reloc = FeatureMatcher::TH_LOW;

    if(verbosity >= MEDIUM){
        cout << "ORBmatcher::TH_LOW (weight. avg) = " << FeatureMatcher::TH_LOW << endl;
        cout << "ORBmatcher::TH_HIGH (weight. avg) = " << FeatureMatcher::TH_HIGH << endl;
        cout << "ORBmatcher::descDistTh_low_reloc (weight. avg) = " << FeatureMatcher::descDistTh_low_reloc << endl;
        cout << "ORBmatcher::descDistTh_high_reloc (weight. avg) = " << FeatureMatcher::descDistTh_high_reloc << endl;
    }

}

    vector<vector<int>> FeatureMatcher::initRotationHistogram(float& rotFactor, const int& histLength){
        vector<vector<int>> rotHist;
        rotHist.resize(histLength);
        for(int i = 0; i < histLength; i++)
            rotHist[i].reserve(500);
        rotFactor = 1.0f / float(histLength);return rotHist;
    }

    void FeatureMatcher::updateRotationHistogram(vector<vector<int>>& rotHist,
                                                     const KeypointIndex& idx,
                                                     const cv::KeyPoint& keyPt, const cv::KeyPoint& refKeyPt,
                                                     const float& rotFactor, const int& histLength){
        float rot = keyPt.angle - refKeyPt.angle;
        if(rot < 0.0)
            rot += 360.0f;
        int bin = (int) round(rot * rotFactor);
        if(bin == histLength)
            bin = 0;
        assert(bin >= 0 && bin < histLength);
        rotHist[bin].push_back(idx);
    }

    void FeatureMatcher::filterMatchesWithOrientation(vector<vector<int>>& rotHist, vector<Pt>& points, int& nMatches){
        int ind1{-1}, ind2{-1}, ind3{-1};
        computeThreeMaxima(rotHist,ind1,ind2,ind3);

        for(int i = 0; i < rotHist.size(); i++){
            if(i == ind1 || i == ind2 || i == ind3)
                continue;
            for(int j : rotHist[i]){
                points[j] = static_cast<Pt>(nullptr);
                nMatches--;
            }
        }
    }

    void FeatureMatcher::filterMatchesWithOrientation(vector<vector<int>>& rotHist, vector<int>& matches, int& nMatches){
        int ind1{-1}, ind2{-1}, ind3{-1};
        computeThreeMaxima(rotHist,ind1,ind2,ind3);

        for(int i = 0; i < rotHist.size(); i++){
            if(i == ind1 || i == ind2 || i == ind3)
                continue;
            for(int idx1 : rotHist[i]){
                if(matches[idx1] >= 0){
                    nMatches--;
                    matches[idx1] =-1;
                }
            }
        }
    }

    void FeatureMatcher::computeThreeMaxima(vector<vector<int>>& rotHist, int &ind1, int &ind2, int &ind3){
        int max1{0}, max2{0}, max3{0};
        for(int i = 0; i < rotHist.size(); i++)
        {
            const int s = (int) rotHist[i].size();
            if(s > max1)
            {
                max3=max2;
                max2=max1;
                max1=s;
                ind3=ind2;
                ind2=ind1;
                ind1=i;
            }
            else if(s > max2)
            {
                max3=max2;
                max2=s;
                ind3=ind2;
                ind2=i;
            }
            else if(s > max3)
            {
                max3=s;
                ind3=i;
            }
        }

        if(max2 < 0.1f*(float)max1)
        {
            ind2=-1;
            ind3=-1;
        }
        else if(max3 < 0.1f*(float)max1)
        {
            ind3=-1;
        }
    }
    } //namespace ORB_SLAM

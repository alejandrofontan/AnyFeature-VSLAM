
#include<stdint-gcc.h>
#include <memory>
#include<limits.h>

#include "FeatureMatcher.h"
#include "Converter.h"
#include "MathFunctions.h"
#include <cmath>

#include "Feature_orb32.h"
#include "Feature_akaze61.h"
#include "Feature_brisk48.h"
#include "Feature_surf64.h"
#include "Feature_kaze64.h"
#include "Feature_sift128.h"
#include "Feature_r2d2_128.h"
#include "Feature_anyFeatBin.h"
#include "Feature_anyFeatNonBin.h"
#include "Feature_aliked128.h"


#include<opencv2/core/core.hpp>
//#include<opencv2/features2d/features2d.hpp>

#include "DBoW2/FeatureVector.h"

using namespace std;

namespace ANYFEATURE_VSLAM
{
    
std::map<FeatureType, Descriptor_Distance_Type> FeatureMatcher::TH_HIGH = {};
std::map<FeatureType, Descriptor_Distance_Type> FeatureMatcher::TH_LOW = {};
std::map<FeatureType, Descriptor_Distance_Type> FeatureMatcher::descDistTh_high_reloc = {};
std::map<FeatureType, Descriptor_Distance_Type> FeatureMatcher::descDistTh_low_reloc = {};

VerbosityLevel FeatureMatcher::verbosity{MEDIUM};

const int FeatureMatcher::HISTO_LENGTH = 30;
float FeatureMatcher::radiusScale{1.15f};

FeatureMatcher::FeatureMatcher(const int& imageWidth, const int& imageHeight, float nnratio, bool checkOri): 
    mfNNratio(nnratio), mbCheckOrientation(checkOri), imageWidth(imageWidth), imageHeight(imageHeight)
{
    std::cout << "Initializing SiftMatchGPU..." << std::endl;
    sift_match_gpu_ = SiftMatchGPU();
    sift_match_gpu_.SetLanguage(SiftMatchGPU::SIFTMATCH_CUDA);
    if (sift_match_gpu_.VerifyContextGL() == 0) {
       std::cout << "Initialization failed!" << std::endl;
    }
    int max_supported = 4000;
    sift_match_gpu_ .Allocate(max_supported, 1);
    std::cout << "Finished initializing SiftMatchGPU." << std::endl;

    torch_device = std::make_shared<torch::Device>(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU);
    matcher_lightglue = std::make_shared<matcher::LightGlue>();
    matcher_lightglue->to(*torch_device);
}

// SearchBruteForce Keyframe-Frame
// Tracking::TrackReferenceKeyframe & Tracking::Relocalization
int FeatureMatcher::SearchBruteForce(const Keyframe& keyframe, const Frame &frame, vector<Pt>& mapPointMatches, const FeatureType& featType)
{
    mapPointMatches.clear();

    // Ensure both frames contain the requested feature type
    auto it1 = keyframe->mDescriptors.find(featType);
    auto it2 = frame.mDescriptors.find(featType);
    if (it1 == keyframe->mDescriptors.end() || it2 == frame.mDescriptors.end()) 
        return 0; 
    
    std::vector<cv::DMatch> matches = featureMatching(keyframe->mDescriptors.at(featType), frame.mDescriptors.at(featType),
        keyframe->mvKeysUn.at(featType), frame.mvKeysUn.at(featType), featType, sBF_kf_lightglue, sBF_kf_robustMatching, sBF_kf_outlierMethod);

    mapPointMatches = vector<Pt>(frame.N.at(featType), static_cast<Pt>(NULL));
    const vector<Pt> mapPointsKF = keyframe->GetMapPointMatches(featType);

    int validMatches = 0;
    for(const auto& m : matches) {
        Pt pMP = mapPointsKF[m.queryIdx];
        if(!pMP || (pMP->isBad()))
            continue;

        mapPointMatches[m.trainIdx] = pMP;  
        validMatches++;
    }
    return validMatches;
}

// SearchForInitialization Frame-Frame
// Tracking::MonocularInitialization
int FeatureMatcher::SearchForInitialization(const Frame &F1, const Frame &F2, 
    vector<cv::Point2f> &pointsPrevMatched, vector<int> &matches12, const FeatureType& featType)
{
    matches12.clear();

    // Ensure both frames contain the requested feature type
    auto it1 = F1.mDescriptors.find(featType);
    auto it2 = F2.mDescriptors.find(featType);
    if (it1 == F1.mDescriptors.end() || it2 == F2.mDescriptors.end()) 
        return 0; 
    
    std::vector<cv::DMatch> matches = featureMatching(F1.mDescriptors.at(featType), F2.mDescriptors.at(featType), 
         F1.mvKeysUn.at(featType), F2.mvKeysUn.at(featType), featType, sFI_ff_lightglue, sFI_ff_robustMatching, sFI_ff_outlierMethod);

    int numMatches = 0;
    matches12 = vector<int>(F1.mvKeysUn.at(featType).size(),-1);
    for(const auto& m : matches) {
        if (F1.keyPtsSize.at(featType)[m.queryIdx] > 1.0)
            continue;
        if (F2.keyPtsSize.at(featType)[m.trainIdx] > 1.0)
            continue;
        if (F1.keyPtsSize.at(featType)[m.queryIdx] != F2.keyPtsSize.at(featType)[m.trainIdx])
            continue;

        matches12[m.queryIdx] = m.trainIdx;
        pointsPrevMatched[m.queryIdx] = F2.mvKeysUn.at(featType)[m.trainIdx].pt;
        numMatches++;
    }
    return numMatches;
}

// SearchForTriangulation Keyframe-Keyframe
// LocalMapping::CreateNewMapPoints
int FeatureMatcher::SearchForTriangulation(const Keyframe& keyframe1, const Keyframe& keyframe2, const mat3f& F12,
                                           vector<pair<size_t, size_t> > &matchedPairs, 
                                           const FeatureType& featType){                                            
    matchedPairs.clear();
    
    // Ensure both frames contain the requested feature type
    auto it1 = keyframe1->mDescriptors.find(featType);
    auto it2 = keyframe2->mDescriptors.find(featType);
    if (it1 == keyframe1->mDescriptors.end() || it2 == keyframe2->mDescriptors.end()) 
        return 0; 
    
    std::vector<cv::DMatch> matches = featureMatching(keyframe1->mDescriptors.at(featType), keyframe2->mDescriptors.at(featType), 
        keyframe1->mvKeysUn.at(featType), keyframe2->mvKeysUn.at(featType), featType, sFT_kk_lightglue, sFT_kk_robustMatching, sFT_kk_outlierMethod);

    matchedPairs.reserve(matches.size());
    for(const auto& m : matches) {
        // Only triangulate points that don't already have a 3D MapPoint
        if(!keyframe1->GetMapPoint(m.queryIdx, featType) && !keyframe2->GetMapPoint(m.trainIdx, featType))
            matchedPairs.emplace_back(static_cast<size_t>(m.queryIdx), static_cast<size_t>(m.trainIdx));   
    }
    return matchedPairs.size();
}

// SearchBruteForce Frame-Frame
// Tracking::TrackWithMotionModel
int FeatureMatcher::SearchBruteForce(Frame &CurrentFrame, const Frame &LastFrame, const FeatureType& featType)
{

    // Ensure both frames contain the requested feature type
    auto it1 = CurrentFrame.mDescriptors.find(featType);
    auto it2 = LastFrame.mDescriptors.find(featType);
    if (it1 == CurrentFrame.mDescriptors.end() || it2 == LastFrame.mDescriptors.end()) 
        return 0; 
    
    std::vector<cv::DMatch> matches = featureMatching(CurrentFrame.mDescriptors.at(featType), LastFrame.mDescriptors.at(featType), 
         CurrentFrame.mvKeysUn.at(featType), LastFrame.mvKeysUn.at(featType), featType, sBF_ff_lightglue, sBF_ff_robustMatching, sBF_ff_outlierMethod);

    int numMatches = 0;
    for(const auto& m : matches) {
        Pt pMP = LastFrame.pts.at(featType)[m.trainIdx];
        if(!pMP || (pMP->isBad()))
            continue;
        if(LastFrame.mvbOutlier.at(featType)[m.trainIdx])
            continue;

        CurrentFrame.pts.at(featType)[m.queryIdx] = pMP;
        numMatches++;
    }
    return numMatches;
}

// SearchByProjection 1
// TrackLocalMap
int FeatureMatcher::SearchByProjection(Frame &frame, const vector<Pt> &mapPoints){

    std::map<FeatureType, std::map<KeyframeId, std::vector<Pt>>> mapPointsByType;
    for (const auto& pt : mapPoints) 
        mapPointsByType[pt->featureType][pt->GetCurrentRefKeyframe()->keyId].push_back(pt);

    std::map<FeatureType, std::vector<size_t>> toBeMatched;
    std::map<FeatureType, cv::Mat> frameDescriptors;
    
    for(const auto& [ft, pts]: frame.pts){
        int ptIdx{-1};
        for(const auto& pt : pts){
            ptIdx++;
            if(pt && (pt->NumberOfObservations() > 0))
                continue;
            cv::Mat desc = frame.mDescriptors.at(ft).row(ptIdx);      
            frameDescriptors[ft].push_back(desc);  
            toBeMatched[ft].push_back(ptIdx);
        }
    }

    int numMatches = 0;
    for(auto& [ft, keyframe]: mapPointsByType){
        std::vector<int> numMachedPoints;
        
        if(toBeMatched[ft].empty())
            continue;

            for(auto& [kfId, pts]: keyframe){
            numMachedPoints.push_back(0);

            cv::Mat descriptors;
            for(auto pt: pts){
                cv::Mat desc = pt->GetDescriptor();
                if (desc.rows > 1 && desc.cols > 0) desc = desc.row(0); 
                descriptors.push_back(desc);
            }
            
            std::vector<cv::DMatch> matches = featureMatching(frameDescriptors.at(ft), descriptors, ft);
            //cv::BFMatcher(getNormType(ft), true).match(frameDescriptors.at(ft), descriptors, matches);

            for(const auto& m : matches) {
                Pt pMP = pts[m.trainIdx];
                if(!pMP || (pMP->isBad()))
                    continue;
                
                int ptIdx = toBeMatched.at(ft)[m.queryIdx];
                if(frame.pts.at(ft)[ptIdx])
                    if(frame.pts.at(ft)[ptIdx]->NumberOfObservations() > 0)
                        continue;

                float radiusTh = 3.0f;
                const float predictedSize = pMP->trackSize;
                float r = radiusScale * radiusTh *  RadiusByViewingCos(pMP->trackViewCos) * predictedSize;

                const vector<size_t> vIndices = frame.GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY, r,
                                    (pMP->trackSize / frame.sizeTolerance),(pMP->trackSize * frame.sizeTolerance), ft);
                if(vIndices.empty())
                    continue;
                for (const auto& idx : vIndices){
                    if(idx == ptIdx){
                        frame.pts.at(ft)[ptIdx] = pMP;
                        auto itMap = toBeMatched.find(ft);
                        if (itMap != toBeMatched.end()) {
                            auto& indices = itMap->second;
                            indices.erase(std::remove(indices.begin(), indices.end(), m.queryIdx), indices.end());
                            for (auto& idx : indices) {
                                if (idx > m.queryIdx) {
                                    idx--;
                                }
                            }
                        }
                        frameDescriptors.at(ft).row(m.queryIdx).release();
                        numMatches++;
                        numMachedPoints.back()++;
                        break;
                    }
                }
                if (toBeMatched.at(ft).empty())
                    break;
                if (numMachedPoints.size() > 1){
                    if (numMachedPoints.back() == 0 && numMachedPoints[numMachedPoints.size() - 2] == 0){
                        break;
                    }
                }
            }
        }
    }
    return numMatches;
        
}

// Fuse 1
// Local Mapping
int FeatureMatcher::Fuse(Keyframe pKF, const vector<Pt> &vpMapPoints, const float& radiusTh, const FeatureType& featType)
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

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius, featType);
        if(vIndices.empty())
            continue;
        // Match to the most similar keypoint in the radius
        const cv::Mat refDescriptor = pMP->GetDescriptor();
        Descriptor_Distance_Type bestDist{highestPossibleDistance};
        int bestIdx{-1};
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF->mvKeysUn.at(featType)[idx];

            //const float keyPtSize = pKF->GetKeyPtSize(KeypointIndex (idx), featType);
            //if((keyPtSize < predictedSize / pKF->sizeTolerance) || (keyPtSize > predictedSize * pKF->sizeTolerance))
            //    continue;

            if(pKF->mvuRight.at(featType)[idx]>=0)
            {
                // Check reprojection error in stereo
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float &kpr = pKF->mvuRight.at(featType)[idx];
                const float ex = u-kpx;
                const float ey = v-kpy;
                const float er = ur-kpr;
                const float e2 = ex*ex+ey*ey+er*er;

                if(e2 * pKF->GetKeyPt1DInf(KeypointIndex (idx), featType) > 7.8)
                    continue;
            }
            else
            {
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float ex = u-kpx;
                const float ey = v-kpy;
                const float e2 = ex*ex+ey*ey;

                if(e2 * pKF->GetKeyPt1DInf(KeypointIndex (idx), featType) > 5.99)
                    continue;
            }

            const cv::Mat &descriptor = pKF->mDescriptors.at(featType).row(idx);
            const Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,pMP->descriptorType);

            if(descDist < bestDist)
            {
                bestDist = descDist;
                bestIdx = idx;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if(bestDist <= TH_LOW[featType])
        {
            Pt pMPinKF = pKF->GetMapPoint(bestIdx, featType);
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

// SEARCH BY PROJECTION 1 ?????
int FeatureMatcher::SearchByProjection(Frame &F, const vector<Pt> &vpMapPoints, const float& radiusTh)
{
    std::cout << "SEARCH BY PROJECTION 1 ?????" << std::endl;
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

        const FeatureType featType = pMP->featureType;
        // The size of the window will depend on the viewing direction
        const float predictedSize = pMP->trackSize;
        float r = radiusScale * radiusTh *  RadiusByViewingCos(pMP->trackViewCos) * predictedSize;

        const vector<size_t> vIndices =
                F.GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY, r,
                                    (pMP->trackSize / F.sizeTolerance),(pMP->trackSize * F.sizeTolerance), featType);

        if(vIndices.empty())
            continue;

        const cv::Mat refDescriptor = pMP->GetDescriptor();
        Descriptor_Distance_Type bestDist{highestPossibleDistance},bestDist2{highestPossibleDistance};
        //float bestSize{-1.0f},bestSize2{-1.0f};
        int bestIdx{-1};

        // Get best and second matches with near keypoints
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            if(F.pts.at(featType)[idx])
                if(F.pts.at(featType)[idx]->NumberOfObservations() > 0)
                    continue;

            if(F.mvuRight.at(featType)[idx]>0)
            {
                const float er = fabs(pMP->mTrackProjXR-F.mvuRight.at(featType)[idx]);
                if(er > r * pMP->trackSigma)
                    continue;
            }

            const cv::Mat &descriptor = F.mDescriptors.at(featType).row(idx);
            const Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor, pMP->descriptorType);

            if(descDist < bestDist)
            {
                bestDist2 = bestDist;
                bestDist = descDist;
                bestIdx = idx;
                //bestSize2 = bestSize;
                //bestSize = F.GetKeyPtSize(KeypointIndex(idx), featType);
            }
            else if(descDist < bestDist2)
            {
                bestDist2 = descDist;
                //bestSize2 = F.GetKeyPtSize(KeypointIndex(idx), featType);
            }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if(bestDist <= TH_HIGH[featType])
        {
            //if((bestSize / bestSize2 < F.sizeTolerance) && (bestSize / bestSize2 > F.invSizeTolerance) && (bestSize2 > 0.0f)){
                if(bestDist > mfNNratio * bestDist2){
                    continue;
                }
            //}
            F.pts.at(featType)[bestIdx]=pMP;
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

// SearchByProjection 2
// Compute Sim3
int FeatureMatcher::SearchByProjection(Keyframe pKF, const mat4f& Scw, const vector<Pt> &vpPoints, vector<Pt> &vpMatched, 
    const float& radiusTh, const FeatureType& featType)
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
        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius, featType);

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

            //const float keyPtSize = pKF->GetKeyPtSize(KeypointIndex (idx), featType);
            //if((keyPtSize < predictedSize / pKF->sizeTolerance) || (keyPtSize > predictedSize * pKF->sizeTolerance))
            //    continue;

            const cv::Mat &descriptor = pKF->mDescriptors.at(featType).row(idx);
            const Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,pMP->descriptorType);

            if(descDist < bestDist)
            {
                bestDist = descDist;
                bestIdx = idx;
            }
        }

        if(bestDist <= TH_LOW[featType])
        {
            vpMatched[bestIdx]=pMP;
            nmatches++;
        }

    }

    return nmatches;
}

// SearchByBoW 2
// ComputeSim3
int FeatureMatcher::SearchByBoW(Keyframe pKF1, Keyframe pKF2, vector<Pt > &vpMatches12, const FeatureType&  featType)
{

    const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeysUn.at(featType);
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
    const vector<Pt> vpMapPoints1 = pKF1->GetMapPointMatches(featType);
    const cv::Mat &Descriptors1 = pKF1->mDescriptors.at(featType);

    const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn.at(featType);
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
    const vector<Pt> vpMapPoints2 = pKF2->GetMapPointMatches(featType);
    const cv::Mat &Descriptors2 = pKF2->mDescriptors.at(featType);

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

                if(bestDist1 < TH_LOW[featType])
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

// Fuse 2
// Loop Closing
int FeatureMatcher::Fuse(Keyframe pKF, const mat4f& Scw, const vector<Pt> &vpPoints, const float& radiusTh, vector<Pt> &vpReplacePoint, const FeatureType& featType)
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
    const set<Pt> spAlreadyFound = pKF->GetMapPoints(featType);

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

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius, featType);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat refDescriptor = pMP->GetDescriptor();
        Descriptor_Distance_Type bestDist{highestPossibleDistance};
        int bestIdx{-1};

        for(vector<size_t>::const_iterator vit=vIndices.begin(); vit!=vIndices.end(); vit++)
        {
            const size_t idx = *vit;

            //const float keyPtSize = pKF->GetKeyPtSize(KeypointIndex (idx), featType);
            //if((keyPtSize < predictedSize / pKF->sizeTolerance) || (keyPtSize > predictedSize * pKF->sizeTolerance))
            //    continue;

            const cv::Mat &descriptor = pKF->mDescriptors.at(featType).row(idx);
            Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,pMP->descriptorType);

            if(descDist < bestDist)
            {
                bestDist = descDist;
                bestIdx = idx;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if(bestDist <= TH_LOW[featType])
        {
            Pt pMPinKF = pKF->GetMapPoint(bestIdx, featType);
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
                                 const float &s12, const mat3f  &R12, const vec3f &t12, const float& radiusTh, const FeatureType& featType)
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

    const vector<Pt> vpMapPoints1 = pKF1->GetMapPointMatches(featType);
    const int N1 = vpMapPoints1.size();

    const vector<Pt> vpMapPoints2 = pKF2->GetMapPointMatches(featType);
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

        const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u,v,radius, featType);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat refDescriptor = pMP->GetDescriptor();
        Descriptor_Distance_Type bestDist{highestPossibleDistance};
        int bestIdx{-1};

        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF2->mvKeysUn.at(featType)[idx];

            //const float keyPtSize = pKF2->GetKeyPtSize(KeypointIndex (idx), featType);
            //if((keyPtSize < predictedSize / pKF2->sizeTolerance) || (keyPtSize > predictedSize * pKF2->sizeTolerance))
            //    continue;

            const cv::Mat &descriptor = pKF2->mDescriptors.at(featType).row(idx);
            const Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,pMP->descriptorType);

            if(descDist < bestDist)
            {
                bestDist = descDist;
                bestIdx = idx;
            }
        }

        if(bestDist <= TH_HIGH[featType])
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

        const vector<size_t> vIndices = pKF1->GetFeaturesInArea(u,v,radius, featType);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat refDescriptor = pMP->GetDescriptor();
        Descriptor_Distance_Type bestDist{highestPossibleDistance};
        int bestIdx{-1};

        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF1->mvKeysUn.at(featType)[idx];

            //const float keyPtSize = pKF1->GetKeyPtSize(KeypointIndex (idx), featType);
            //if((keyPtSize < predictedSize / pKF1->sizeTolerance) || (keyPtSize > predictedSize * pKF1->sizeTolerance))
            //    continue;

            const cv::Mat &descriptor = pKF1->mDescriptors.at(featType).row(idx);
            const Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,pMP->descriptorType);

            if(descDist < bestDist)
            {
                bestDist = descDist;
                bestIdx = idx;
            }
        }

        if(bestDist <= TH_HIGH[featType])
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

// SearchByProjection 4
// Relocalization
int FeatureMatcher::SearchByProjection(Frame &CurrentFrame, Keyframe pKF, const set<Pt> &sAlreadyFound, const float& radiusTh,
     const bool& useHighMatchingThreshold, const FeatureType& featType)
{
    Descriptor_Distance_Type descDistanceTh = descDistTh_low_reloc[featType];
    if(useHighMatchingThreshold)
        descDistanceTh = descDistTh_high_reloc[featType];

    const mat3f Rcw = CurrentFrame.Tcw.block<3,3>(0,0);
    const vec3f tcw = CurrentFrame.Tcw.block<3,1>(0,3);
    const vec3f Ow = -Rcw.transpose() * tcw;

    // Rotation Histogram (to check rotation consistency)
    int nMatches{0};
    float rotFactor{};
    vector<vector<int>> rotHist = initRotationHistogram(rotFactor,HISTO_LENGTH);

    const vector<Pt> vpMPs = pKF->GetMapPointMatches(featType);

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
                                                                                (predictedSize / CurrentFrame.sizeTolerance),
                                                                                (predictedSize * CurrentFrame.sizeTolerance), 
                                                                                featType);

                if(vIndices2.empty())
                    continue;

                const cv::Mat refDescriptor = pMP->GetDescriptor();
                Descriptor_Distance_Type bestDist{highestPossibleDistance};
                int bestIdx2{-1};

                for(vector<size_t>::const_iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.pts.at(featType)[i2])
                        continue;

                    const cv::Mat &descriptor = CurrentFrame.mDescriptors.at(featType).row(i2);
                    const Descriptor_Distance_Type descDist = DescriptorDistance(refDescriptor,descriptor,pMP->descriptorType);

                    if(descDist < bestDist)
                    {
                        bestDist = descDist;
                        bestIdx2 = i2;
                    }
                }

                if(bestDist <= descDistanceTh)
                {
                    CurrentFrame.pts.at(featType)[bestIdx2]=pMP;
                    nMatches++;

                    if(mbCheckOrientation)
                        updateRotationHistogram(rotHist,bestIdx2, pKF->mvKeysUn.at(featType)[i],CurrentFrame.mvKeysUn.at(featType)[bestIdx2],rotFactor,HISTO_LENGTH);
                }
            }
        }
    }

    if(mbCheckOrientation)
        filterMatchesWithOrientation(rotHist,CurrentFrame.pts.at(featType),nMatches);

    return nMatches;
}

Descriptor_Distance_Type FeatureMatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b, const DescriptorType& descriptorType_)
{
    switch(descriptorType_) {
        // DescriptorDistance
        case DESC_ALIKED128:
             return DescriptorDistance_aliked128(a,b);
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

cv::NormTypes FeatureMatcher::getNormType(const FeatureType& featureType_){
    switch(featureType_) {
        case FEAT_ALIKED128:
            return cv::NORM_L2;
        case FEAT_ANYFEATNONBIN:
            return cv::NORM_L2;
        case FEAT_ANYFEATBIN:
            return cv::NORM_HAMMING;
        case FEAT_R2D2:
            return cv::NORM_L2;
        case FEAT_SIFT128:
            return cv::NORM_L2;
        case FEAT_KAZE64:
            return cv::NORM_L2;
        case FEAT_SURF64:
            return cv::NORM_L2;
        case FEAT_BRISK:
            return cv::NORM_HAMMING;
        case FEAT_AKAZE61:
            return cv::NORM_HAMMING;
        case FEAT_ORB:
            return cv::NORM_HAMMING;
    }
}

void FeatureMatcher::setDescriptorDistanceThresholds(const string &feature_settings_yaml_file, const FeatureType& featureType) {

    cv::FileStorage fSettings(feature_settings_yaml_file, cv::FileStorage::READ);
    cout << endl  << "Loading Feature Matcher Settings from : " << feature_settings_yaml_file << endl;
    FeatureMatcher::TH_LOW[featureType] = fSettings["FeatureMatcher.TH_LOW"];
    FeatureMatcher::TH_HIGH[featureType] = fSettings["FeatureMatcher.TH_HIGH"];
    FeatureMatcher::descDistTh_low_reloc[featureType] = fSettings["FeatureMatcher.descDistTh_high_reloc"];
    FeatureMatcher::descDistTh_high_reloc[featureType] = fSettings["FeatureMatcher.descDistTh_low_reloc"];
    cout <<  "- TH_LOW: " << FeatureMatcher::TH_LOW[featureType] << endl;
    cout <<  "- TH_HIGH: " << FeatureMatcher::TH_HIGH[featureType] << endl;
    cout <<  "- descDistTh_low_reloc: " << FeatureMatcher::descDistTh_low_reloc[featureType] << endl;
    cout <<  "- descDistTh_high_reloc: " << FeatureMatcher::descDistTh_high_reloc[featureType] << endl;
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

    std::vector<cv::DMatch> FeatureMatcher::featureMatching(const cv::Mat& desc1, const cv::Mat& desc2, const FeatureType& ft){
        std::vector<cv::DMatch> matches;
        switch(ft) {
            case FEAT_ALIKED128:
            case FEAT_ANYFEATNONBIN:
            case FEAT_KAZE64:
            case FEAT_SURF64:
            case FEAT_R2D2:
                bf_matcher_L2.match(desc1, desc2, matches);
                break;
            case FEAT_SIFT128:
                {
                    sift_match_gpu_.SetDescriptors(0, desc1.rows, desc1.ptr<float>());
                    sift_match_gpu_.SetDescriptors(1, desc2.rows, desc2.ptr<float>());

                    const int max_out = 4000;
                    uint32_t (*match_buffer)[2] = new uint32_t[max_out][2];

                    int num_matches = sift_match_gpu_.GetSiftMatch(max_out, match_buffer, 0.7f, 0.8f, 1);

                    matches.clear();
                    matches.reserve(num_matches);
                    for (int i = 0; i < num_matches; ++i) {
                        matches.emplace_back(
                            static_cast<int>(match_buffer[i][0]), 
                            static_cast<int>(match_buffer[i][1]), 
                            0.0f);
                    }
                    delete[] match_buffer;
                    break;
                }
            case FEAT_ANYFEATBIN:
            case FEAT_BRISK:
            case FEAT_AKAZE61:
            case FEAT_ORB:
                bf_matcher_hamming.match(desc1, desc2, matches);
                break;
        }
        return matches;
    }

    std::vector<cv::DMatch> FeatureMatcher::featureMatching(const cv::Mat& desc1, const cv::Mat& desc2, 
        const std::vector<cv::KeyPoint>& kps1, const std::vector<cv::KeyPoint>& kps2, const FeatureType& ft, 
        bool lightglue, bool robustMatching, int outlierMehod){

        std::vector<cv::DMatch> matches;
        switch(ft) {
            case FEAT_SIFT128:
            case FEAT_ALIKED128:
                if (lightglue){
                    matches = lightglueMatching(kps1, desc1, kps2, desc2, 0.0f);
                    break;
                }
            case FEAT_ANYFEATNONBIN:
            case FEAT_R2D2:
            case FEAT_KAZE64:
            case FEAT_SURF64:
                bf_matcher_L2.match(desc1, desc2, matches);
                break;
            case FEAT_ANYFEATBIN:
            case FEAT_BRISK:
            case FEAT_AKAZE61:
            case FEAT_ORB:
                bf_matcher_hamming.match(desc1, desc2, matches);
                break;
        }

        if (robustMatching)
            return robustFeatureMatching(matches, kps1, kps2, outlierMehod);
        return matches;
    }

    std::vector<cv::DMatch> FeatureMatcher::robustFeatureMatching(std::vector<cv::DMatch>& matches,
        const std::vector<cv::KeyPoint>& kps1, const std::vector<cv::KeyPoint>& kps2, int outlierMehod){

        std::sort(matches.begin(), matches.end(),
            [](const cv::DMatch& a, const cv::DMatch& b) { return a.distance < b.distance; });
        
        const size_t maxForRansac = 2000; // tune for speed
        if (matches.size() > maxForRansac) matches.resize(maxForRansac);

        // Build point correspondences ---
        std::vector<cv::Point2f> pts1; pts1.reserve(matches.size());
        std::vector<cv::Point2f> pts2; pts2.reserve(matches.size());
        for (const auto& m : matches) {
            // Safety: ensure indices are valid
            if (m.queryIdx < 0 || m.queryIdx >= (int)kps1.size()) continue;
            if (m.trainIdx < 0 || m.trainIdx >= (int)kps2.size()) continue;

            pts1.push_back(kps1[m.queryIdx].pt);
            pts2.push_back(kps2[m.trainIdx].pt);
        }

        if (pts1.size() < 8) return matches; // not enough after filtering

        const double reprojThreshold = 3.0;  // pixels
        const double confidence      = 0.95;

        std::vector<uchar> inlierMask;
        cv::Mat F = cv::findFundamentalMat(
            pts1, pts2,
            outlierMehod,
            reprojThreshold,
            confidence,
            inlierMask
        );

        if (F.empty() || inlierMask.size() != pts1.size())
            return matches;

        std::vector<cv::DMatch> inlierMatches;
        inlierMatches.reserve(matches.size());

        // Rebuild matchesUsed aligned with pts1/pts2:
        std::vector<cv::DMatch> matchesUsed;
        matchesUsed.reserve(matches.size());

        for (const auto& m : matches) {
            if (m.queryIdx < 0 || m.queryIdx >= (int)kps1.size()) continue;
            if (m.trainIdx < 0 || m.trainIdx >= (int)kps2.size()) continue;
            matchesUsed.push_back(m);
        }

        if (matchesUsed.size() != inlierMask.size())
            return matches; // alignment mismatch fallback

        for (size_t i = 0; i < inlierMask.size(); ++i) {
            if (inlierMask[i]) inlierMatches.push_back(matchesUsed[i]);
        }

        // std::cout << "FeatureMatcher::featureMatching: "
        //           << inlierMatches.size() << " inliers found out of "
        //           << matches.size() << " matches." << std::endl;

        return inlierMatches;
    }

    } //namespace ORB_SLAM

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

#include "Frame.h"
#include "Converter.h"
#include "FeatureMatcher.h"
#include <thread>

namespace ANYFEATURE_VSLAM
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :vocabulary(frame.vocabulary), featureExtractorLeft(frame.featureExtractorLeft), featureExtractorRight(frame.featureExtractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     pts(frame.pts), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     refKeyframe(frame.refKeyframe),
     sizeTolerance(frame.sizeTolerance),invSizeTolerance(frame.invSizeTolerance),
     keyPtsSigma2(frame.keyPtsSigma2),keyPtsInf(frame.keyPtsInf),keyPtsSize(frame.keyPtsSize),
     maxKeyPtSize(frame.maxKeyPtSize),maxKeyPtSigma(frame.maxKeyPtSigma)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(frame.Tcw(3,3) == 1.0f)
        SetPose(frame.Tcw);
}


Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
             shared_ptr<FeatureExtractor>& extractorLeft, shared_ptr<FeatureExtractor>& extractorRight,
             shared_ptr<Vocabulary> vocabulary, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :vocabulary(vocabulary),
    featureExtractorLeft(extractorLeft),featureExtractorRight(extractorRight),
    mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     refKeyframe(static_cast<Keyframe>(nullptr))
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    sizeTolerance = extractorLeft->GetScaleFactor();
    invSizeTolerance = 1.0f / sizeTolerance;

    // Feature extraction
    //thread threadLeft(&Frame::ExtractFeatures,this,0,imLeft);
    //thread threadRight(&Frame::ExtractFeatures,this,1,imRight);
    //threadLeft.join();
    //threadRight.join();

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoMatches(vocabulary->descriptorType);

    pts = vector<Pt>(N,static_cast<Pt>(nullptr));
    mvbOutlier = vector<bool>(N,false);


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp,
             shared_ptr<FeatureExtractor>& extractor,
             shared_ptr<Vocabulary> vocabulary, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :vocabulary(vocabulary),
    featureExtractorLeft(extractor),featureExtractorRight(static_cast<shared_ptr<FeatureExtractor>>(nullptr)),
    mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    sizeTolerance = featureExtractorLeft->GetScaleFactor();
    invSizeTolerance = 1.0f / sizeTolerance;

    // Feature extraction
    //ExtractFeatures(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    pts = vector<Pt>(N,static_cast<Pt>(nullptr));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}


Frame::Frame(const Image & img, const double &timeStamp,
             shared_ptr<FeatureExtractor>& extractor,
             shared_ptr<Vocabulary> vocabulary, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :vocabulary(vocabulary),
    featureExtractorLeft(extractor),featureExtractorRight(static_cast<shared_ptr<FeatureExtractor>>(nullptr)),
    mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    sizeTolerance = featureExtractorLeft->GetScaleFactor();
    invSizeTolerance = 1.0f / sizeTolerance;

    // Feature extraction
    ExtractFeatures(0,img);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    pts = vector<Pt>(N,static_cast<Pt>(nullptr));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(img.grayImg);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame::ExtractFeatures(int flag, const Image & img)
{
    if(flag==0){
#ifdef VANILLA_ORB_SLAM2
        (*featureExtractorLeft)(img,mvKeys,mDescriptors,keyPtsSigma2,keyPtsInf,keyPtsSize,true);
#else

        (*featureExtractorLeft)(img,mvKeys,mDescriptors,keyPtsSigma2,keyPtsInf,keyPtsSize);
#endif
        maxKeyPtSize = featureExtractorLeft->GetMaxKeyPtSize();
        maxKeyPtSigma = featureExtractorLeft->GetMaxKeyPtSigma();
    }
    else{
        std::cout << "This part of the code (Frame::ExtractORB) is not prepared to run with independent sigmas and sizes."<< std::endl;
        std::terminate();
        //(*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
    }

}

void Frame::SetPose(const mat4f& Tcw_)
{
    Tcw = Tcw_;
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{ 
    Rcw = Tcw.block<3,3>(0,0);
    tcw = Tcw.block<3,1>(0,3);
    Rwc = Rcw.transpose();
    twc = -Rwc * tcw;
}

bool Frame::isInFrustum(Pt pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    vec3f P = pMP->GetWorldPos();

    // 3D in camera coordinates
    const vec3f Pc = Rcw * P + tcw;
    const float &PcX = Pc(0);
    const float &PcY = Pc(1);
    const float &PcZ = Pc(2);

    // Check positive depth
    if(PcZ < 0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f / PcZ;
    const float u = fx * PcX * invz + cx;
    const float v = fy * PcY * invz + cy;

    if(u < mnMinX || u > mnMaxX)
        return false;
    if(v < mnMinY || v > mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const vec3f PO = P - twc;
    const float dist = PO.norm();

    if(dist < minDistance || dist > maxDistance)
        return false;

    // Check viewing angle
    vec3f Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn) / dist;

    if(viewCos < viewingCosLimit)
        return false;

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;

    pMP->trackSigma = pMP->PredictSigma(dist);
    pMP->trackSize = pMP->PredictSize(dist);
    pMP->trackViewCos = viewCos;

    return true;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r,
                                        const float& minSize, const float& maxSize) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];

                if(keyPtsSize[vCell[j]] < minSize)
                    continue;
                if(keyPtsSize[vCell[j]] > maxSize)
                    continue;

                const float distx = kpUn.pt.x - x;
                const float disty = kpUn.pt.y - y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
        vocabulary->transform(mDescriptors,mBowVec,mFeatVec);
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeStereoMatches(const DescriptorType& descriptorType)
{
    std::cout << "This function (Frame::ComputeStereoMatches) has not been modified yet to work with AnyFeature-VSLAM"<< endl;
    //std::terminate();

    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const Descriptor_Distance_Type thOrbDist = (FeatureMatcher::TH_HIGH + FeatureMatcher::TH_LOW) / Descriptor_Distance_Type(2);

    const int nRows = featureExtractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;

        //const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const float r = 2.0f * GetKeyPtSize(iR); // GetKeyPtSizeRight(iR) !!!!!!!!!!!!!!!!!!!!!!!!!!

        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        Descriptor_Distance_Type bestDist1{FeatureMatcher::TH_HIGH};
        size_t bestIdxR = 0;

        const cv::Mat &descriptorLeft = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &descriptorRight = mDescriptorsRight.row(iR);
                const Descriptor_Distance_Type descDist = FeatureMatcher::DescriptorDistance(descriptorLeft, descriptorRight, descriptorType);

                if(descDist < bestDist1)
                {
                    bestDist1 = descDist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist1 < thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = 1.0f / GetKeyPtSize(iL);
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = featureExtractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= featureExtractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = featureExtractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = GetKeyPtSize(iL) * ((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}


void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

vec3f Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z > 0.0f)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        vec3f x3Dc{x, y, z};
        return Rwc * x3Dc + twc;
    }
    else
        return vec3f{0.0,0.0,-1.0};
}

    float Frame::GetKeyPtSize(const KeypointIndex &keyPtIdx) const {
        return keyPtsSize[keyPtIdx];
    }

    float Frame::GetKeyPt1DSigma2(const KeypointIndex &keyPtIdx) const
    {
        return 0.5f * (keyPtsSigma2[keyPtIdx](0,0) + keyPtsSigma2[keyPtIdx](1,1));
    }

    mat2f Frame::GetKeyPt2DSigma2(const KeypointIndex &keyPtIdx) const
    {
        return keyPtsSigma2[keyPtIdx];
    }

    mat3f Frame::GetKeyPt3DSigma2(const KeypointIndex &keyPtIdx) const
    {
        mat3f sigma2Matrix{mat3f::Zero()};
        sigma2Matrix.block<2,2>(0,0) = keyPtsSigma2[keyPtIdx];
        sigma2Matrix(2,2) = GetKeyPt1DSigma2(keyPtIdx);
        return sigma2Matrix;
    }

    float Frame::GetKeyPt1DInf(const KeypointIndex &keyPtIdx) const
    {
        return 0.5f * (keyPtsInf[keyPtIdx](0,0) + keyPtsInf[keyPtIdx](1,1));
    }

    mat2f Frame::GetKeyPt2DInf(const KeypointIndex &keyPtIdx) const
    {
        return keyPtsInf[keyPtIdx];
    }

    mat3f Frame::GetKeyPt3DInf(const KeypointIndex &keyPtIdx) const
    {
        mat3f infMatrix{mat3f::Zero()};
        infMatrix.block<2,2>(0,0) = keyPtsInf[keyPtIdx];
        infMatrix(2,2) = GetKeyPt1DInf(keyPtIdx);
        return infMatrix;
    }
} //namespace ORB_SLAM
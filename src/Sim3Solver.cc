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


#include "Sim3Solver.h"

#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>

#include "KeyFrame.h"
#include "FeatureMatcher.h"
#include "Converter.h"

#include "DBoW2/Random.h"

namespace ANYFEATURE_VSLAM
{


Sim3Solver::Sim3Solver(Keyframe pKF1, Keyframe pKF2, const vector<Pt> &vpMatched12, const bool bFixScale):
    mnIterations(0), mnBestInliers(0), mbFixScale(bFixScale)
{
    mpKF1 = pKF1;
    mpKF2 = pKF2;

    vector<Pt> vpKeyFrameMP1 = pKF1->GetMapPointMatches();

    mN1 = vpMatched12.size();

    mvpMapPoints1.reserve(mN1);
    mvpMapPoints2.reserve(mN1);
    mvpMatches12 = vpMatched12;
    mvnIndices1.reserve(mN1);
    mvX3Dc1.reserve(mN1);
    mvX3Dc2.reserve(mN1);

    mat3f Rcw1 = pKF1->GetRotation();
    vec3f tcw1 = pKF1->GetTranslation();
    mat3f Rcw2 = pKF2->GetRotation();
    vec3f tcw2 = pKF2->GetTranslation();

    mvAllIndices.reserve(mN1);

    size_t idx=0;
    for(int i1=0; i1<mN1; i1++)
    {
        if(vpMatched12[i1])
        {
            Pt pMP1 = vpKeyFrameMP1[i1];
            Pt pMP2 = vpMatched12[i1];

            if(!pMP1)
                continue;

            if(pMP1->isBad() || pMP2->isBad())
                continue;

            KeypointIndex indexKF1 = pMP1->GetIndexInKeyFrame(pKF1);
            KeypointIndex indexKF2 = pMP2->GetIndexInKeyFrame(pKF2);

            if(indexKF1<0 || indexKF2<0)
                continue;

            const float sigmaSquare1 = pKF1->GetKeyPt1DSigma2(indexKF1);
            const float sigmaSquare2 = pKF2->GetKeyPt1DSigma2(indexKF2);

            mvnMaxError1.push_back(9.210*sigmaSquare1);
            mvnMaxError2.push_back(9.210*sigmaSquare2);

            mvpMapPoints1.push_back(pMP1);
            mvpMapPoints2.push_back(pMP2);
            mvnIndices1.push_back(i1);

            vec3f X3D1w = pMP1->GetWorldPos();
            mvX3Dc1.push_back(Rcw1 * X3D1w + tcw1);

            vec3f X3D2w = pMP2->GetWorldPos();
            mvX3Dc2.push_back(Rcw2 * X3D2w + tcw2);

            mvAllIndices.push_back(idx);
            idx++;
        }
    }

    mK1 = pKF1->mK;
    mK2 = pKF2->mK;

    FromCameraToImage(mvX3Dc1,mvP1im1,mK1);
    FromCameraToImage(mvX3Dc2,mvP2im2,mK2);

    SetRansacParameters();
}

void Sim3Solver::SetRansacParameters(double probability, int minInliers, int maxIterations)
{
    mRansacProb = probability;
    mRansacMinInliers = minInliers;
    mRansacMaxIts = maxIterations;    

    N = mvpMapPoints1.size(); // number of correspondences

    mvbInliersi.resize(N);

    // Adjust Parameters according to number of correspondences
    float epsilon = (float)mRansacMinInliers/N;

    // Set RANSAC iterations according to probability, epsilon, and max iterations
    int nIterations;

    if(mRansacMinInliers==N)
        nIterations=1;
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(epsilon,3)));

    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));

    mnIterations = 0;
}

cv::Mat Sim3Solver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers)
{
    bNoMore = false;
    vbInliers = vector<bool>(mN1,false);
    nInliers=0;

    if(N<mRansacMinInliers)
    {
        bNoMore = true;
        return cv::Mat();
    }

    vector<size_t> vAvailableIndices;

    mat3f P3Dc1i;
    mat3f P3Dc2i;

    int nCurrentIterations = 0;
    while(mnIterations<mRansacMaxIts && nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        mnIterations++;

        vAvailableIndices = mvAllIndices;

        // Get min set of points
        for(short i = 0; i < 3; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];
            P3Dc1i.col(i) = mvX3Dc1[idx];
            P3Dc2i.col(i) = mvX3Dc2[idx];

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        ComputeSim3(P3Dc1i,P3Dc2i);

        CheckInliers();

        if(mnInliersi>=mnBestInliers)
        {
            mvbBestInliers = mvbInliersi;
            mnBestInliers = mnInliersi;
            bestT12 = T12i;
            bestRotation = R12i;
            bestTranslation = t12i;
            bestScale = s12i;

            if(mnInliersi > mRansacMinInliers)
            {
                nInliers = mnInliersi;
                for(int i=0; i<N; i++)
                    if(mvbInliersi[i])
                        vbInliers[mvnIndices1[i]] = true;
                return Converter::toCvMat(bestT12).clone();
            }
        }
    }

    if(mnIterations>=mRansacMaxIts)
        bNoMore=true;

    return cv::Mat();
}

cv::Mat Sim3Solver::find(vector<bool> &vbInliers12, int &nInliers)
{
    bool bFlag;
    return iterate(mRansacMaxIts,bFlag,vbInliers12,nInliers);
}

void Sim3Solver::ComputeCentroid(const mat3f &P, mat3f &Pr, vec3f &C)
{
    C = P.rowwise().sum();
    C = C / P.cols();
    for(int i = 0; i < P.cols(); i++)
        Pr.col(i) = P.col(i) - C;
}

void Sim3Solver::ComputeSim3(mat3f &P1, mat3f &P2)
{
    // Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientation using unit quaternions

    // Step 1: Centroid and relative coordinates
    mat3f Pr1; // Relative coordinates to centroid (set 1)
    mat3f Pr2; // Relative coordinates to centroid (set 2)
    vec3f O1; // Centroid of P1
    vec3f O2; // Centroid of P2

    ComputeCentroid(P1,Pr1,O1);
    ComputeCentroid(P2,Pr2,O2);

    // Step 2: Compute M matrix
    mat3f M = Pr2 * Pr1.transpose();

    // Step 3: Compute N matrix
    mat4f Nmatrix{};
    Nmatrix(0,0) = M(0,0) + M(1,1) + M(2,2);
    Nmatrix(0,1) = M(1,2) - M(2,1);
    Nmatrix(0,2) = M(2,0) - M(0,2);
    Nmatrix(0,3) = M(0,1) - M(1,0);

    Nmatrix(1,0) = Nmatrix(0,1);
    Nmatrix(1,1) = M(0,0) - M(1,1) - M(2,2);
    Nmatrix(1,2) = M(0,1) + M(1,0);
    Nmatrix(1,3) = M(2,0) + M(0,2);

    Nmatrix(2,0) = Nmatrix(0,2);
    Nmatrix(2,1) = Nmatrix(1,2);
    Nmatrix(2,2) = -M(0,0) + M(1,1) - M(2,2);
    Nmatrix(2,3) = M(1,2) + M(2,1);

    Nmatrix(0,3) = Nmatrix(3,0);
    Nmatrix(1,3) = Nmatrix(3,1);
    Nmatrix(2,3) = Nmatrix(3,2);
    Nmatrix(3,3) = -M(0,0) - M(1,1) + M(2,2);

    // Step 4: Eigenvector of the highest eigenvalue
    Eigen::EigenSolver<mat4f> eigSolver;
    eigSolver.compute(Nmatrix);

    vec4f eval = eigSolver.eigenvalues().real();
    mat4f evec = eigSolver.eigenvectors().real();

    int maxIndex;
    eval.maxCoeff(&maxIndex);
    vec3f vec = evec.block<3,1>(1,maxIndex);

    // Rotation angle. sin is the norm of the imaginary part, cos is the real part
    double ang = atan2(vec.norm(),evec(0,maxIndex));
    vec = 2.0 * ang * vec / vec.norm(); //Angle-axis representation. quaternion angle is the half

    cv::Mat mR12i_tmp;
    mR12i_tmp.create(3,3,CV_32F);
    cv::Rodrigues(Converter::toCvMat(vec).clone(),mR12i_tmp); // computes the rotation matrix from angle-axis
    R12i = Converter::toMatrix3f(mR12i_tmp);

    // Step 5: Rotate set 2
    mat3f P3 =  R12i * Pr2;

    // Step 6: Scale

    if(!mbFixScale)
    {
        double nom = Converter::toCvMat(Pr1).dot(Converter::toCvMat(P3));
        cv::Mat aux_P3(cv::Size(3,3),CV_32F);
        aux_P3 = Converter::toCvMat(P3).clone();
        cv::pow(Converter::toCvMat(P3).clone(),2,aux_P3);
        double den = 0;

        for(int i=0; i<aux_P3.rows; i++)
        {
            for(int j=0; j<aux_P3.cols; j++)
            {
                den+=aux_P3.at<float>(i,j);
            }
        }

        s12i = nom/den;
    }
    else
        s12i = 1.0f;

    // Step 7: Translation
    t12i = O1 - s12i * R12i * O2;

    // Step 8: Transformation

    // Step 8.1 T12
    T12i = mat4f::Identity();

    mat3f sR = s12i * R12i;
    T12i.block<3,3>(0,0) = sR;
    T12i.block<3,1>(0,3) = t12i;

    // Step 8.2 T21

    T21i = mat4f::Identity();

    mat3f sRinv = (1.0f / s12i) * R12i.transpose();
    T21i.block<3,3>(0,0) = sRinv;
    vec3f tinv = -sRinv * t12i;
    T21i.block<3,1>(0,3) = tinv;
}


void Sim3Solver::CheckInliers()
{
    vector<cv::Mat> vP1im2, vP2im1;
    Project(mvX3Dc2,vP2im1,T12i,mK1);
    Project(mvX3Dc1,vP1im2,T21i,mK2);

    mnInliersi=0;

    for(size_t i=0; i<mvP1im1.size(); i++)
    {
        cv::Mat dist1 = mvP1im1[i]-vP2im1[i];
        cv::Mat dist2 = vP1im2[i]-mvP2im2[i];

        const float err1 = dist1.dot(dist1);
        const float err2 = dist2.dot(dist2);

        if(err1<mvnMaxError1[i] && err2<mvnMaxError2[i])
        {
            mvbInliersi[i]=true;
            mnInliersi++;
        }
        else
            mvbInliersi[i]=false;
    }
}


mat3f Sim3Solver::GetEstimatedRotation()
{
    return bestRotation;
}

vec3f Sim3Solver::GetEstimatedTranslation()
{
    return bestTranslation;
}

float Sim3Solver::GetEstimatedScale()
{
    return bestScale;
}

void Sim3Solver::Project(const vector<vec3f> &vP3Dw, vector<cv::Mat> &vP2D, const mat4f& Tcw, cv::Mat K)
{
    mat3f Rcw = Tcw.block<3,3>(0,0);
    vec3f tcw = Tcw.block<3,1>(0,3);
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);

    vP2D.clear();
    vP2D.reserve(vP3Dw.size());

    for(size_t i=0, iend=vP3Dw.size(); i<iend; i++)
    {
        vec3f P3Dc = Rcw * vP3Dw[i] + tcw;
        const float invz = 1.0f/(P3Dc(2));
        const float x = P3Dc(0) * invz;
        const float y = P3Dc(1) * invz;

        vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));
    }
}

void Sim3Solver::FromCameraToImage(const vector<vec3f> &vP3Dc, vector<cv::Mat> &vP2D, cv::Mat K)
{
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);

    vP2D.clear();
    vP2D.reserve(vP3Dc.size());

    for(size_t i=0, iend=vP3Dc.size(); i<iend; i++)
    {
        const float invz = 1.0f /(vP3Dc[i](2));
        const float x = vP3Dc[i](0) * invz;
        const float y = vP3Dc[i](1) * invz;

        vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));
    }
}

} //namespace ORB_SLAM

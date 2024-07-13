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

#include "Initializer.h"
#include "Converter.h"
#include "Utils.h"

#include<thread>

namespace ANYFEATURE_VSLAM
{

Initializer::Initializer(const Frame &ReferenceFrame, float sigma_, int iterations)
{
    K = Converter::toMatrix3f(ReferenceFrame.mK.clone());

    keypoints1 = ReferenceFrame.mvKeysUn;

    sigma = sigma_;
    sigma2 = sigma_ * sigma_;
    maxIterations = iterations;
}

bool Initializer::Initialize(const Frame &CurrentFrame, const vector<int> &matches12_,
                             mat3f &R21, vec3f &t21,
                             vector<vec3f> &pts3D, vector<bool> &isTriangulated)
{
    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2
    keypoints2 = CurrentFrame.mvKeysUn;

    matches12.clear();
    matches12.reserve(keypoints2.size());
    matched1.resize(keypoints1.size());
    for(size_t i{0}, iEnd = matches12_.size();i < iEnd; i++)
    {
        if(matches12_[i] >= 0)
        {
            matches12.emplace_back(i,matches12_[i]);
            matched1[i] = true;
        }
        else
            matched1[i] = false;
    }

    const size_t N = matches12.size();

    // Indices for minimum set selection
    vector<size_t> allIndices;
    allIndices.reserve(N);
    vector<size_t> availableIndices;

    for(int i{0}; i < N; i++)
        allIndices.push_back(i);

    // Generate sets of 8 points for each RANSAC iteration
    mvSets = vector< vector<size_t> >(maxIterations,vector<size_t>(8,0));

    for(int it{0}; it < maxIterations; it++)
    {
        availableIndices = allIndices;

        // Select a minimum set
        for(size_t j{0}; j < 8; j++)
        {
            int randi = RandomIntegerGenerator::GetRandomInteger(0,int(availableIndices.size())-1);
            size_t idx = availableIndices[randi];

            mvSets[it][j] = idx;

            availableIndices[randi] = availableIndices.back();
            availableIndices.pop_back();
        }
    }

    // Launch threads to compute in parallel a fundamental matrix and a homography
    vector<bool> matchesInliersH, matchesInliersF;
    float SH, SF;
    mat3f H{},F{};
    thread threadH(&Initializer::FindHomography ,this,ref(matchesInliersH), ref(SH), ref(H));
    thread threadF(&Initializer::FindFundamental,this,ref(matchesInliersF), ref(SF), ref(F));

    // Wait until both threads have finished
    threadH.join();
    threadF.join();

    // Compute ratio of scores
    float RH = SH / (SH + SF);

    // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
    if(RH > minRH)
        return ReconstructH(matchesInliersH,H,K,R21,t21,pts3D,isTriangulated,minParallax,minTriangulated);

    else //if(pF_HF>0.6)
        return ReconstructF(matchesInliersF,F,K,R21,t21,pts3D,isTriangulated,minParallax,minTriangulated);

}


void Initializer::FindHomography(vector<bool> &matchesInliers, float &score, mat3f &H21)
{
    // Number of putative matches
    const size_t N = matches12.size();

    // Normalize coordinates
    vector<vec2f> Pn1, Pn2;
    mat3f T1, T2;
    Normalize(keypoints1,Pn1, T1);
    Normalize(keypoints2,Pn2, T2);
    mat3f T2inv = T2.inverse();

    // Best Results variables
    score = 0.f;
    matchesInliers = vector<bool>(N,false);

    // Iteration variables
    vector<vec2f> Pn1i(8), Pn2i(8);
    mat3f H21i, H12i;
    vector<bool> currentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for(int it{0}; it < maxIterations; it++)
    {
        // Select a minimum set
        for(size_t j{0}; j < 8; j++)
        {
            size_t idx = mvSets[it][j];

            Pn1i[j] = Pn1[matches12[idx].first];
            Pn2i[j] = Pn2[matches12[idx].second];
        }

        mat3f Hn = ComputeH21(Pn1i,Pn2i);
        H21i = T2inv * Hn * T1;
        H12i = H21i.inverse();

        currentScore = CheckHomography(H21i, H12i, currentInliers, sigma);

        if(currentScore > score)
        {
            H21 = H21i;
            matchesInliers = currentInliers;
            score = currentScore;
        }
    }
}


void Initializer::FindFundamental(vector<bool> &matchesInliers, float &score, mat3f &F21)
{
    // Number of putative matches
    const size_t N = matches12.size();

    // Normalize coordinates
    vector<vec2f> Pn1, Pn2;
    mat3f T1, T2;
    Normalize(keypoints1,Pn1, T1);
    Normalize(keypoints2,Pn2, T2);
    mat3f T2t = T2.transpose();

    // Best Results variables
    score = 0.0;
    matchesInliers = vector<bool>(N,false);

    // Iteration variables
    vector<vec2f> Pn1i(8), Pn2i(8);
    mat3f F21i;
    vector<bool> currentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for(int it{0}; it < maxIterations; it++)
    {
        // Select a minimum set
        for(int j{0}; j < 8; j++)
        {
            size_t idx = mvSets[it][j];

            Pn1i[j] = Pn1[matches12[idx].first];
            Pn2i[j] = Pn2[matches12[idx].second];
        }

        mat3f Fn = ComputeF21(Pn1i,Pn2i);

        F21i = T2t * Fn * T1;

        currentScore = CheckFundamental(F21i, currentInliers, sigma);

        if(currentScore > score)
        {
            F21 = F21i;
            matchesInliers = currentInliers;
            score = currentScore;
        }
    }
}


mat3f Initializer::ComputeH21(const vector<vec2f> &P1, const vector<vec2f> &P2)
{
    const size_t N = P1.size();

    Eigen::MatrixXf A(2 * N, 9);

    for(int i{0}; i < N; i++)
    {
        const float u1 = P1[i](0);
        const float v1 = P1[i](1);
        const float u2 = P2[i](0);
        const float v2 = P2[i](1);

        int row = 2 * i;
        A(row,0) = 0.f;
        A(row,1) = 0.f;
        A(row,2) = 0.f;
        A(row,3) = -u1;
        A(row,4) = -v1;
        A(row,5) = -1.f;
        A(row,6) = v2 * u1;
        A(row,7) = v2 * v1;
        A(row,8) = v2;

        row++;
        A(row,0) = u1;
        A(row,1) = v1;
        A(row,2) = 1.f;
        A(row,3) = 0.f;
        A(row,4) = 0.f;
        A(row,5) = 0.f;
        A(row,6) = -u2 * u1;
        A(row,7) = -u2 * v1;
        A(row,8) = -u2;
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullV);
    Eigen::Matrix<float,3,3,Eigen::RowMajor> H(svd.matrixV().col(8).data());

    return H;
}

mat3f Initializer::ComputeF21(const vector<vec2f> &P1,const vector<vec2f> &P2)
{
    const size_t N = P1.size();

    Eigen::MatrixXf A(N, 9);

    for(int i{0}; i < N; i++)
    {
        const float u1 = P1[i](0);
        const float v1 = P1[i](1);
        const float u2 = P2[i](0);
        const float v2 = P2[i](1);

        A(i,0) = u2 * u1;
        A(i,1) = u2 * v1;
        A(i,2) = u2;
        A(i,3) = v2 * u1;
        A(i,4) = v2 * v1;
        A(i,5) = v2;
        A(i,6) = u1;
        A(i,7) = v1;
        A(i,8) = 1.f;
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix<float,3,3,Eigen::RowMajor> Fpre(svd.matrixV().col(8).data());

    Eigen::JacobiSVD<Eigen::Matrix3f> svd2(Fpre, Eigen::ComputeFullU | Eigen::ComputeFullV);

    vec3f w = svd2.singularValues();
    w(2) = 0.f;

    return svd2.matrixU() * Eigen::DiagonalMatrix<float,3>(w) * svd2.matrixV().transpose();
}

float Initializer::CheckHomography(const mat3f &H21, const mat3f &H12, vector<bool> &matchesInliers, const float& sigma_)
{   
    const size_t N = matches12.size();
    matchesInliers.resize(N);

    const float h11 = H21(0,0);
    const float h12 = H21(0,1);
    const float h13 = H21(0,2);
    const float h21 = H21(1,0);
    const float h22 = H21(1,1);
    const float h23 = H21(1,2);
    const float h31 = H21(2,0);
    const float h32 = H21(2,1);
    const float h33 = H21(2,2);

    const float h11inv = H12(0,0);
    const float h12inv = H12(0,1);
    const float h13inv = H12(0,2);
    const float h21inv = H12(1,0);
    const float h22inv = H12(1,1);
    const float h23inv = H12(1,2);
    const float h31inv = H12(2,0);
    const float h32inv = H12(2,1);
    const float h33inv = H12(2,2);

    float score{0.f};

    const float invSigmaSquare = 1.f/(sigma_ * sigma_);

    for(int i{0}; i < N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = keypoints1[matches12[i].first];
        const cv::KeyPoint &kp2 = keypoints2[matches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in first image
        // x2in1 = H12*x2

        const float w2in1inv = 1.f/(h31inv*u2+h32inv*v2+h33inv);
        const float u2in1 = (h11inv*u2+h12inv*v2+h13inv)*w2in1inv;
        const float v2in1 = (h21inv*u2+h22inv*v2+h23inv)*w2in1inv;

        const float squareDist1 = (u1-u2in1)*(u1-u2in1)+(v1-v2in1)*(v1-v2in1);

        const float chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1 > chiSquare_th)
            bIn = false;
        else
            score += chiSquare_th - chiSquare1;

        // Reprojection error in second image
        // x1in2 = H21*x1

        const float w1in2inv = 1.f / (h31*u1+h32*v1+h33);
        const float u1in2 = (h11*u1+h12*v1+h13) * w1in2inv;
        const float v1in2 = (h21*u1+h22*v1+h23) * w1in2inv;

        const float squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);

        const float chiSquare2 = squareDist2 * invSigmaSquare;

        if(chiSquare2 > chiSquare_th)
            bIn = false;
        else
            score += chiSquare_th - chiSquare2;

        if(bIn)
            matchesInliers[i] = true;
        else
            matchesInliers[i] = false;
    }

    return score;
}

float Initializer::CheckFundamental(const mat3f &F21, vector<bool> &matchesInliers, const float& sigma_)
{
    const size_t N = matches12.size();

    const float f11 = F21(0,0);
    const float f12 = F21(0,1);
    const float f13 = F21(0,2);
    const float f21 = F21(1,0);
    const float f22 = F21(1,1);
    const float f23 = F21(1,2);
    const float f31 = F21(2,0);
    const float f32 = F21(2,1);
    const float f33 = F21(2,2);

    matchesInliers.resize(N);

    float score = 0.f;
    const float invSigmaSquare = 1.f / (sigma_ * sigma_);

    for(int i{0}; i < N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = keypoints1[matches12[i].first];
        const cv::KeyPoint &kp2 = keypoints2[matches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in second image
        // l2=F21x1=(a2,b2,c2)

        const float a2 = f11*u1+f12*v1+f13;
        const float b2 = f21*u1+f22*v1+f23;
        const float c2 = f31*u1+f32*v1+f33;

        const float num2 = a2*u2+b2*v2+c2;

        const float squareDist1 = num2*num2/(a2*a2+b2*b2);

        const float chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1 > chiSquare_th_fund)
            bIn = false;
        else
            score += chiSquare_thScore_fund - chiSquare1;

        // Reprojection error in second image
        // l1 =x2tF21=(a1,b1,c1)

        const float a1 = f11*u2+f21*v2+f31;
        const float b1 = f12*u2+f22*v2+f32;
        const float c1 = f13*u2+f23*v2+f33;

        const float num1 = a1*u1+b1*v1+c1;

        const float squareDist2 = num1*num1/(a1*a1+b1*b1);

        const float chiSquare2 = squareDist2*invSigmaSquare;

        if(chiSquare2 > chiSquare_th_fund)
            bIn = false;
        else
            score += chiSquare_thScore_fund - chiSquare2;

        if(bIn)
            matchesInliers[i] = true;
        else
            matchesInliers[i] = false;
    }

    return score;
}

bool Initializer::ReconstructF(const vector<bool> &inliers,
                               const mat3f &F21, const mat3f &K_,
                               mat3f &R21, vec3f &t21,
                               vector<vec3f> &pts3D, vector<bool> &isTriangulated,
                               const float& minParallax_, const int& minTriangulated_)
{
    // Count inliers
    float N{0};
    for(size_t i{0}, iEnd = inliers.size() ; i < iEnd; i++)
        if(inliers[i])
            N++;

    // Compute Essential Matrix from Fundamental Matrix
    mat3f E21 = K_.transpose() * F21 * K_;

    // Recover the 4 motion hypotheses
    mat3f R1{}, R2{};
    vec3f t{};
    DecomposeE(E21,R1,R2,t);
    vec3f t1{t}, t2{-t};

    // Reconstruct with the 4 hyphoteses and check
    vector<vec3f> pts3D_1, pts3D_2, pts3D_3, pts3D_4;
    vector<bool> isTriangulated_1,isTriangulated_2,isTriangulated_3, isTriangulated_4;
    float parallax1,parallax2, parallax3, parallax4;

    int nGood1 = CheckRT(R1,t1,keypoints1,keypoints2,matches12,inliers,K_, pts3D_1, xSigma2 * sigma2, isTriangulated_1, parallax1);
    int nGood2 = CheckRT(R2,t1,keypoints1,keypoints2,matches12,inliers,K_, pts3D_2, xSigma2 * sigma2, isTriangulated_2, parallax2);
    int nGood3 = CheckRT(R1,t2,keypoints1,keypoints2,matches12,inliers,K_, pts3D_3, xSigma2 * sigma2, isTriangulated_3, parallax3);
    int nGood4 = CheckRT(R2,t2,keypoints1,keypoints2,matches12,inliers,K_, pts3D_4, xSigma2 * sigma2, isTriangulated_4, parallax4);

    int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

    // If there is not a clear winner or not enough triangulated points reject initialization
    {
        int nMinGood = max(static_cast<int>(percMinGood * N),minTriangulated_);

        int nsimilar{0};
        if(float(nGood1) > percMaxGood * float(maxGood))
            nsimilar++;
        if(float(nGood2) > percMaxGood * float(maxGood))
            nsimilar++;
        if(float(nGood3) > percMaxGood * float(maxGood))
            nsimilar++;
        if(float(nGood4) > percMaxGood * float(maxGood))
            nsimilar++;

        if(maxGood < nMinGood || nsimilar > 1)
            return false;
    }

    // If best reconstruction has enough parallax initialize
    if(maxGood == nGood1)
    {
        if(parallax1 > minParallax_)
        {
            pts3D = pts3D_1;
            isTriangulated = isTriangulated_1;
            R21 = R1;
            t21 = t1;
            return true;
        }
    }else if(maxGood==nGood2)
    {
        if(parallax2 > minParallax_)
        {
            pts3D = pts3D_2;
            isTriangulated = isTriangulated_2;
            R21 = R2;
            t21 = t1;
            return true;
        }
    }else if(maxGood == nGood3)
    {
        if(parallax3 > minParallax_)
        {
            pts3D = pts3D_3;
            isTriangulated = isTriangulated_3;
            R21 = R1;
            t21 = t2;
            return true;
        }
    }else if(maxGood == nGood4)
    {
        if(parallax4 > minParallax_)
        {
            pts3D = pts3D_4;
            isTriangulated = isTriangulated_4;
            R21 = R2;
            t21 = t2;
            return true;
        }
    }

    return false;
}

bool Initializer::ReconstructH(const vector<bool> &inliers,
                               const mat3f &H21, const mat3f &K_,
                               mat3f &R21, vec3f &t21,
                               vector<vec3f> &pts3D, vector<bool> &isTriangulated,
                               const float &minParallax_, const int &minTriangulated_)
{

    // Count inliers
    int N=0;
    for(bool inlier : inliers)
        if(inlier)
            N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988

    mat3f invK = K_.inverse();
    mat3f A = invK * H21 * K_;

    Eigen::JacobiSVD<mat3f> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const mat3f& U = svd.matrixU();
    mat3f V = svd.matrixV();
    mat3f Vt = V.transpose();
    vec3f w = svd.singularValues();

    float s = U.determinant() * Vt.determinant();

    float d1{w(0)},d2{w(1)},d3{w(2)};
    if((d1/d2) < dRatio || (d2/d3) < dRatio)
        return false;

    vector<mat3f> vR;
    vector<vec3f> vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
    vec4f x1{aux1,aux1,-aux1,-aux1};
    vec4f x3{aux3,-aux3,aux3,-aux3};

    //case d'=d2
    float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

    float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
    vec4f stheta{aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    for(int i = 0; i < 4; i++)
    {
        mat3f Rp{mat3f::Identity()};
        Rp(0,0) = ctheta;
        Rp(0,2) = -stheta(i);
        Rp(2,0) = stheta(i);
        Rp(2,2) = ctheta;

        mat3f R = s * U * Rp * Vt;
        vR.push_back(R);

        vec3f tp{x1(i),0.f,-x3(i)};
        tp *= d1-d3;

        vec3f t = U * tp;
        vt.emplace_back(t / t.norm());

        vec3f np{x1(i),0.f,x3(i)};

        vec3f n = V * np;
        if(n(2) < 0)
            n = -n;
        vn.push_back(n);
    }

    //case d'=-d2
    float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

    float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
    vec4f sphi{aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for(int i{0}; i < 4; i++)
    {
        mat3f Rp{mat3f::Identity()};
        Rp(0,0) = cphi;
        Rp(0,2) = sphi(i);
        Rp(1,1) = -1.f;
        Rp(2,0) = sphi(i);
        Rp(2,2) = -cphi;

        mat3f R = s * U * Rp * Vt;
        vR.push_back(R);

        vec3f tp{x1(i),0.f,x3(i)};
        tp *= d1 + d3;

        vec3f t = U * tp;
        vt.emplace_back(t / t.norm());

        vec3f np{x1(i),0.f,x3(i)};

        vec3f n = V * np;
        if(n(2) < 0.f)
            n = -n;
        vn.push_back(n);
    }


    int bestGood = 0;
    int secondBestGood = 0;    
    int bestSolutionIdx = -1;
    float bestParallax = -1;
    vector<vec3f> bestP3D;
    vector<bool> bestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    for(size_t i{0}; i < 8; i++)
    {
        float parallaxi;
        vector<vec3f> P3Di;
        vector<bool> triangulatedi;
        int nGood = CheckRT(vR[i],vt[i],keypoints1,keypoints2,matches12,inliers,K_,P3Di, xSigma2 * sigma2, triangulatedi, parallaxi);

        if(nGood > bestGood)
        {
            secondBestGood = bestGood;
            bestGood = nGood;
            bestSolutionIdx = int(i);
            bestParallax = parallaxi;
            bestP3D = P3Di;
            bestTriangulated = triangulatedi;
        }
        else if(nGood > secondBestGood)
        {
            secondBestGood = nGood;
        }
    }


    if(float(secondBestGood) < percSecondBestGood * float(bestGood)
        && bestParallax >= minParallax_
        && float(bestGood) > float(minTriangulated_)
        && float(bestGood) > percBestGood * float(N))
    {
        R21 = vR[bestSolutionIdx];
        t21 = vt[bestSolutionIdx];
        pts3D = bestP3D;
        isTriangulated = bestTriangulated;

        return true;
    }

    return false;
}

bool Initializer::Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                              mat34f &Tc1w ,mat34f &Tc2w , vec3f &x3D)
{
    mat4f A{};
    A.block<1,4>(0,0) = kp1.pt.x * Tc1w.block<1,4>(2,0) - Tc1w.block<1,4>(0,0);
    A.block<1,4>(1,0) = kp1.pt.y * Tc1w.block<1,4>(2,0) - Tc1w.block<1,4>(1,0);
    A.block<1,4>(2,0) = kp2.pt.x * Tc2w.block<1,4>(2,0) - Tc2w.block<1,4>(0,0);
    A.block<1,4>(3,0) = kp2.pt.y * Tc2w.block<1,4>(2,0) - Tc2w.block<1,4>(1,0);

    Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);

    Eigen::Vector4f x3Dh = svd.matrixV().col(3);

    if(x3Dh(3) == 0)
        return false;

    // Euclidean coordinates
    x3D = x3Dh.head(3) / x3Dh(3);

    return true;
}

void Initializer::Normalize(const vector<cv::KeyPoint> &keypoints, vector<vec2f> &normalizedPoints, mat3f &T)
{
    const size_t N = keypoints.size();
    normalizedPoints.resize(N);

    float meanX{0.f}, meanY{0.f};
    for(int i{0}; i < N; i++)
    {
        meanX += keypoints[i].pt.x;
        meanY += keypoints[i].pt.y;
    }
    meanX = meanX / float(N);
    meanY = meanY / float(N);

    float meanDevX{0.f}, meanDevY{0.f};
    for(int i{0}; i < N; i++)
    {
        normalizedPoints[i](0) = keypoints[i].pt.x - meanX;
        normalizedPoints[i](1) = keypoints[i].pt.y - meanY;

        meanDevX += fabs(normalizedPoints[i](0));
        meanDevY += fabs(normalizedPoints[i](1));
    }
    meanDevX = meanDevX / float(N);
    meanDevY = meanDevY / float(N);

    const float sX{1.f / meanDevX};
    const float sY{1.f / meanDevY};

    for(int i{0}; i < N; i++)
    {
        normalizedPoints[i](0) *= sX;
        normalizedPoints[i](1) *= sY;
    }

    T.setZero();
    T(0,0) = sX;
    T(1,1) = sY;
    T(0,2) = -meanX * sX;
    T(1,2) = -meanY * sY;
    T(2,2) = 1.f;

}


int Initializer::CheckRT(const mat3f &R, const vec3f &t,
                         const vector<cv::KeyPoint> &keypoints1_, const vector<cv::KeyPoint> &keypoints2_,
                         const vector<Match> &matches12_, const vector<bool> &inliers,
                         const mat3f &K_, vector<vec3f> &pts3D,
                         const float& squareError_th, vector<bool> &isGood, float& parallax)
{
    // Calibration parameters
    const float fx{K_(0,0)};
    const float fy{K_(1,1)};
    const float cx{K_(0,2)};
    const float cy{K_(1,2)};

    isGood = vector<bool>(keypoints1_.size(),false);
    pts3D.resize(keypoints1_.size());

    vector<float> cosParallax;
    cosParallax.reserve(keypoints1_.size());

    // Camera 1 Projection Matrix K[I|0]
    mat34f P1{mat34f::Zero()};
    P1.block<3,3>(0,0) = K_;

    vec3f O1{vec3f::Zero()};

    // Camera 2 Projection Matrix K[R|t]
    mat34f P2{mat34f::Zero()};
    P2.block<3,3>(0,0) = R;
    P2.block<3,1>(0,3) = t;
    P2 = K_ * P2;

    vec3f O2 = -R.transpose() * t;

    int nGood{0};
    for(size_t i{0}, iEnd = matches12_.size(); i < iEnd; i++)
    {
        if(!inliers[i])
            continue;

        const cv::KeyPoint &kp1 = keypoints1_[matches12_[i].first];
        const cv::KeyPoint &kp2 = keypoints2_[matches12_[i].second];

        Eigen::Vector3f p3dC1;

        Triangulate(kp1,kp2,P1,P2,p3dC1);

        if(!isfinite(p3dC1(0)) || !isfinite(p3dC1(1)) || !isfinite(p3dC1(2)))
        {
            isGood[matches12_[i].first] = false;
            continue;
        }

        // Check parallax
        vec3f normal1 = p3dC1 - O1;
        float dist1 = normal1.norm();

        vec3f normal2 = p3dC1 - O2;
        float dist2 = normal2.norm();

        float cosParallax_i = normal1.dot(normal2) / (dist1 * dist2);
        //float sinParallax_i = normal1.cross(normal2).norm()  / (dist1 * dist2);

        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        if(p3dC1(2) <= 0 && cosParallax_i < minCos)
            continue;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        vec3f p3dC2 = R * p3dC1 + t;
        if(p3dC2(2) <= 0 && cosParallax_i < minCos)
            continue;

        // Check reprojection error in first image
        float im1x, im1y;
        float invZ1 = 1.f / p3dC1(2);
        im1x = fx * p3dC1(0) * invZ1 + cx;
        im1y = fy * p3dC1(1) * invZ1 + cy;

        float squareError1 = (im1x-kp1.pt.x) * (im1x-kp1.pt.x) + (im1y-kp1.pt.y) * (im1y-kp1.pt.y);

        if(squareError1 > squareError_th)
            continue;

        // Check reprojection error in second image
        float im2x, im2y;
        float invZ2 = 1.f / p3dC2(2);
        im2x = fx * p3dC2(0) * invZ2 + cx;
        im2y = fy * p3dC2(1) * invZ2 + cy;

        float squareError2 = (im2x-kp2.pt.x) * (im2x-kp2.pt.x) + (im2y-kp2.pt.y) * (im2y-kp2.pt.y);

        if(squareError2 > squareError_th)
            continue;

        cosParallax.push_back(cosParallax_i);
        pts3D[matches12_[i].first] = vec3f(p3dC1(0),p3dC1(1),p3dC1(2));
        nGood++;

        if(cosParallax_i < minCos)
            isGood[matches12_[i].first] = true;
    }

    if(nGood > 0)
    {
        sort(cosParallax.begin(),cosParallax.end());

        size_t idx = min(parallaxIdx,int(cosParallax.size()-1));
        parallax = acosf(cosParallax[idx]) * 180.f / float(CV_PI);
    }
    else
        parallax = 0.f;

    return nGood;
}

void Initializer::DecomposeE(const mat3f &E, mat3f &R1, mat3f &R2, vec3f &t)
{
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f Vt = svd.matrixV().transpose();

    t = U.col(2);
    t = t / t.norm();

    mat3f W{mat3f::Zero()};
    W(0,1) = -1.f;
    W(1,0) = 1.f;
    W(2,2) = 1.f;

    R1 = U * W * Vt;
    if(R1.determinant() < 0.f)
        R1 = -R1;

    R2 = U * W.transpose() * Vt;
    if(R2.determinant() < 0.f)
        R2 = -R2;
}

} // namespace ANYFEATURE_VSLAM

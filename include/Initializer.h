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

#ifndef INITIALIZER_H
#define INITIALIZER_H

#include <opencv2/opencv.hpp>
#include "Frame.h"


namespace ANYFEATURE_VSLAM
{

// THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
class Initializer
{
    typedef pair<int,int> Match;

public:

    // Fix the reference frame
    Initializer(const Frame &ReferenceFrame, float sigma_, int iterations);

    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    bool Initialize(const Frame &CurrentFrame, const vector<int> &matches12_,
                    mat3f &R21, vec3f &t21,
                    vector<vec3f> &pts3D, vector<bool> &isTriangulated);

private:

    void FindHomography (vector<bool> &matchesInliers, float &score, mat3f &H21);
    void FindFundamental(vector<bool> &matchesInliers, float &score, mat3f &F21);

    static mat3f ComputeH21(const vector<vec2f> &P1, const vector<vec2f> &P2);
    static mat3f ComputeF21(const vector<vec2f> &P1, const vector<vec2f> &P2);

    float CheckHomography (const mat3f &H21, const mat3f &H12, vector<bool> &matchesInliers, const float& sigma);
    float CheckFundamental(const mat3f &F21, vector<bool> &matchesInliers, const float& sigma);

    bool ReconstructF(const vector<bool> &inliers,
                      const mat3f &F21, const mat3f &K_,
                      mat3f &R21, vec3f &t21,
                      vector<vec3f> &pts3D, vector<bool> &isTriangulated,
                      const float &minParallax_, const int &minTriangulated);

    bool ReconstructH(const vector<bool> &inliers,
                      const mat3f &H21, const mat3f &K_,
                      mat3f &R21, vec3f &t21,
                      vector<vec3f> &pts3D, vector<bool> &isTriangulated,
                      const float &minParallax_, const int &minTriangulated);

    static bool Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, mat34f &Tc1w ,mat34f &Tc2w , vec3f &x3D);

    static void Normalize(const vector<cv::KeyPoint> &keypoints, vector<vec2f> &normalizedPoints, mat3f &T);

    int CheckRT(const mat3f &R, const vec3f &t,
                const vector<cv::KeyPoint> &keypoints1_, const vector<cv::KeyPoint> &keypoints2_,
                const vector<Match> &matches12_, const vector<bool> &inliers,
                const mat3f &K_, vector<vec3f> &pts3D,
                const float& squareError_th, vector<bool> &isGood, float& parallax);

    static void DecomposeE(const mat3f &E, mat3f &R1, mat3f &R2, vec3f &t);


    // Keypoints from Reference Frame (Frame 1)
    vector<cv::KeyPoint> keypoints1;

    // Keypoints from Current Frame (Frame 2)
    vector<cv::KeyPoint> keypoints2;

    // Current Matches from Reference to Current
    vector<Match> matches12;
    vector<bool> matched1;

    // Calibration
    mat3f K;

    // Standard Deviation and Variance
    float sigma, sigma2;

    // Ransac max iterations
    int maxIterations;

    // Ransac sets
    vector<vector<size_t> > mvSets;   

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Heuristics

    // Initialize
    const float minRH{0.4};
    const float minParallax{1.0f};
    const int minTriangulated{50};

    // ReconstructF
    const float percMaxGood{0.7f};
    const float xSigma2{4.0f};
    const float percMinGood{0.9f};

    // ReconstructH
    const float dRatio{1.00001f};
    const float percBestGood{0.9f};
    const float percSecondBestGood{0.75f};

    // CheckRT
    const float minCos{0.99998f};
    const int parallaxIdx{50};

    // CheckHomography
    const float chiSquare_th{5.991f};

    // CheckFundamental
    const float chiSquare_th_fund{3.841f};
    const float chiSquare_thScore_fund{5.991f};

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
};

} // namespace ANYFEATURE_VSLAM

#endif // INITIALIZER_H

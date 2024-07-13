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

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "Image.h"

#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"
#include "FeatureVocabulary.h"
#include "KeyFrame.h"
#include "FeatureExtractor.h"
#include "Types.h"
#include "Vocabulary.h"
#include <opencv2/opencv.hpp>

namespace ANYFEATURE_VSLAM
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
typedef shared_ptr<ANYFEATURE_VSLAM::MapPoint> Pt;

class KeyFrame;
typedef shared_ptr<ANYFEATURE_VSLAM::KeyFrame> Keyframe;

class Frame
{
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
          shared_ptr<FeatureExtractor>& extractorLeft, shared_ptr<FeatureExtractor>& extractorRight,
          shared_ptr<Vocabulary> vocabulary, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp,
          shared_ptr<FeatureExtractor>& extractor,
          shared_ptr<Vocabulary> vocabulary, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for mono cameras.
    Frame(const Image & img, const double &timeStamp,
          shared_ptr<FeatureExtractor>& extractor,
          shared_ptr<Vocabulary> vocabulary, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractFeatures(int flag, const Image & img);

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(const mat4f& Tcw_);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline vec3f GetCameraCenter(){
        return twc;
    }

    // Returns inverse of rotation
    inline mat3f GetRotationInverse(){
        return Rwc;
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(Pt pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const float& minSize, const float& maxSize) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches(const DescriptorType& descriptorType);

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    vec3f UnprojectStereo(const int &i);

    [[nodiscard]] float GetKeyPtSize(const KeypointIndex &keyPtIdx) const;
    [[nodiscard]] float GetKeyPt1DSigma2(const KeypointIndex &keyPtIdx) const;
    [[nodiscard]] mat2f GetKeyPt2DSigma2(const KeypointIndex &keyPtIdx) const;
    [[nodiscard]] mat3f GetKeyPt3DSigma2(const KeypointIndex &keyPtIdx) const;
    [[nodiscard]] float GetKeyPt1DInf(const KeypointIndex &keyPtIdx) const;
    [[nodiscard]] mat2f GetKeyPt2DInf(const KeypointIndex &keyPtIdx) const;
    [[nodiscard]] mat3f GetKeyPt3DInf(const KeypointIndex &keyPtIdx) const;

public:
    // Vocabulary used for relocalization.
    shared_ptr<Vocabulary> vocabulary;

    // Feature extractor. The right is used only in the stereo case.
    shared_ptr<FeatureExtractor> featureExtractorLeft, featureExtractorRight;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK; // Remove ???????????????????????
    mat3f K;

    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;

    // Corresponding stereo coordinate and depth for each keypoint.
    // "mono" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<Pt> pts;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    mat4f Tcw{mat4f::Zero()};

    // Current and Next Frame id.
    static long unsigned int nNextId;
    FrameId mnId;

    // Reference Keyframe.
    Keyframe refKeyframe;

    // Scale pyramid info.
    float sizeTolerance{};
    float invSizeTolerance{};
    vector<mat2f> keyPtsSigma2{};
    vector<mat2f> keyPtsInf{};
    vector<float> keyPtsSize{};
    float maxKeyPtSize{};
    float maxKeyPtSigma{};

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    mat3f Rcw;
    vec3f tcw;
    mat3f Rwc;
    vec3f twc;
};

}// namespace ORB_SLAM

#endif // FRAME_H

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

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include<pangolin/pangolin.h>

#include<mutex>

namespace ANYFEATURE_VSLAM
{

class MapDrawer
{
public:
    MapDrawer(shared_ptr<Map> pMap, const string &strSettingPath, const vector<FeatureType>& featureTypes);

    shared_ptr<Map> mpMap;

    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawTrajectory(vec3f& trajectoryCenter, float& cameraHeight);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const mat4f &Tcw_);
    void SetReferenceKeyFrame(Keyframe pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
    void SetCurrentOpenGLCameraMatrix(const mat3f& Rwc,const vec3f& twc, pangolin::OpenGlMatrix &M);
    void AddLoopClosureKeyframe(const mat4f &Tcw_);

private:

    float mKeyFrameSize{0.05f};
    float mKeyFrameLineWidth{1.0f};
    float mGraphLineWidth{0.9f};
    float mPointSize{2.0f};
    float mCameraSize{0.08f};
    float mCameraLineWidth{3.0f};

    std::mutex mMutexCamera;
    mat4f mCameraPose{mat4f::Zero()};
    std::mutex mutexLoopClosures;
    std::vector<vec3f>loopClosures{};

    std::vector<vec3f> trajectory{};
    vector<FeatureType> featureTypes{};
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
